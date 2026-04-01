import argparse
import asyncio
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw


@dataclass
class DroneConfig:
    name: str
    role: str
    mavlink_udp: str
    source_ned_m: np.ndarray
    destination_ned_m: np.ndarray
    desired_offset_from_leader_m: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=float)
    )


@dataclass
class SwarmParams:
    control_rate_hz: float
    neighbor_range_m: float
    arrival_radius_m: float
    max_speed_mps: float
    k_sep: float
    k_coh: float
    k_ali: float
    k_mig: float
    k_follow: float
    min_sep_distance_m: float


@dataclass
class DroneState:
    position_ned_m: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    velocity_ned_mps: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    telemetry_ready: bool = False


class Px4SwarmLeaderFollower:
    def __init__(self, config_path: Path):
        self.config_path = config_path
        self.params: Optional[SwarmParams] = None
        self.drone_cfgs: List[DroneConfig] = []
        self.systems: Dict[str, System] = {}
        self.states: Dict[str, DroneState] = {}
        self.leader_name: Optional[str] = None
        self._should_stop = False

    def load_config(self) -> None:
        with self.config_path.open("r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        swarm_cfg = cfg["swarm"]
        self.params = SwarmParams(
            control_rate_hz=float(swarm_cfg["control_rate_hz"]),
            neighbor_range_m=float(swarm_cfg["neighbor_range_m"]),
            arrival_radius_m=float(swarm_cfg["arrival_radius_m"]),
            max_speed_mps=float(swarm_cfg["max_speed_mps"]),
            k_sep=float(swarm_cfg["k_sep"]),
            k_coh=float(swarm_cfg["k_coh"]),
            k_ali=float(swarm_cfg["k_ali"]),
            k_mig=float(swarm_cfg["k_mig"]),
            k_follow=float(swarm_cfg["k_follow"]),
            min_sep_distance_m=float(swarm_cfg["min_sep_distance_m"]),
        )

        self.drone_cfgs = []
        for d in cfg["drones"]:
            drone_cfg = DroneConfig(
                name=str(d["name"]),
                role=str(d["role"]).strip().lower(),
                mavlink_udp=str(d["mavlink_udp"]),
                source_ned_m=np.array(d["source_ned_m"], dtype=float),
                destination_ned_m=np.array(d["destination_ned_m"], dtype=float),
                desired_offset_from_leader_m=np.array(
                    d.get("desired_offset_from_leader_m", [0.0, 0.0, 0.0]), dtype=float
                ),
            )
            self.drone_cfgs.append(drone_cfg)

        leaders = [d.name for d in self.drone_cfgs if d.role == "leader"]
        if len(leaders) != 1:
            raise ValueError("Configuration must include exactly one leader drone.")
        self.leader_name = leaders[0]

    async def connect_and_prepare(self) -> None:
        assert self.params is not None
        for cfg in self.drone_cfgs:
            system = System()
            self.systems[cfg.name] = system
            self.states[cfg.name] = DroneState()

            print(f"[connect] {cfg.name} -> {cfg.mavlink_udp}")
            await system.connect(system_address=cfg.mavlink_udp)

            async for state in system.core.connection_state():
                if state.is_connected:
                    print(f"[connected] {cfg.name}")
                    break

        await asyncio.gather(*(self._start_telemetry_task(cfg.name) for cfg in self.drone_cfgs))

        print("[wait] Waiting for telemetry readiness from all drones...")
        while not all(self.states[cfg.name].telemetry_ready for cfg in self.drone_cfgs):
            await asyncio.sleep(0.1)

        print("[arm/offboard] Arming and starting offboard mode...")
        for cfg in self.drone_cfgs:
            drone = self.systems[cfg.name]
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await drone.action.arm()
            try:
                await drone.offboard.start()
            except OffboardError as e:
                await drone.action.disarm()
                raise RuntimeError(
                    f"Failed to start offboard mode for {cfg.name}: {e}"
                ) from e

        print("[ready] Swarm controller is active.")

    async def _start_telemetry_task(self, drone_name: str) -> None:
        asyncio.create_task(self._telemetry_loop(drone_name))

    async def _telemetry_loop(self, drone_name: str) -> None:
        drone = self.systems[drone_name]
        async for pv in drone.telemetry.position_velocity_ned():
            self.states[drone_name].position_ned_m = np.array(
                [
                    pv.position.north_m,
                    pv.position.east_m,
                    pv.position.down_m,
                ],
                dtype=float,
            )
            self.states[drone_name].velocity_ned_mps = np.array(
                [
                    pv.velocity.north_m_s,
                    pv.velocity.east_m_s,
                    pv.velocity.down_m_s,
                ],
                dtype=float,
            )
            self.states[drone_name].telemetry_ready = True

    def _neighbors(self, me: str) -> List[str]:
        assert self.params is not None
        p_me = self.states[me].position_ned_m
        neighbors: List[str] = []
        for other in self.states:
            if other == me:
                continue
            dist = np.linalg.norm(self.states[other].position_ned_m - p_me)
            if dist <= self.params.neighbor_range_m:
                neighbors.append(other)
        return neighbors

    def _flocking_velocity(self, me: str, neighbors: List[str]) -> np.ndarray:
        assert self.params is not None

        p_i = self.states[me].position_ned_m
        v_i = self.states[me].velocity_ned_mps

        if not neighbors:
            return np.zeros(3, dtype=float)

        sep = np.zeros(3, dtype=float)
        neighbor_positions = []
        neighbor_velocities = []

        for n in neighbors:
            p_j = self.states[n].position_ned_m
            v_j = self.states[n].velocity_ned_mps
            delta = p_i - p_j
            dist = np.linalg.norm(delta)
            if dist < 1e-6:
                continue

            safe_dist = max(dist, self.params.min_sep_distance_m)
            sep += delta / (safe_dist * safe_dist)
            neighbor_positions.append(p_j)
            neighbor_velocities.append(v_j)

        if not neighbor_positions:
            return np.zeros(3, dtype=float)

        mean_pos = np.mean(np.array(neighbor_positions), axis=0)
        mean_vel = np.mean(np.array(neighbor_velocities), axis=0)

        v_sep = self.params.k_sep * sep
        v_coh = self.params.k_coh * (mean_pos - p_i)
        v_ali = self.params.k_ali * (mean_vel - v_i)
        return v_sep + v_coh + v_ali

    def _leader_velocity(self, cfg: DroneConfig, neighbors: List[str]) -> np.ndarray:
        assert self.params is not None

        p = self.states[cfg.name].position_ned_m
        v_flock = self._flocking_velocity(cfg.name, neighbors)
        v_mig = self.params.k_mig * (cfg.destination_ned_m - p)
        return v_flock + v_mig

    def _follower_velocity(self, cfg: DroneConfig, neighbors: List[str]) -> np.ndarray:
        assert self.params is not None
        assert self.leader_name is not None

        p_f = self.states[cfg.name].position_ned_m
        p_l = self.states[self.leader_name].position_ned_m

        v_flock = self._flocking_velocity(cfg.name, neighbors)
        v_dest = self.params.k_mig * (cfg.destination_ned_m - p_f)

        # Formation control keeps follower aligned relative to the leader.
        v_follow = self.params.k_follow * (p_l + cfg.desired_offset_from_leader_m - p_f)
        return v_flock + v_dest + v_follow

    def _bounded(self, v: np.ndarray) -> np.ndarray:
        assert self.params is not None
        speed = np.linalg.norm(v)
        if speed < 1e-6:
            return v
        if speed <= self.params.max_speed_mps:
            return v
        return (v / speed) * self.params.max_speed_mps

    async def _publish_velocity(self, drone_name: str, velocity_ned: np.ndarray) -> None:
        cmd = VelocityNedYaw(
            float(velocity_ned[0]),
            float(velocity_ned[1]),
            float(velocity_ned[2]),
            0.0,
        )
        await self.systems[drone_name].offboard.set_velocity_ned(cmd)

    async def run(self) -> None:
        assert self.params is not None

        dt = 1.0 / self.params.control_rate_hz
        while not self._should_stop:
            all_reached = True
            publish_tasks = []

            for cfg in self.drone_cfgs:
                p = self.states[cfg.name].position_ned_m
                dist_to_goal = float(np.linalg.norm(cfg.destination_ned_m - p))

                neighbors = self._neighbors(cfg.name)
                if cfg.role == "leader":
                    v = self._leader_velocity(cfg, neighbors)
                else:
                    v = self._follower_velocity(cfg, neighbors)

                if dist_to_goal <= self.params.arrival_radius_m:
                    v = np.zeros(3, dtype=float)
                else:
                    all_reached = False

                v = self._bounded(v)
                publish_tasks.append(self._publish_velocity(cfg.name, v))

            await asyncio.gather(*publish_tasks)

            if all_reached:
                print("[complete] All drones reached destination radius.")
                self._should_stop = True

            await asyncio.sleep(dt)

    async def shutdown(self) -> None:
        print("[shutdown] Stopping offboard and disarming.")
        for cfg in self.drone_cfgs:
            drone = self.systems[cfg.name]
            try:
                await drone.offboard.stop()
            except OffboardError:
                pass
            try:
                await drone.action.disarm()
            except Exception:
                pass


async def async_main(config_path: Path) -> None:
    controller = Px4SwarmLeaderFollower(config_path)
    controller.load_config()

    try:
        await controller.connect_and_prepare()
        await controller.run()
    finally:
        await controller.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="PX4 + Gazebo leader-follower swarm controller using local interaction rules."
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config.yaml",
        help="Path to YAML configuration file.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(args.config).resolve()
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    try:
        asyncio.run(async_main(config_path))
    except KeyboardInterrupt:
        print("\n[exit] Interrupted by user.")


if __name__ == "__main__":
    main()
