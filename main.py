import argparse
import asyncio
import logging
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
    mavsdk_server_port: int
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
    filtered_cmd_ned_mps: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=float)
    )
    telemetry_ready: bool = False
    vehicle_uuid: Optional[int] = None


class LeaderFollowerSwarmController:
    def __init__(self, config_path: Path):
        self.config_path = config_path
        self.logger = logging.getLogger("px4_swarm")

        self.drone_cfgs: List[DroneConfig] = []
        self.params: Optional[SwarmParams] = None
        self.leader_name: Optional[str] = None

        self.systems: Dict[str, System] = {}
        self.states: Dict[str, DroneState] = {}
        self._telemetry_tasks: Dict[str, asyncio.Task] = {}
        self._used_vehicle_uuids: set[int] = set()

        self.connect_timeout_sec = 12.0
        self.connect_call_timeout_sec = 8.0
        self.log_every_n_cycles = 10

        self._stop_requested = False

    def _setup_logging(self) -> None:
        self.logger.setLevel(logging.DEBUG)
        self.logger.propagate = False

        if self.logger.handlers:
            return

        formatter = logging.Formatter(
            fmt="%(asctime)s | %(levelname)s | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )

        console = logging.StreamHandler()
        console.setLevel(logging.INFO)
        console.setFormatter(formatter)
        self.logger.addHandler(console)

        log_path = self.config_path.parent / "swarm_debug.log"
        file_handler = logging.FileHandler(log_path, encoding="utf-8")
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

        self.logger.info("Logging to %s", log_path)

    def load_config(self) -> None:
        self._setup_logging()

        with self.config_path.open("r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        self.connect_timeout_sec = float(
            cfg.get("mavsdk", {}).get("action_timeout_sec", 12.0)
        )
        self.connect_call_timeout_sec = float(
            cfg.get("mavsdk", {}).get("connect_call_timeout_sec", 8.0)
        )
        self.log_every_n_cycles = int(
            cfg.get("logging", {}).get("log_every_n_cycles", 10)
        )

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
                mavsdk_server_port=int(d.get("mavsdk_server_port", 50051 + len(self.drone_cfgs))),
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

        self.logger.info(
            "Loaded config: drones=%d leader=%s control_rate=%.2fHz connect_timeout=%.1fs connect_call_timeout=%.1fs",
            len(self.drone_cfgs),
            self.leader_name,
            self.params.control_rate_hz,
            self.connect_timeout_sec,
            self.connect_call_timeout_sec,
        )

    @staticmethod
    def _parse_udp_port(endpoint: str) -> Optional[int]:
        prefix = "udp://:"
        if not endpoint.startswith(prefix):
            return None
        try:
            return int(endpoint[len(prefix):])
        except ValueError:
            return None

    def _endpoint_patterns(self) -> List[List[str]]:
        configured = [cfg.mavlink_udp for cfg in self.drone_cfgs]

        known_patterns = [
            [f"udp://:{14560 + i + 1}" for i in range(len(self.drone_cfgs))],
            [f"udp://:{14560 + i}" for i in range(len(self.drone_cfgs))],
            [f"udp://:{14540 + i + 1}" for i in range(len(self.drone_cfgs))],
            [f"udp://:{14540 + i}" for i in range(len(self.drone_cfgs))],
        ]

        patterns: List[List[str]] = [configured]
        for p in known_patterns:
            if p != configured and p not in patterns:
                patterns.append(p)
        return patterns

    async def _wait_until_connected(self, system: System):
        async for state in system.core.connection_state():
            if state.is_connected:
                return state

    async def _connect_exact_endpoint(self, cfg: DroneConfig, endpoint: str) -> bool:
        system = System(port=cfg.mavsdk_server_port)
        self.logger.info(
            "[connect] %s -> %s (grpc_port=%d)",
            cfg.name,
            endpoint,
            cfg.mavsdk_server_port,
        )

        try:
            await asyncio.wait_for(
                system.connect(system_address=endpoint),
                timeout=self.connect_call_timeout_sec,
            )
        except asyncio.TimeoutError:
            self.logger.warning(
                "[warn] connect() timeout for %s on %s", cfg.name, endpoint
            )
            return False
        except Exception:
            self.logger.exception(
                "[error] connect() failed for %s on %s", cfg.name, endpoint
            )
            return False

        try:
            conn_state = await asyncio.wait_for(
                self._wait_until_connected(system), timeout=self.connect_timeout_sec
            )
        except asyncio.TimeoutError:
            self.logger.warning(
                "[warn] connection_state timeout for %s on %s", cfg.name, endpoint
            )
            return False
        except Exception:
            self.logger.exception(
                "[error] connection_state() failed for %s on %s", cfg.name, endpoint
            )
            return False

        vehicle_uuid = getattr(conn_state, "uuid", None)
        if vehicle_uuid is not None and vehicle_uuid in self._used_vehicle_uuids:
            self.logger.warning(
                "[warn] duplicate uuid=%s for %s on %s",
                vehicle_uuid,
                cfg.name,
                endpoint,
            )
            return False

        if vehicle_uuid is not None:
            self._used_vehicle_uuids.add(vehicle_uuid)

        cfg.mavlink_udp = endpoint
        self.states[cfg.name] = DroneState(vehicle_uuid=vehicle_uuid)
        self.systems[cfg.name] = system
        self.logger.info("[connected] %s on %s uuid=%s", cfg.name, endpoint, vehicle_uuid)
        return True

    def _reset_connections(self) -> None:
        self.systems = {}
        self.states = {}
        self._used_vehicle_uuids = set()

    async def _stop_mavsdk_backends(self) -> None:
        if not self.systems:
            return

        for system in self.systems.values():
            try:
                stop_fn = getattr(system, "_stop_mavsdk_server", None)
                if callable(stop_fn):
                    stop_fn()
            except Exception:
                pass

        await asyncio.sleep(0.2)

    async def _stop_telemetry(self) -> None:
        if not self._telemetry_tasks:
            return

        tasks = list(self._telemetry_tasks.values())
        self._telemetry_tasks = {}
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    async def _connect_pattern(self, endpoints: List[str]) -> bool:
        await self._stop_telemetry()
        await self._stop_mavsdk_backends()
        self._reset_connections()

        for cfg, endpoint in zip(self.drone_cfgs, endpoints):
            ok = await self._connect_exact_endpoint(cfg, endpoint)
            if not ok:
                return False

        self.logger.info("[connect-pattern] connected endpoints=%s", endpoints)
        return True

    async def _telemetry_loop(self, drone_name: str) -> None:
        drone = self.systems[drone_name]
        try:
            async for pv in drone.telemetry.position_velocity_ned():
                state = self.states.get(drone_name)
                if state is None:
                    return

                state.position_ned_m = np.array(
                    [
                        pv.position.north_m,
                        pv.position.east_m,
                        pv.position.down_m,
                    ],
                    dtype=float,
                )
                state.velocity_ned_mps = np.array(
                    [
                        pv.velocity.north_m_s,
                        pv.velocity.east_m_s,
                        pv.velocity.down_m_s,
                    ],
                    dtype=float,
                )
                if not state.telemetry_ready:
                    self.logger.info("[telemetry-ready] %s", drone_name)
                    state.telemetry_ready = True
        except asyncio.CancelledError:
            return
        except Exception:
            self.logger.exception("[error] telemetry loop crashed for %s", drone_name)
            raise

    async def _start_telemetry(self) -> None:
        await self._stop_telemetry()
        for cfg in self.drone_cfgs:
            self._telemetry_tasks[cfg.name] = asyncio.create_task(
                self._telemetry_loop(cfg.name)
            )

        self.logger.info("[wait] Waiting for telemetry readiness...")
        while not all(self.states[cfg.name].telemetry_ready for cfg in self.drone_cfgs):
            await asyncio.sleep(0.1)

    async def _verify_stream_diversity(self) -> bool:
        # Before arming, vehicles can spawn close. We only need to reject clearly mirrored streams.
        sample_count = 10
        min_pair_distance = 0.03

        for _ in range(sample_count):
            positions = [self.states[cfg.name].position_ned_m.copy() for cfg in self.drone_cfgs]

            pair_distances = []
            for i in range(len(positions)):
                for j in range(i + 1, len(positions)):
                    pair_distances.append(float(np.linalg.norm(positions[i] - positions[j])))

            if pair_distances and max(pair_distances) >= min_pair_distance:
                return True
            await asyncio.sleep(0.2)

        return False

    async def _publish_velocity(self, drone_name: str, velocity_ned: np.ndarray) -> None:
        cmd = VelocityNedYaw(
            float(velocity_ned[0]),
            float(velocity_ned[1]),
            float(velocity_ned[2]),
            0.0,
        )
        await self.systems[drone_name].offboard.set_velocity_ned(cmd)

    async def _publish_all_zero(self) -> None:
        await asyncio.gather(
            *(
                self._publish_velocity(cfg.name, np.zeros(3, dtype=float))
                for cfg in self.drone_cfgs
            )
        )

    async def _prime_offboard_setpoints(self) -> None:
        assert self.params is not None
        dt = 1.0 / self.params.control_rate_hz

        self.logger.info("[prime] Sending initial offboard setpoints...")
        for _ in range(max(12, int(self.params.control_rate_hz * 1.2))):
            await self._publish_all_zero()
            await asyncio.sleep(dt)

    async def _prime_single_offboard_setpoints(self, drone_name: str) -> None:
        assert self.params is not None
        dt = 1.0 / self.params.control_rate_hz

        for _ in range(max(6, int(self.params.control_rate_hz * 0.6))):
            await self._publish_velocity(drone_name, np.zeros(3, dtype=float))
            await asyncio.sleep(dt)

    async def _arm_takeoff_and_offboard(self) -> None:
        self.logger.info("[arm] Arming all drones...")
        for cfg in self.drone_cfgs:
            await self.systems[cfg.name].action.arm()

        self.logger.info("[takeoff] Sending PX4 takeoff command...")
        await asyncio.gather(*(self.systems[cfg.name].action.takeoff() for cfg in self.drone_cfgs))

        self.logger.info("[takeoff] Waiting for airborne state...")
        airborne_threshold_down_m = -0.5
        for _ in range(300):
            if all(
                self.states[cfg.name].position_ned_m[2] <= airborne_threshold_down_m
                for cfg in self.drone_cfgs
            ):
                break
            await asyncio.sleep(0.1)

        await self._prime_offboard_setpoints()

        self.logger.info("[offboard] Starting offboard mode...")
        for cfg in self.drone_cfgs:
            # MAVSDK/PX4 can still require a fresh per-vehicle setpoint right before start().
            await self._prime_single_offboard_setpoints(cfg.name)
            try:
                await self.systems[cfg.name].offboard.start()
                self.logger.info("[offboard-start] %s", cfg.name)
            except OffboardError as e:
                self.logger.warning(
                    "[offboard-warn] first start failed for %s: %s; retrying once",
                    cfg.name,
                    e,
                )
                await self._prime_single_offboard_setpoints(cfg.name)
                try:
                    await self.systems[cfg.name].offboard.start()
                    self.logger.info("[offboard-start] %s (retry)", cfg.name)
                except OffboardError as e2:
                    try:
                        await self.systems[cfg.name].action.disarm()
                    except Exception:
                        pass
                    raise RuntimeError(
                        f"Failed to start offboard for {cfg.name}: {e2}"
                    ) from e2

    async def _climb_to_source_altitudes(self) -> None:
        assert self.params is not None
        dt = 1.0 / self.params.control_rate_hz
        climb_rate = min(1.0, max(0.4, self.params.max_speed_mps * 0.33))
        tolerance = 0.35

        self.logger.info("[staging] Climbing to configured source altitudes...")
        for cycle in range(1, 700):
            all_ready = True
            tasks = []

            for cfg in self.drone_cfgs:
                current_down = float(self.states[cfg.name].position_ned_m[2])
                target_down = float(cfg.source_ned_m[2])
                error = target_down - current_down

                if abs(error) > tolerance:
                    all_ready = False
                    cmd_down = float(np.clip(0.55 * error, -climb_rate, climb_rate))
                    cmd = np.array([0.0, 0.0, cmd_down], dtype=float)
                else:
                    cmd = np.zeros(3, dtype=float)

                tasks.append(self._publish_velocity(cfg.name, cmd))

                if cycle % self.log_every_n_cycles == 0:
                    self.logger.info(
                        "[staging] drone=%s current_down=%.2f target_down=%.2f cmd_down=%.2f",
                        cfg.name,
                        current_down,
                        target_down,
                        cmd[2],
                    )

            await asyncio.gather(*tasks)

            if all_ready:
                self.logger.info("[staging] Source altitude hold achieved.")
                for _ in range(max(6, int(self.params.control_rate_hz))):
                    await self._publish_all_zero()
                    await asyncio.sleep(dt)
                return

            await asyncio.sleep(dt)

        raise RuntimeError("Altitude staging timed out before reaching source heights.")

    async def _validate_endpoint_identity(self) -> None:
        assert self.params is not None
        dt = 1.0 / self.params.control_rate_hz

        self.logger.info("[identity-check] Running endpoint-to-vehicle identity probe...")

        pulse_speed = min(0.9, self.params.max_speed_mps * 0.35)
        pulse_duration = 2.0
        settle_duration = 0.8
        min_target_forward_m = 0.08
        separation_margin_m = 0.06
        max_other_forward_m = 0.10

        for target in self.drone_cfgs:
            baseline = {
                cfg.name: self.states[cfg.name].position_ned_m.copy() for cfg in self.drone_cfgs
            }

            pulse_steps = max(1, int(pulse_duration / dt))
            for _ in range(pulse_steps):
                tasks = []
                for cfg in self.drone_cfgs:
                    if cfg.name == target.name:
                        cmd = np.array([pulse_speed, 0.0, 0.0], dtype=float)
                    else:
                        cmd = np.zeros(3, dtype=float)
                    tasks.append(self._publish_velocity(cfg.name, cmd))
                await asyncio.gather(*tasks)
                await asyncio.sleep(dt)

            settle_steps = max(1, int(settle_duration / dt))
            for _ in range(settle_steps):
                await self._publish_all_zero()
                await asyncio.sleep(dt)

            moved = {}
            north_delta = {}
            for cfg in self.drone_cfgs:
                delta = self.states[cfg.name].position_ned_m - baseline[cfg.name]
                moved[cfg.name] = float(np.linalg.norm(delta))
                north_delta[cfg.name] = float(delta[0])

            target_move = moved[target.name]
            target_forward = north_delta[target.name]
            other_moves = [moved[cfg.name] for cfg in self.drone_cfgs if cfg.name != target.name]
            max_other = max(other_moves) if other_moves else 0.0
            other_forward = [north_delta[cfg.name] for cfg in self.drone_cfgs if cfg.name != target.name]
            max_other_forward = max(other_forward) if other_forward else 0.0

            self.logger.info(
                "[identity-check] target=%s moved=%.2fm target_forward=%.2fm max_other=%.2fm max_other_forward=%.2fm",
                target.name,
                target_move,
                target_forward,
                max_other,
                max_other_forward,
            )

            if (
                target_forward < min_target_forward_m
                or target_forward < (max_other_forward + separation_margin_m)
                or max_other_forward > max_other_forward_m
            ):
                raise RuntimeError(
                    "Vehicle identity check failed. One endpoint may be controlling multiple drones "
                    "or telemetry is aliased. Verify PX4 multi-vehicle ports and ensure each drone "
                    "has a unique MAVLink UDP stream."
                )

        self.logger.info("[identity-check] Passed.")

    async def connect_and_prepare(self) -> None:
        connection_ok = False
        failed_patterns: List[List[str]] = []

        for pattern in self._endpoint_patterns():
            self.logger.info("[connect-pattern] trying endpoints=%s", pattern)
            pattern_connected = await self._connect_pattern(pattern)
            if not pattern_connected:
                failed_patterns.append(pattern)
                continue

            await self._start_telemetry()
            diverse = await self._verify_stream_diversity()
            if not diverse:
                self.logger.error(
                    "[connect-pattern] telemetry streams appear aliased for endpoints=%s",
                    pattern,
                )
                await self._stop_telemetry()
                failed_patterns.append(pattern)
                continue

            connection_ok = True
            break

        if not connection_ok:
            raise RuntimeError(
                "Unable to establish 3 unique vehicle MAVLink streams. "
                f"Tried endpoint patterns: {failed_patterns}. "
                "This usually means PX4 multi-vehicle is not actually running 3 instances or "
                "ports are different from expected."
            )

        await self._arm_takeoff_and_offboard()
        await self._climb_to_source_altitudes()
        await self._validate_endpoint_identity()

        self.logger.info("[ready] Leader-follower controller active.")

    def _neighbors(self, me: str) -> List[str]:
        assert self.params is not None
        p_me = self.states[me].position_ned_m

        neighbors: List[str] = []
        for other in self.states:
            if other == me:
                continue
            distance = float(np.linalg.norm(self.states[other].position_ned_m - p_me))
            if distance <= self.params.neighbor_range_m:
                neighbors.append(other)
        return neighbors

    def _flocking_term(self, me: str, neighbors: List[str]) -> np.ndarray:
        assert self.params is not None

        if not neighbors:
            return np.zeros(3, dtype=float)

        p_i = self.states[me].position_ned_m
        v_i = self.states[me].velocity_ned_mps

        sep = np.zeros(3, dtype=float)
        positions = []
        velocities = []

        for n in neighbors:
            p_j = self.states[n].position_ned_m
            v_j = self.states[n].velocity_ned_mps
            delta = p_i - p_j
            dist = float(np.linalg.norm(delta))
            if dist < 1e-6:
                continue

            effective = max(dist, self.params.min_sep_distance_m)
            sep += delta / (effective * effective)
            positions.append(p_j)
            velocities.append(v_j)

        if not positions:
            return np.zeros(3, dtype=float)

        mean_pos = np.mean(np.array(positions), axis=0)
        mean_vel = np.mean(np.array(velocities), axis=0)

        v_sep = self.params.k_sep * sep
        v_coh = self.params.k_coh * (mean_pos - p_i)
        v_ali = self.params.k_ali * (mean_vel - v_i)

        return v_sep + v_coh + v_ali

    def _leader_command(self, cfg: DroneConfig) -> np.ndarray:
        assert self.params is not None
        p = self.states[cfg.name].position_ned_m

        goal_error = cfg.destination_ned_m - p
        return self.params.k_mig * goal_error

    def _follower_command(self, cfg: DroneConfig, neighbors: List[str]) -> np.ndarray:
        assert self.params is not None
        assert self.leader_name is not None

        p_f = self.states[cfg.name].position_ned_m
        v_f = self.states[cfg.name].velocity_ned_mps

        p_l = self.states[self.leader_name].position_ned_m
        v_l = self.states[self.leader_name].velocity_ned_mps

        desired_slot = p_l + cfg.desired_offset_from_leader_m
        v_slot = self.params.k_follow * (desired_slot - p_f)

        v_flock = self._flocking_term(cfg.name, neighbors)
        v_align_leader = self.params.k_ali * (v_l - v_f)
        v_goal = 0.35 * self.params.k_mig * (cfg.destination_ned_m - p_f)

        return v_slot + v_flock + v_align_leader + v_goal

    def _apply_speed_limits(self, cmd_ned: np.ndarray) -> np.ndarray:
        assert self.params is not None

        xy = cmd_ned[:2]
        z = float(cmd_ned[2])

        xy_speed = float(np.linalg.norm(xy))
        max_xy = self.params.max_speed_mps
        max_z = min(1.25, self.params.max_speed_mps * 0.55)

        if xy_speed > 1e-6 and xy_speed > max_xy:
            xy = xy / xy_speed * max_xy

        z = float(np.clip(z, -max_z, max_z))
        return np.array([xy[0], xy[1], z], dtype=float)

    def _smooth_command(self, drone_name: str, raw_cmd: np.ndarray) -> np.ndarray:
        alpha = 0.35
        prev = self.states[drone_name].filtered_cmd_ned_mps
        blended = (1.0 - alpha) * prev + alpha * raw_cmd
        self.states[drone_name].filtered_cmd_ned_mps = blended
        return blended

    def _is_arrived(self, cfg: DroneConfig) -> bool:
        assert self.params is not None
        p = self.states[cfg.name].position_ned_m

        if cfg.role == "leader":
            return (
                float(np.linalg.norm(cfg.destination_ned_m - p)) <= self.params.arrival_radius_m
            )

        assert self.leader_name is not None
        slot_target = self.states[self.leader_name].position_ned_m + cfg.desired_offset_from_leader_m

        goal_ok = float(np.linalg.norm(cfg.destination_ned_m - p)) <= (self.params.arrival_radius_m + 0.6)
        slot_ok = float(np.linalg.norm(slot_target - p)) <= (self.params.arrival_radius_m + 0.8)
        return goal_ok and slot_ok

    async def run(self) -> None:
        assert self.params is not None
        dt = 1.0 / self.params.control_rate_hz
        cycle = 0

        while not self._stop_requested:
            cycle += 1
            publish_tasks = []
            all_arrived = True

            for cfg in self.drone_cfgs:
                neighbors = self._neighbors(cfg.name)

                if cfg.role == "leader":
                    raw_cmd = self._leader_command(cfg)
                else:
                    raw_cmd = self._follower_command(cfg, neighbors)

                raw_cmd = self._apply_speed_limits(raw_cmd)
                cmd = self._smooth_command(cfg.name, raw_cmd)

                if self._is_arrived(cfg):
                    cmd = np.zeros(3, dtype=float)
                else:
                    all_arrived = False

                publish_tasks.append(self._publish_velocity(cfg.name, cmd))

                if cycle % self.log_every_n_cycles == 0:
                    p = self.states[cfg.name].position_ned_m
                    goal_dist = float(np.linalg.norm(cfg.destination_ned_m - p))
                    self.logger.info(
                        "[loop] drone=%s role=%s goal_dist=%.2f neighbors=%d pos=[%.2f, %.2f, %.2f] cmd=[%.2f, %.2f, %.2f]",
                        cfg.name,
                        cfg.role,
                        goal_dist,
                        len(neighbors),
                        p[0],
                        p[1],
                        p[2],
                        cmd[0],
                        cmd[1],
                        cmd[2],
                    )

            await asyncio.gather(*publish_tasks)

            if all_arrived:
                self.logger.info("[complete] All drones reached destination constraints.")
                self._stop_requested = True

            await asyncio.sleep(dt)

    async def shutdown(self) -> None:
        await self._stop_telemetry()
        self.logger.info("[shutdown] Stopping offboard and disarming...")
        for cfg in self.drone_cfgs:
            drone = self.systems.get(cfg.name)
            if drone is None:
                continue

            try:
                await drone.offboard.stop()
            except Exception:
                pass

            try:
                await drone.action.disarm()
            except Exception:
                pass


async def async_main(config_path: Path) -> None:
    controller = LeaderFollowerSwarmController(config_path)
    controller.load_config()

    try:
        await controller.connect_and_prepare()
        await controller.run()
    finally:
        await controller.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="PX4 leader-follower swarm controller with local interaction rules."
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
