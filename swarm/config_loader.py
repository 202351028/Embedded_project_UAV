from pathlib import Path
from typing import List
import os

import numpy as np
import yaml

from .models import DroneConfig, SwarmConfig, SwarmParams, ZoneConfig


def _read_runtime_count(config_path: Path) -> int | None:
    env_count = os.getenv("SWARM_DRONE_COUNT", "").strip()
    if env_count.isdigit() and int(env_count) > 0:
        return int(env_count)

    runtime_state = config_path.parent / ".swarm_runtime.env"
    if not runtime_state.exists():
        return None

    try:
        for line in runtime_state.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            if key.strip() == "SWARM_DRONE_COUNT":
                value = value.strip()
                if value.isdigit() and int(value) > 0:
                    return int(value)
    except Exception:
        return None

    return None


def _fibonacci_sphere_offsets(count: int, radius: float) -> List[np.ndarray]:
    if count <= 0:
        return []

    points: List[np.ndarray] = []
    phi = np.pi * (3.0 - np.sqrt(5.0))

    for i in range(count):
        y = 1.0 - (2.0 * (i + 0.5) / count)
        r_xy = np.sqrt(max(0.0, 1.0 - y * y))
        theta = phi * i
        x = np.cos(theta) * r_xy
        z = np.sin(theta) * r_xy

        # Keep offsets mostly around leader altitude while still being 3D-ish.
        point = np.array([x, z, 0.45 * y], dtype=float)
        norm = float(np.linalg.norm(point))
        if norm > 1e-9:
            point = (point / norm) * radius
        points.append(point)

    return points


def load_swarm_config(config_path: Path) -> SwarmConfig:
    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    connect_timeout_sec = float(cfg.get("mavsdk", {}).get("action_timeout_sec", 12.0))
    connect_call_timeout_sec = float(
        cfg.get("mavsdk", {}).get("connect_call_timeout_sec", 8.0)
    )
    log_every_n_cycles = int(cfg.get("logging", {}).get("log_every_n_cycles", 10))

    swarm_cfg = cfg["swarm"]
    params = SwarmParams(
        control_rate_hz=float(swarm_cfg["control_rate_hz"]),
        neighbor_range_m=float(swarm_cfg["neighbor_range_m"]),
        arrival_radius_m=float(swarm_cfg["arrival_radius_m"]),
        max_speed_mps=float(swarm_cfg["max_speed_mps"]),
        k_sep=float(swarm_cfg["k_sep"]),
        k_coh=float(swarm_cfg["k_coh"]),
        k_ali=float(swarm_cfg["k_ali"]),
        k_mig=float(swarm_cfg["k_mig"]),
        k_follow=float(swarm_cfg["k_follow"]),
        k_neighbor_response=float(swarm_cfg.get("k_neighbor_response", 0.65)),
        k_obstacle=float(swarm_cfg.get("k_obstacle", 1.25)),
        k_threat_repulsion=float(swarm_cfg.get("k_threat_repulsion", 1.7)),
        k_threat_tangential=float(swarm_cfg.get("k_threat_tangential", 1.15)),
        min_sep_distance_m=float(swarm_cfg["min_sep_distance_m"]),
        k_stationary_damping=float(swarm_cfg.get("k_stationary_damping", 0.55)),
    )

    behavior = cfg.get("behavior", {})
    identity_check_enabled = bool(behavior.get("identity_check_enabled", True))
    followers_require_leader_visibility = bool(
        behavior.get("followers_require_leader_visibility", True)
    )

    formation_cfg = cfg.get("formation", {})
    formation_radius_m = float(formation_cfg.get("sphere_radius_m", 3.0))
    use_config_offsets = bool(formation_cfg.get("use_config_offsets", False))

    drone_cfgs: List[DroneConfig] = []
    for d in cfg["drones"]:
        destination_ned = d.get("destination_ned_m", d["source_ned_m"])
        drone_cfgs.append(
            DroneConfig(
                name=str(d["name"]),
                role=str(d["role"]).strip().lower(),
                mavlink_udp=str(d["mavlink_udp"]),
                mavsdk_server_port=int(d.get("mavsdk_server_port", 50051 + len(drone_cfgs))),
                source_ned_m=np.array(d["source_ned_m"], dtype=float),
                destination_ned_m=np.array(destination_ned, dtype=float),
                desired_offset_from_leader_m=np.array(
                    d.get("desired_offset_from_leader_m", [0.0, 0.0, 0.0]), dtype=float
                ),
            )
        )

    requested_count = _read_runtime_count(config_path)
    if requested_count is not None and requested_count > len(drone_cfgs):
        template = next((d for d in drone_cfgs if d.role != "leader"), drone_cfgs[0])
        last_udp_port = int(drone_cfgs[-1].mavlink_udp.split(":")[-1])
        last_grpc = max(d.mavsdk_server_port for d in drone_cfgs)
        base_len = len(drone_cfgs)

        for idx in range(base_len, requested_count):
            offset = idx - base_len + 1
            drone_cfgs.append(
                DroneConfig(
                    name=f"drone_{idx + 1}",
                    role="follower",
                    mavlink_udp=f"udp://:{last_udp_port + offset}",
                    mavsdk_server_port=last_grpc + offset,
                    source_ned_m=template.source_ned_m.copy(),
                    destination_ned_m=template.destination_ned_m.copy(),
                    desired_offset_from_leader_m=np.zeros(3, dtype=float),
                )
            )

    leaders = [d.name for d in drone_cfgs if d.role == "leader"]
    if len(leaders) != 1:
        raise ValueError("Configuration must include exactly one leader drone.")
    leader_name = leaders[0]

    leader_cfg = next(d for d in drone_cfgs if d.name == leader_name)
    for drone_cfg in drone_cfgs:
        if drone_cfg.role != "leader":
            drone_cfg.destination_ned_m = leader_cfg.destination_ned_m.copy()

    if not use_config_offsets:
        followers = [d for d in drone_cfgs if d.role != "leader"]
        offsets = _fibonacci_sphere_offsets(len(followers), formation_radius_m)
        for d, offset in zip(followers, offsets):
            d.desired_offset_from_leader_m = offset

    zone_height_scale = float(cfg.get("environment", {}).get("zone_height_scale", 2.0))
    zones: List[ZoneConfig] = []
    for z in cfg.get("environment", {}).get("zones", []):
        kind = str(z.get("kind", "obstacle")).strip().lower()
        if kind not in {"obstacle", "threat"}:
            raise ValueError(f"Unsupported zone kind '{kind}' in environment.zones")

        base_height = float(z.get("height_m", 3.0 if kind == "obstacle" else 1.0))
        zones.append(
            ZoneConfig(
                name=str(z.get("name", f"{kind}_{len(zones) + 1}")),
                kind=kind,
                center_ned_m=np.array(z["center_ned_m"], dtype=float),
                radius_m=float(z.get("radius_m", 2.0)),
                influence_m=float(z.get("influence_m", 6.0)),
                repulsion_gain=float(z.get("repulsion_gain", 1.0)),
                tangential_gain=float(z.get("tangential_gain", 0.0)),
                speed_scale=float(z.get("speed_scale", 1.0)),
                height_m=base_height * zone_height_scale,
            )
        )

    return SwarmConfig(
        drone_cfgs=drone_cfgs,
        params=params,
        leader_name=leader_name,
        zones=zones,
        connect_timeout_sec=connect_timeout_sec,
        connect_call_timeout_sec=connect_call_timeout_sec,
        log_every_n_cycles=log_every_n_cycles,
        identity_check_enabled=identity_check_enabled,
        followers_require_leader_visibility=followers_require_leader_visibility,
        formation_radius_m=formation_radius_m,
    )
