from dataclasses import dataclass, field
from typing import List

import numpy as np


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
    k_neighbor_response: float
    k_obstacle: float
    k_threat_repulsion: float
    k_threat_tangential: float
    min_sep_distance_m: float
    k_stationary_damping: float


@dataclass
class ZoneConfig:
    name: str
    kind: str
    center_ned_m: np.ndarray
    radius_m: float
    influence_m: float
    repulsion_gain: float
    tangential_gain: float = 0.0
    speed_scale: float = 1.0
    height_m: float = 2.0


@dataclass
class DroneState:
    position_ned_m: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    velocity_ned_mps: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    filtered_cmd_ned_mps: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=float)
    )
    telemetry_ready: bool = False
    vehicle_uuid: int | None = None


@dataclass
class SwarmConfig:
    drone_cfgs: List[DroneConfig]
    params: SwarmParams
    leader_name: str
    zones: List[ZoneConfig]
    connect_timeout_sec: float
    connect_call_timeout_sec: float
    log_every_n_cycles: int
    identity_check_enabled: bool
    followers_require_leader_visibility: bool
    formation_radius_m: float
