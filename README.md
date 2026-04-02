# PX4 + Gazebo Leader-Follower Swarm (WSL-ready)

This package implements a decentralized leader-follower swarm controller based on your paper context:

- Local neighbor checks at each control cycle
- Separation, cohesion, alignment terms for followers
- Leader migration to destination only
- Followers react to neighbors and local slot constraints only (no follower destination term)
- Environment zones (obstacles + threat areas) with avoidance behavior

## Project Files

- `main.py`: Thin entrypoint
- `swarm/controller.py`: Swarm runtime and control loops
- `swarm/config_loader.py`: Central config parsing and formation slot generation
- `swarm/world_builder.py`: Config-driven Gazebo world generator
- `swarm/models.py`: Shared dataclasses
- `config.yaml`: Drone and swarm parameters
- `requirements.txt`: Python dependencies
- `gazebo/worlds/swarm_city_realworld.world`: custom Gazebo world with source/destination and hazard zones
- `scripts/sync_world_from_config.py`: regenerates world from config.yaml
- `scripts/install_swarm_world.sh`: regenerates and copies world into PX4 tree

## 1) What this code expects in PX4 SITL

Run multiple PX4 vehicles in Gazebo so each has a MAVLink UDP endpoint.
This config uses:

 leader: `udp://:14561`
 follower 1: `udp://:14562`
 follower 2: `udp://:14563`

If your PX4 launch uses different ports, update `mavlink_udp` in `config.yaml`.

## 2) Setup in WSL (you run these)

```bash
cd /mnt/d/IIITV/Semester_6/Embedded/px4_swarm_leader_follower
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## 3) Install the custom world in PX4 (one-time)

```bash
cd /mnt/d/IIITV/Semester_6/Embedded/px4_swarm_leader_follower
chmod +x ./scripts/install_swarm_world.sh
./scripts/install_swarm_world.sh
```

## 4) Launch PX4 + Gazebo multi-UAV

```bash
cd /mnt/d/IIITV/Semester_6/Embedded/px4_swarm_leader_follower
chmod +x ./start_px4_swarm.sh
SWARM_WORLD=swarm_city_realworld ./start_px4_swarm.sh
```

If your PX4 version does not accept `-w`, the script auto-falls back to the default world.

`start_px4_swarm.sh` now auto-reads drone count from `config.yaml` and spawns vehicles in a centered close grid (not a line).

## 5) Run in WSL (after PX4+Gazebo multi-UAV is up)

```bash
python3 main.py --config config.yaml
```

Run this controller while PX4 and Gazebo Classic are already up. In PX4 Gazebo Classic `sitl_multiple_run.sh`, vehicle instance IDs start at 1, so the common endpoints are 14561, 14562, and 14563. Older setups can use 14540-series ports. The controller auto-tries one-based and zero-based variants of both families with timeout logs.

The controller now writes detailed diagnostics to `swarm_debug.log` in this project folder.

## 6) Configuration Notes

In `config.yaml`, each drone has:

- `source_ned_m`: intended start location (for scenario consistency)
- `destination_ned_m`: goal location (leader only)
- `desired_offset_from_leader_m`: formation slot (followers)

Main swarm tuning parameters:

- `k_sep`, `k_coh`, `k_ali`: follower flocking behavior
- `k_mig`: leader migration toward destination
- `k_follow`: follower alignment with the leader slot
- `k_neighbor_response`: follower response from neighbor centroid/velocity
- `k_stationary_damping`: follower damping when leader is not visible
- `k_obstacle`: obstacle avoidance gain
- `k_threat_repulsion`: threat region repulsive gain
- `k_threat_tangential`: threat region tangential bypass gain
- `neighbor_range_m`: communication/local sensing radius
- `max_speed_mps`: safety cap
- `mavsdk.action_timeout_sec`: timeout waiting for connected state
- `mavsdk.connect_call_timeout_sec`: timeout for each `system.connect()` call
- `logging.log_every_n_cycles`: control-loop heartbeat interval
- `environment.zones`: obstacle/threat definitions
- `environment.zone_height_scale`: global height multiplier for world zone columns
- `formation.sphere_radius_m`: follower spherical formation radius around leader
- `behavior.followers_require_leader_visibility`: freeze-to-local flock behavior when leader is out of range

## 7) Important behavior

- Neighbor set is recomputed continuously for each drone.
- The controller first commands each drone to climb to its configured source altitude, then transitions into swarm motion.
- Followers continuously align and reposition using local flocking plus neighbor response and leader slot correction when leader is in local range.
- Followers do not use any private destination target.
- Obstacle and threat zones inject repulsive/tangential corrections into commanded velocity.
- Drones stop when they reach their destination radius.

Visual cues in the custom world:

- Green ground disc: source area
- Blue ground disc: destination area
- Red solid volumes: obstacle areas
- Orange translucent discs: threat areas
- Yellow ring at source: leader start cue (leader visual marker)

## 8) Safety Reminder

Test first in simulation only. Start with low `max_speed_mps` and conservative gains.
