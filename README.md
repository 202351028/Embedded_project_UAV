# PX4 + Gazebo Leader-Follower Swarm (WSL-ready)

This package implements a decentralized leader-follower swarm controller based on your paper context:

- Local neighbor checks at each control cycle
- Separation, cohesion, alignment terms for followers
- Leader migration to destination only
- Follower formation alignment with desired offset from the leader
- Per-drone source and destination in config

## Project Files

- `main.py`: Async PX4 swarm controller (MAVSDK offboard velocity control)
- `config.yaml`: Drone and swarm parameters
- `requirements.txt`: Python dependencies

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

## 3) Run in WSL (after PX4+Gazebo multi-UAV is up)

```bash
python3 main.py --config config.yaml
```

Run this controller while PX4 and Gazebo Classic are already up. In PX4 Gazebo Classic `sitl_multiple_run.sh`, vehicle instance IDs start at 1, so the common endpoints are 14561, 14562, and 14563. Older setups can use 14540-series ports. The controller auto-tries one-based and zero-based variants of both families with timeout logs.

The controller now writes detailed diagnostics to `swarm_debug.log` in this project folder.

## 4) Configuration Notes

In `config.yaml`, each drone has:

- `source_ned_m`: intended start location (for scenario consistency)
- `destination_ned_m`: goal location
- `desired_offset_from_leader_m`: formation slot (followers)

Main swarm tuning parameters:

- `k_sep`, `k_coh`, `k_ali`: follower flocking behavior
- `k_mig`: leader migration toward destination
- `k_follow`: follower alignment with the leader slot
- `neighbor_range_m`: communication/local sensing radius
- `max_speed_mps`: safety cap
- `mavsdk.action_timeout_sec`: timeout waiting for connected state
- `mavsdk.connect_call_timeout_sec`: timeout for each `system.connect()` call
- `logging.log_every_n_cycles`: control-loop heartbeat interval

## 5) Important behavior

- Neighbor set is recomputed continuously for each drone.
- The controller first commands each drone to climb to its configured source altitude, then transitions into swarm motion.
- Followers continuously align and reposition using local flocking plus leader slot correction.
- Drones stop when they reach their destination radius.

## 6) Safety Reminder

Test first in simulation only. Start with low `max_speed_mps` and conservative gains.
