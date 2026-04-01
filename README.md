# PX4 + Gazebo Leader-Follower Swarm (WSL-ready)

This package implements a decentralized leader-follower swarm controller based on your paper context:

- Local neighbor checks at each control cycle
- Separation, cohesion, alignment terms
- Leader migration to destination
- Follower formation alignment with desired offset from leader
- Per-drone source and destination in config

## Project Files

- `main.py`: Async PX4 swarm controller (MAVSDK offboard velocity control)
- `config.yaml`: Drone and swarm parameters
- `requirements.txt`: Python dependencies

## 1) What this code expects in PX4 SITL

Run multiple PX4 vehicles in Gazebo so each has a MAVLink UDP endpoint.
This config uses:

- leader: `udp://:14540`
- follower 1: `udp://:14541`
- follower 2: `udp://:14542`

If your PX4 launch uses different ports, update `mavlink_udp` in `config.yaml`.

## 2) Setup in WSL (you run these)

```bash
cd ~/your_workspace/px4_swarm_leader_follower
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## 3) Run in WSL (after PX4+Gazebo multi-UAV is up)

```bash
python3 main.py --config config.yaml
```

## 4) Configuration Notes

In `config.yaml`, each drone has:

- `source_ned_m`: intended start location (for scenario consistency)
- `destination_ned_m`: goal location
- `desired_offset_from_leader_m`: formation slot (followers)

Main swarm tuning parameters:

- `k_sep`, `k_coh`, `k_ali`: flocking behavior
- `k_mig`: migration toward destination
- `k_follow`: follower alignment with leader slot
- `neighbor_range_m`: communication/local sensing radius
- `max_speed_mps`: safety cap

## 5) Important behavior

- Neighbor set is recomputed continuously for each drone.
- Followers continuously align and reposition using both neighbor flocking and leader slot correction.
- Drones stop when they reach their destination radius.

## 6) Safety Reminder

Test first in simulation only. Start with low `max_speed_mps` and conservative gains.
