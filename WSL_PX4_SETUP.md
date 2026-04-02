# PX4 + Gazebo Classic Setup in WSL2

This guide shows a clean setup for PX4 with Gazebo Classic on WSL2, then runs your swarm controller against the simulator.

Use Gazebo Classic if the newer Gazebo integration is freezing or lagging in WSL.

## 0) Recommended base

- Windows 11 preferred
- WSL2 required
- Ubuntu 22.04 LTS recommended

If Ubuntu is not installed yet, open PowerShell as Administrator and run:

```powershell
wsl --install -d Ubuntu-22.04
```

Reboot if Windows asks for it.

## 1) Update Ubuntu and install prerequisites

Run these in your Ubuntu WSL terminal:

```bash
sudo apt update
sudo apt upgrade -y

sudo apt install -y \
    git \
    curl \
    wget \
    zip \
    unzip \
    gnupg \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    ninja-build \
    ccache \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python3-empy \
    python3-toml \
    python3-jinja2 \
    python3-numpy
```

If `python3 -m venv` later fails with `ensurepip is not available`, install the versioned venv package too:

```bash
sudo apt install -y python3.10-venv
```

## 2) Clone PX4-Autopilot

Use the Linux filesystem in WSL for PX4. It is faster and more stable than building from `/mnt/d`.

```bash
cd ~
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd ~/PX4-Autopilot
```

If you already cloned it, refresh submodules:

```bash
cd ~/PX4-Autopilot
git submodule update --init --recursive
```

## 3) Switch to a PX4 version that works well with Gazebo Classic

For Gazebo Classic, PX4 v1.14 is the safest choice.

```bash
cd ~/PX4-Autopilot
git fetch --all --tags
git checkout v1.14.0
git submodule update --init --recursive
```

## 4) Run PX4 dependency setup

```bash
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx
```

When that finishes, close the WSL terminal and open a new one, or reboot WSL if the script asks for it.

## 5) Build and run a single Gazebo Classic vehicle first

Start with one vehicle to confirm the simulator works before trying multiple drones.

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

What you should see:

- Gazebo Classic launches
- PX4 console starts
- A single vehicle appears in the scene

If this does not launch, run this to check available targets and scripts:

```bash
find Tools -type f -name "*gazebo*" | head -n 50
```

## 6) Install custom swarm world (optional but recommended)

```bash
cd /mnt/d/IIITV/Semester_6/Embedded/px4_swarm_leader_follower
chmod +x ./scripts/install_swarm_world.sh
./scripts/install_swarm_world.sh
```

This installs `swarm_city_realworld.world` into PX4's world directory.

## 7) Launch 3 vehicles for your swarm

On PX4 v1.14, try the multi-vehicle Gazebo Classic launcher:

```bash
cd /mnt/d/IIITV/Semester_6/Embedded/px4_swarm_leader_follower
chmod +x ./start_px4_swarm.sh
SWARM_WORLD=swarm_city_realworld ./start_px4_swarm.sh
```

If your PX4 branch does not support world selection via `-w`, the script auto-falls back to default world launch.

The launcher reads `config.yaml` to determine drone count and generates a close grid spawn layout automatically.

If that script is not present, search for the exact helper in your tree:

```bash
find ~/PX4-Autopilot/Tools -type f -name "*multiple*run*.sh" 2>/dev/null
```

If you find a different script name, run it with the same `-n 3 -m iris` style arguments.

## 8) Verify MAVLink UDP ports

Open another WSL terminal and check UDP listeners after you start `main.py` (MAVSDK creates local listeners):

```bash
ss -lun | grep -E "14561|14562|14563|14540|14541|14542|14543"
```

Your swarm controller is configured to use:

- leader: `udp://:14561`
- follower 1: `udp://:14562`
- follower 2: `udp://:14563`

Note: some PX4 versions use 14540-series ports. The controller now auto-tries both 14560-series and 14540-series endpoints with timeout logs.

If the ports differ, update `mavlink_udp` in `config.yaml`.

## 9) Install and run your swarm controller

In the project folder:

```bash
cd /mnt/d/IIITV/Semester_6/Embedded/px4_swarm_leader_follower
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
python3 main.py --config config.yaml
```

Expected connection logs:

- `[connect] drone_1 -> udp://:14561`
- `[connected] drone_1`
- `[connect] drone_2 -> udp://:14562`
- `[connected] drone_2`
- `[connect] drone_3 -> udp://:14563`
- `[connected] drone_3`

Detailed runtime diagnostics are written to `swarm_debug.log` in `px4_swarm_leader_follower`.

## 10) Suggested terminal layout

- Terminal 1: PX4 multi-vehicle Gazebo Classic launcher
- Terminal 2: swarm controller (`python3 main.py --config config.yaml`)
- Terminal 3: diagnostics (`ss`, `find`, log checks)

## 11) Common problems and fixes

### A) `cd: ~/PX4-Autopilot: No such file or directory`

Find the real PX4 path:

```bash
find /mnt/d -maxdepth 6 -type d -name PX4-Autopilot 2>/dev/null
```

Then use the found path in your commands or launcher script.

### B) `python3 -m venv` fails

Install the missing venv packages and recreate the environment:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip python3.10-venv
rm -rf .venv
python3 -m venv .venv
```

### C) Controller stops at `[connect] drone_1`

That means the first endpoint is reachable but follow-up endpoints are not matching your PX4 port mapping, or PX4 was not fully up.

Check:

```bash
ss -lun | grep -E "14561|14562|14563|14540|14541|14542|14543"
```

If only one port appears, keep PX4 running and check controller logs for endpoint fallback warnings.
Also inspect `swarm_debug.log` for `[warn] connect() timeout` or `[error]` lines to identify the exact failing endpoint.

### D) Gazebo freezes or feels too heavy

Use Gazebo Classic, keep the PX4 tree on the WSL Linux filesystem, and avoid building from `/mnt/d`.

### E) Multi-vehicle script not found

Search the PX4 tree for the exact helper:

```bash
find ~/PX4-Autopilot/Tools -type f -name "*multiple*run*.sh" 2>/dev/null
```

If needed, send that output and the launch command can be adjusted to your exact PX4 layout.

## 12) Optional helper script

If you want a one-command launcher, keep this script in your swarm project and run it from WSL:

```bash
cd /mnt/d/IIITV/Semester_6/Embedded/px4_swarm_leader_follower
chmod +x ./start_px4_swarm.sh
./start_px4_swarm.sh
```

If the script cannot find PX4, edit its `PX4_DIR` path candidates to match your actual PX4 location.
