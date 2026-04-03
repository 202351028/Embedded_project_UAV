#!/usr/bin/env bash
set -euo pipefail

PX4_DIR=""
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_NAME="${SWARM_WORLD:-swarm_city_realworld}"
CONFIG_PATH="${SWARM_CONFIG:-$ROOT_DIR/config.yaml}"

for CANDIDATE in \
	"$HOME/PX4-Autopilot" \
	"/mnt/d/PX4-Autopilot" \
	"/mnt/d/IIITV/Semester_6/Embedded/PX4-Autopilot" \
	"$(pwd)/PX4-Autopilot" \
	"$(pwd)/../PX4-Autopilot"
do
	if [ -d "$CANDIDATE" ]; then
		PX4_DIR="$CANDIDATE"
		break
	fi
done

if [ -z "$PX4_DIR" ]; then
	echo "PX4-Autopilot folder not found."
	echo "Find it with: find /mnt/d -maxdepth 4 -type d -name PX4-Autopilot 2>/dev/null"
	echo "Then edit this script and set PX4_DIR to that path."
	exit 1
fi

cd "$PX4_DIR"
echo "Using PX4 path: $PX4_DIR"
echo "Requested world: $WORLD_NAME"

if [ ! -f "$CONFIG_PATH" ]; then
	echo "Config file not found: $CONFIG_PATH"
	exit 1
fi

if [ -f "$ROOT_DIR/scripts/sync_world_from_config.py" ]; then
	python3 "$ROOT_DIR/scripts/sync_world_from_config.py" \
		--config "$CONFIG_PATH" \
		--output "$ROOT_DIR/gazebo/worlds/${WORLD_NAME}.world"
fi

if [ -f "$ROOT_DIR/scripts/install_swarm_world.sh" ]; then
	SWARM_CONFIG="$CONFIG_PATH" SWARM_WORLD="$WORLD_NAME" PX4_DIR="$PX4_DIR" bash "$ROOT_DIR/scripts/install_swarm_world.sh"
fi

DRONE_COUNT="${SWARM_DRONE_COUNT:-}"
if [ -z "$DRONE_COUNT" ]; then
	DRONE_COUNT=$(python3 - <<PY
import yaml
from pathlib import Path
p = Path(r"$CONFIG_PATH")
cfg = yaml.safe_load(p.read_text(encoding="utf-8"))
print(len(cfg.get("drones", [])))
PY
)
fi

if ! [[ "$DRONE_COUNT" =~ ^[0-9]+$ ]] || [ "$DRONE_COUNT" -lt 1 ]; then
	echo "Invalid drone count: $DRONE_COUNT"
	exit 1
fi

GRID_SPACING="${SWARM_GRID_SPACING:-2.2}"
MODEL_NAME="${SWARM_MODEL:-iris}"
SPAWN_SCRIPT=$(python3 - <<PY
import math
import yaml
from pathlib import Path

cfg = yaml.safe_load(Path(r"$CONFIG_PATH").read_text(encoding="utf-8"))
drones = cfg.get("drones", [])
if not drones:
    print("")
    raise SystemExit(0)

leader = next((d for d in drones if str(d.get("role","")).lower() == "leader"), drones[0])
sx, sy = leader.get("source_ned_m", [0.0, 0.0, -6.0])[:2]
spacing = float(r"$GRID_SPACING")
n = int(r"$DRONE_COUNT")
cols = max(1, int(math.ceil(math.sqrt(n))))
rows = int(math.ceil(n / cols))

parts = []
for i in range(n):
    row = i // cols
    col = i % cols
    x = float(sx) + (col - (cols - 1) / 2.0) * spacing
    y = float(sy) + (row - (rows - 1) / 2.0) * spacing
    parts.append(f"$MODEL_NAME:1:{x:.2f}:{y:.2f}")

print(",".join(parts))
PY
)

echo "Drone count: $DRONE_COUNT"
echo "Grid spacing: $GRID_SPACING"

RUNTIME_STATE_FILE="$ROOT_DIR/.swarm_runtime.env"
cat > "$RUNTIME_STATE_FILE" <<EOF
SWARM_DRONE_COUNT=$DRONE_COUNT
SWARM_WORLD=$WORLD_NAME
SWARM_MODEL=$MODEL_NAME
SWARM_GRID_SPACING=$GRID_SPACING
EOF
echo "Wrote runtime state: $RUNTIME_STATE_FILE"

# WSL commonly has no ALSA playback device. Disable audio to avoid OpenAL startup errors.
if [ "${SWARM_DISABLE_AUDIO:-1}" = "1" ]; then
	export ALSOFT_DRIVERS="null"
	export SDL_AUDIODRIVER="dummy"
	echo "Audio disabled (SWARM_DISABLE_AUDIO=1)."
fi

run_classic() {
	if [ -n "$SPAWN_SCRIPT" ]; then
		if [ -n "$WORLD_NAME" ]; then
			set +e
			./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s "$SPAWN_SCRIPT" -w "$WORLD_NAME"
			status=$?
			set -e
			if [ "$status" -eq 0 ]; then
				return 0
			fi

			echo "Scripted spawn + world failed for this PX4 version. Retrying scripted spawn default world..."
		fi

		set +e
		./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s "$SPAWN_SCRIPT"
		status=$?
		set -e
		if [ "$status" -eq 0 ]; then
			return 0
		fi

		echo "Scripted spawn unsupported. Falling back to -n based launch..."
	fi

	if [ -n "$WORLD_NAME" ]; then
		set +e
		./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n "$DRONE_COUNT" -m "$MODEL_NAME" -w "$WORLD_NAME"
		status=$?
		set -e
		if [ "$status" -eq 0 ]; then
			return 0
		fi

		echo "World flag unsupported or world missing for this PX4 version. Retrying default world..."
	fi

	./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n "$DRONE_COUNT" -m "$MODEL_NAME"
}

run_gz() {
	if [ -n "$WORLD_NAME" ]; then
		set +e
		./Tools/simulation/gz/sitl_multiple_run.sh -n "$DRONE_COUNT" -m "$MODEL_NAME" -w "$WORLD_NAME"
		status=$?
		set -e
		if [ "$status" -eq 0 ]; then
			return 0
		fi

		echo "World flag unsupported or world missing for this PX4 version. Retrying default world..."
	fi

	./Tools/simulation/gz/sitl_multiple_run.sh -n "$DRONE_COUNT" -m "$MODEL_NAME"
}

if [ -f "./Tools/simulation/gazebo-classic/sitl_multiple_run.sh" ]; then
	run_classic
elif [ -f "./Tools/simulation/gz/sitl_multiple_run.sh" ]; then
	run_gz
else
	echo "sitl_multiple_run.sh not found for this PX4 version."
	echo "Available files under Tools/simulation/gazebo-classic/ and Tools/simulation/gz/:"
	ls ./Tools/simulation/gazebo-classic/ 2>/dev/null || true
	ls ./Tools/simulation/gz/ 2>/dev/null || true
fi
