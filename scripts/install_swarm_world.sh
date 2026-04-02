#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_PATH="${SWARM_CONFIG:-$ROOT_DIR/config.yaml}"
WORLD_SRC="$ROOT_DIR/gazebo/worlds/swarm_city_realworld.world"

if [ ! -f "$CONFIG_PATH" ]; then
  echo "Config file not found: $CONFIG_PATH"
  exit 1
fi

python3 "$ROOT_DIR/scripts/sync_world_from_config.py" \
  --config "$CONFIG_PATH" \
  --output "$WORLD_SRC"

PX4_DIR="${PX4_DIR:-}"

if [ -z "$PX4_DIR" ]; then
  for CANDIDATE in \
    "$HOME/PX4-Autopilot" \
    "/mnt/d/PX4-Autopilot" \
    "/mnt/d/IIITV/Semester_6/Embedded/PX4-Autopilot" \
    "$ROOT_DIR/PX4-Autopilot" \
    "$ROOT_DIR/../PX4-Autopilot"
  do
    if [ -d "$CANDIDATE" ]; then
      PX4_DIR="$CANDIDATE"
      break
    fi
  done
fi

if [ -z "$PX4_DIR" ]; then
  echo "PX4-Autopilot folder not found. Set PX4_DIR before running this script."
  exit 1
fi

CLASSIC_WORLD_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"
GZ_WORLD_DIR="$PX4_DIR/Tools/simulation/gz/worlds"

TARGET_DIR=""
if [ -d "$CLASSIC_WORLD_DIR" ]; then
  TARGET_DIR="$CLASSIC_WORLD_DIR"
elif [ -d "$GZ_WORLD_DIR" ]; then
  TARGET_DIR="$GZ_WORLD_DIR"
else
  echo "No known PX4 world directory found under $PX4_DIR"
  exit 1
fi

cp "$WORLD_SRC" "$TARGET_DIR/swarm_city_realworld.world"

echo "Installed world to: $TARGET_DIR/swarm_city_realworld.world"
