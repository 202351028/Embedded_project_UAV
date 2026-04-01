#!/usr/bin/env bash
set -e

PX4_DIR=""

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

if [ -f "./Tools/simulation/gz/sitl_multiple_run.sh" ]; then
	./Tools/simulation/gz/sitl_multiple_run.sh -n 3 -m x500
else
	echo "sitl_multiple_run.sh not found for this PX4 version."
	echo "Available files under Tools/simulation/gz/:"
	ls ./Tools/simulation/gz/
fi
