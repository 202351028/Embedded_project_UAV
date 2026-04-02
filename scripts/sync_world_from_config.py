#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from swarm.world_builder import write_world_file


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate Gazebo world from swarm config so source/destination/zones stay in sync."
    )
    parser.add_argument("--config", required=True, help="Path to config.yaml")
    parser.add_argument("--output", required=True, help="Path to output .world file")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(args.config).resolve()
    output_path = Path(args.output).resolve()

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    write_world_file(config_path=config_path, world_output=output_path)
    print(f"Generated world: {output_path}")


if __name__ == "__main__":
    main()
