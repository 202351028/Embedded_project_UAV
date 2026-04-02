import argparse
import asyncio
from pathlib import Path

from swarm.controller import async_main


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="PX4 leader-follower swarm controller with local interaction rules."
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config.yaml",
        help="Path to YAML configuration file.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(args.config).resolve()
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    try:
        asyncio.run(async_main(config_path))
    except KeyboardInterrupt:
        print("\n[exit] Interrupted by user.")


if __name__ == "__main__":
    main()
