import argparse
import asyncio
from typing import List

from mavsdk import System


async def wait_connected(system: System, timeout_s: float):
    async def _inner():
        async for state in system.core.connection_state():
            if state.is_connected:
                return

    await asyncio.wait_for(_inner(), timeout=timeout_s)


async def sample_position(system: System, timeout_s: float):
    async def _inner():
        async for pv in system.telemetry.position_velocity_ned():
            return (
                float(pv.position.north_m),
                float(pv.position.east_m),
                float(pv.position.down_m),
            )

    return await asyncio.wait_for(_inner(), timeout=timeout_s)


async def run(endpoints: List[str], timeout_s: float, grpc_base_port: int):
    systems = []
    for i, ep in enumerate(endpoints):
        grpc_port = grpc_base_port + i
        s = System(port=grpc_port)
        print(f"[connect] {ep} (grpc_port={grpc_port})")
        await asyncio.wait_for(s.connect(system_address=ep), timeout=timeout_s)
        await wait_connected(s, timeout_s)
        systems.append(s)

    samples = []
    for i in range(8):
        row = []
        for s in systems:
            row.append(await sample_position(s, timeout_s))
        samples.append(row)
        await asyncio.sleep(0.25)

    print("\nPosition samples (north, east, down):")
    for i, row in enumerate(samples):
        print(f"sample {i+1}: {row}")

    print("\nPairwise max distances across samples:")
    n = len(endpoints)
    for i in range(n):
        for j in range(i + 1, n):
            dmax = 0.0
            for row in samples:
                pi = row[i]
                pj = row[j]
                d = ((pi[0] - pj[0]) ** 2 + (pi[1] - pj[1]) ** 2 + (pi[2] - pj[2]) ** 2) ** 0.5
                dmax = max(dmax, d)
            print(f"{endpoints[i]} <-> {endpoints[j]} : {dmax:.3f} m")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Quick MAVLink stream uniqueness checker")
    parser.add_argument("--endpoints", nargs="+", required=True)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--grpc-base-port", type=int, default=50151)
    args = parser.parse_args()

    asyncio.run(run(args.endpoints, args.timeout, args.grpc_base_port))
