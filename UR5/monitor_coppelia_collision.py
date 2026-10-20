#!/usr/bin/env python3
"""Monitor CoppeliaSim collision pairs during trajectory playback.

This is an independent diagnostic tool. It does not modify the ROS2
controller or the CoppeliaSim scene. The current scene contains the real
Workpiece shape and GunCollisionHull. The MATLAB-only wire collision model
cannot be checked from CoppeliaSim until a corresponding scene object exists.
"""

from __future__ import annotations

import argparse
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def collision_flag(sim, first, second):
    """Return the boolean collision flag from sim.checkCollision."""
    result = sim.checkCollision(first, second)
    if isinstance(result, (tuple, list)):
        return bool(result[0])
    return bool(result)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Monitor UR5/tool versus Workpiece in CoppeliaSim"
    )
    parser.add_argument(
        "--period",
        type=float,
        default=0.05,
        help="Polling period in seconds (default: 0.05)",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Check once and exit",
    )
    parser.add_argument(
        "--stop-on-collision",
        action="store_true",
        help="Stop CoppeliaSim when a collision is detected",
    )
    args = parser.parse_args()

    if args.period <= 0:
        parser.error("--period must be positive")

    client = RemoteAPIClient("127.0.0.1", 23000)
    sim = client.require("sim")

    ur5 = sim.getObject("/UR5")
    workpiece = sim.getObject("/Workpiece")
    gun_hull = sim.getObject("/UR5/connection/WeldingGunMount/GunCollisionHull")

    print("CoppeliaSim collision monitor connected.")
    print("  UR5 ↔ Workpiece")
    print("  GunCollisionHull ↔ Workpiece")
    print("  wireCollision: MATLAB-only; not present in CoppeliaSim")
    print("  stop-on-collision:", bool(args.stop_on_collision))

    previous = None
    while True:
        ur5_hit = collision_flag(sim, ur5, workpiece)
        gun_hit = collision_flag(sim, gun_hull, workpiece)
        current = (ur5_hit, gun_hit)

        if current != previous:
            print(
                "collision state: "
                f"UR5↔Workpiece={int(ur5_hit)}, "
                f"GunCollisionHull↔Workpiece={int(gun_hit)}"
            )
            previous = current

        if (ur5_hit or gun_hit) and args.stop_on_collision:
            print("COLLISION DETECTED: stopping CoppeliaSim.")
            sim.stopSimulation()
            return 2

        if args.once:
            return int(ur5_hit or gun_hit)

        time.sleep(args.period)


if __name__ == "__main__":
    raise SystemExit(main())
