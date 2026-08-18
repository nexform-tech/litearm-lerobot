#!/usr/bin/env python3
"""Read LiteArm observations through the LeRobot Robot interface.

Read-only: connects, prints the observation features and a few state vectors.
"""
import argparse

from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

DEFAULT_ENDPOINT = "tcp/192.168.31.237:7447"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--endpoint", default=DEFAULT_ENDPOINT,
                    help=f"litearm-server zenoh endpoint (default {DEFAULT_ENDPOINT})")
    ap.add_argument("--arm-id", default="armA", help="Arm id (default armA)")
    ap.add_argument("--count", type=int, default=5, help="number of samples")
    ap.add_argument("--no-enable", action="store_true",
                    help="do not enable motors on connect")
    args = ap.parse_args()

    robot = LiteArmRobot(LiteArmRobotConfig(
        endpoint=args.endpoint,
        arm_id=args.arm_id,
        enable_on_connect=not args.no_enable,
    ))
    robot.connect()
    try:
        print("observation_features:", robot.observation_features)
        for i in range(args.count):
            obs = robot.get_observation()
            print(f"[{i}] observation.state = {[round(v, 4) for v in obs['observation.state']]}")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
