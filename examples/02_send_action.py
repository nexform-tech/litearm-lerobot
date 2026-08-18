#!/usr/bin/env python3
"""Drive a scripted sinusoidal trajectory through robot.send_action().

⚠️ MOTION EXAMPLE — drives the real arm! Start with a low speed and keep
your hand near the emergency stop.
"""
import argparse
import math
import time

from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

DEFAULT_ENDPOINT = "tcp/192.168.31.237:7447"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--endpoint", default=DEFAULT_ENDPOINT,
                    help=f"litearm-server zenoh endpoint (default {DEFAULT_ENDPOINT})")
    ap.add_argument("--arm-id", default="armA", help="Arm id (default armA)")
    ap.add_argument("--duration", type=float, default=10.0, help="seconds")
    ap.add_argument("--amplitude", type=float, default=0.05,
                    help="sine amplitude in rad (keep small on first runs)")
    ap.add_argument("--speed", type=float, default=0.3,
                    help="movej speed 0..1 (keep low on first runs)")
    ap.add_argument("--hz", type=float, default=30.0, help="control frequency")
    args = ap.parse_args()

    robot = LiteArmRobot(LiteArmRobotConfig(
        endpoint=args.endpoint,
        arm_id=args.arm_id,
        control_frequency_hz=args.hz,
        movej_speed=args.speed,
        use_commander=True,
    ))
    robot.connect()
    try:
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            t = time.monotonic() - start
            q = [args.amplitude * math.sin(0.5 * t)] * 7
            robot.send_action({"action": q})
            time.sleep(1.0 / args.hz)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
