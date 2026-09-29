#!/usr/bin/env python3
"""Drive a small sine offset through robot.send_action().

MOTION EXAMPLE — this drives the real arm. Keep --amplitude small, put the
emergency stop within reach, and clear the workspace first.

The trajectory is an offset **from the pose the arm is in when you start**, not
an absolute joint vector: commanding absolute angles would fling the arm across
its whole range on the first step.
"""
import argparse
import math
import time

from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None,
                    help="CDC port (default: auto-detect)")
    ap.add_argument("--duration", type=float, default=10.0, help="seconds")
    ap.add_argument("--amplitude", type=float, default=0.05,
                    help="sine amplitude in rad (keep small on first runs)")
    ap.add_argument("--hz", type=float, default=30.0,
                    help="how often send_action() is called (the servo loop "
                         "itself runs at config.servo_hz)")
    args = ap.parse_args()

    cfg = LiteArmRobotConfig(port=args.port)
    with LiteArmRobot(cfg) as robot:
        q0 = list(robot.get_observation()["observation.state"])   # 起点
        print(f"start pose: {[round(v, 4) for v in q0]}")
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            t = time.monotonic() - start
            # ⚠ 从**实测起点**偏移，不是把全轴设成同一个绝对值
            q = list(q0)
            q[0] += args.amplitude * math.sin(0.5 * t)
            robot.send_action({"action": q})
            time.sleep(1.0 / args.hz)


if __name__ == "__main__":
    main()
