#!/usr/bin/env python3
"""Read LiteArm observations through the LeRobot Robot interface.

Connects, prints the observation features and a few state vectors.

Two modes:

* **default** — enables the motors and starts the servo loop, which holds the
  measured pose. This *does* command the arm: leave it free to move.
* **--no-enable** — a genuine observation-only session. The motors stay
  disabled, the servo loop is never started, and nothing is ever commanded
  (send_action raises). Safe to run with the arm powered but idle, e.g. on
  first bring-up; observations still work, because the arm streams status
  frames whether or not it is enabled.
"""
import argparse

from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None,
                    help="CDC port, e.g. /dev/ttyACM0 "
                         "(default: auto-detect VID:PID 1d50:606f)")
    ap.add_argument("--count", type=int, default=5, help="number of samples")
    ap.add_argument("--no-enable", action="store_true",
                    help="observation-only session: leave the motors disabled "
                         "(no servo loop, nothing is ever commanded). "
                         "Without this flag the motors are enabled and the "
                         "servo loop holds the measured pose.")
    args = ap.parse_args()

    cfg = LiteArmRobotConfig(port=args.port,
                             enable_on_connect=not args.no_enable)
    with LiteArmRobot(cfg) as robot:
        print("observation_features:", robot.observation_features)
        for i in range(args.count):
            obs = robot.get_observation()
            print(f"[{i}] observation.state = "
                  f"{[round(v, 4) for v in obs['observation.state']]}")


if __name__ == "__main__":
    main()
