#!/usr/bin/env python3
"""Record a LiteArm teleop episode into a LeRobotDataset (no cameras).

While the arm is in zero-gravity (or being moved by a script), each step
records the current joint positions as both observation and action.

Output layout: <root>/<repo_id> — directly usable with the LeRobot train
scripts (policy training) once a camera feature is added if needed.
"""
import argparse
import time
from pathlib import Path

import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset

from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

DEFAULT_ENDPOINT = "tcp/192.168.31.237:7447"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--endpoint", default=DEFAULT_ENDPOINT,
                    help=f"litearm-server zenoh endpoint (default {DEFAULT_ENDPOINT})")
    ap.add_argument("--arm-id", default="armA", help="Arm id (default armA)")
    ap.add_argument("--repo-id", default="litearm_demo", help="dataset repo id")
    ap.add_argument("--root", default="data", help="dataset root directory")
    ap.add_argument("--fps", type=int, default=30, help="recording frequency")
    ap.add_argument("--task", default="push", help="task name for the episode")
    ap.add_argument("--episodes", type=int, default=1, help="number of episodes")
    ap.add_argument("--episode-length", type=int, default=50,
                    help="frames per episode")
    args = ap.parse_args()

    robot = LiteArmRobot(LiteArmRobotConfig(
        endpoint=args.endpoint,
        arm_id=args.arm_id,
        use_commander=True,
    ))
    robot.connect()
    try:
        # LeRobotDataset.create expects full feature specs (not the shorthand
        # returned by robot.observation_features / action_features).
        features = {
            "observation.state": {"dtype": "float32", "shape": (robot.config.num_joints,)},
            "action": {"dtype": "float32", "shape": (robot.config.num_joints,)},
        }
        # LeRobotDataset.create stores the dataset directly at <root>/<repo_id>
        # and requires that directory not to exist yet (no silent overwrites).
        dataset_root = Path(args.root) / args.repo_id
        if dataset_root.exists():
            raise SystemExit(
                f"dataset already exists: {dataset_root} — pick a new --repo-id "
                f"or delete the directory to re-record."
            )
        dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            fps=args.fps,
            root=dataset_root,
            robot_type=robot.name,
            features=features,
            use_videos=False,
        )
        print(f"dataset: {dataset}")

        for episode in range(args.episodes):
            print(f"--- episode {episode}/{args.episodes}: move the arm ... ---")
            dataset.episode_buffer = dataset.create_episode_buffer()
            for step in range(args.episode_length):
                obs = robot.get_observation()
                action = list(obs["observation.state"])
                robot.send_action({"action": action})      # hold / follow the demo
                dataset.add_frame({
                    "observation.state": np.asarray(obs["observation.state"], dtype=np.float32),
                    "action": np.asarray(action, dtype=np.float32),
                    "task": args.task,
                })
                time.sleep(1.0 / args.fps)
            dataset.save_episode()
            print(f"--- episode {episode} saved ---")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
