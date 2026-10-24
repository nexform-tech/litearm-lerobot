#!/usr/bin/env python3
"""Record a state-replay diagnostic episode that is not training data.

This example has no independent action source. It sends the measured joint
state back to the arm and records the accepted target under a diagnostic
feature, never under ``action``. The missing action feature deliberately keeps
the output from masquerading as an imitation-learning dataset.
"""
import argparse
import time
from pathlib import Path

import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset

from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

STATE_REPLAY_FEATURE = "diagnostic.state_replay_target"
_REFUSAL = """refusing to record: this example has no independent action source.

It would otherwise label the measured joint state as the action and produce an
identity dataset that is misleading for policy training. Use a recorder with a
real leader, teleoperator, planner, or policy action source. If you only need a
driver/data-path diagnostic, rerun with --allow-state-replay-demo. That output
has no `action` feature and is not policy-training data.
"""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None,
                        help="CDC port (default: auto-detect)")
    parser.add_argument("--repo-id", default="litearm_state_replay_demo",
                        help="diagnostic dataset repo id")
    parser.add_argument("--root", default="diagnostic_data",
                        help="diagnostic dataset root directory")
    parser.add_argument("--fps", type=int, default=30,
                        help="recording frequency")
    parser.add_argument("--task", default="state_replay",
                        help="diagnostic scenario label")
    parser.add_argument("--episodes", type=int, default=1,
                        help="number of episodes")
    parser.add_argument("--episode-length", type=int, default=50,
                        help="frames per episode")
    parser.add_argument(
        "--allow-state-replay-demo",
        action="store_true",
        help="explicitly create non-training state-replay diagnostic data",
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    if not args.allow_state_replay_demo:
        raise SystemExit(_REFUSAL)

    print(
        "WARNING: recording a state-replay diagnostic with no `action` feature. "
        "Do not use this output for policy training."
    )

    cfg = LiteArmRobotConfig(port=args.port)
    with LiteArmRobot(cfg) as robot:
        # LeRobotDataset.create expects full feature specs (not the shorthand
        # returned by robot.observation_features / action_features).
        features = {
            "observation.state": {"dtype": "float32", "shape": (robot.config.num_joints,)},
            STATE_REPLAY_FEATURE: {"dtype": "float32", "shape": (robot.config.num_joints,)},
        }
        # LeRobotDataset.create stores the dataset directly at <root>/<repo_id>
        # and requires that directory not to exist yet (no silent overwrites).
        dataset_root = Path(args.root) / args.repo_id
        if dataset_root.exists():
            raise SystemExit(
                f"dataset already exists: {dataset_root} — pick a new --repo-id "
                "because this workflow never overwrites an existing recording."
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
            print(f"--- diagnostic episode {episode}/{args.episodes} ---")
            dataset.episode_buffer = dataset.create_episode_buffer()
            for step in range(args.episode_length):
                obs = robot.get_observation()
                replay_target = list(obs["observation.state"])
                accepted = robot.send_action({"action": replay_target})["action"]
                dataset.add_frame({
                    "observation.state": np.asarray(obs["observation.state"], dtype=np.float32),
                    STATE_REPLAY_FEATURE: np.asarray(accepted, dtype=np.float32),
                    "task": f"diagnostic.state_replay:{args.task}",
                })
                time.sleep(1.0 / args.fps)
            dataset.save_episode()
            print(f"--- episode {episode} saved ---")


if __name__ == "__main__":
    main()
