# litearm-lerobot

[LeRobot](https://github.com/huggingface/lerobot) `Robot` adapter for the LiteArm
robotic arm. Use the LiteArm with LeRobot's standard `Robot` interface: read
observations, send actions, and record `LeRobotDataset` episodes for policy
training.

## Features

- 🦾 **Full LeRobot `Robot` interface** — observation/action features, connect,
  calibrate, get_observation, send_action, disconnect
- 🎯 **7-DOF joint control** — `observation.state` (7,) and `action` (7,)
- 🎥 **Dataset recording** — drop-in `LeRobotDataset` episodes for LeRobot training
- 🔄 **Non-blocking `send_action`** — a background commander stream drives the arm
  at your configured control frequency
- 🧩 **No numpy / no hardware on the client** — built on [litearm-python](../litearm-python)

> 📖 Full developer guide & API reference: [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
> · 中文文档：[README.zh-CN.md](README.zh-CN.md)

## Requirements

| Item | Requirement |
|---|---|
| Python | 3.10+ |
| LeRobot | `lerobot>=0.3.0,<0.5` |
| Base SDK | [litearm-python](https://pypi.org/project/litearm-python) `>=0.1.0` |

## Installation

```bash
pip install litearm-lerobot            # release install
pip install -e .                       # development install (from this directory)
```

> If the `lerobot` conda/venv environment is used, install into it instead,
> e.g. `conda run -n lerobot pip install -e .`.

## Quick Start

```python
from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

robot = LiteArmRobot(LiteArmRobotConfig(
    endpoint="tcp/192.168.31.237:7447",   # address of the litearm-server
    arm_id="armA",
))
robot.connect()

obs = robot.get_observation()             # {"observation.state": [q0..q6]}
print(robot.observation_features)         # {"observation.state": (7,)}

robot.send_action({"action": [0.0] * 7})  # move to home position
robot.disconnect()
```

`LiteArmRobot` is registered with LeRobot's config registry under the name
**`litearm`** (`RobotConfig.register_subclass("litearm")`), so you can also
instantiate it from a YAML config or a `RobotConfig`.

## Record a dataset

```python
from pathlib import Path
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

robot = LiteArmRobot(LiteArmRobotConfig(endpoint="tcp/192.168.31.237:7447"))
robot.connect()

features = {
    "observation.state": {"dtype": "float32", "shape": (robot.config.num_joints,)},
    "action": {"dtype": "float32", "shape": (robot.config.num_joints,)},
}
dataset = LeRobotDataset.create(
    repo_id="litearm_demo", fps=30,
    root=Path("data") / "litearm_demo",          # must NOT already exist
    robot_type=robot.name, features=features, use_videos=False,
)
dataset.episode_buffer = dataset.create_episode_buffer()
obs = robot.get_observation()
dataset.add_frame({
    "observation.state": np.asarray(obs["observation.state"], dtype=np.float32),
    "action": np.asarray(obs["observation.state"], dtype=np.float32),
    "task": "push",
})
dataset.save_episode()
robot.disconnect()
```

## Examples

| Example | Description |
|---|---|
| [examples/01_read_observation.py](examples/01_read_observation.py) | Read-only: connect and print observations |
| [examples/02_send_action.py](examples/02_send_action.py) | Drive a scripted sinusoidal trajectory (motion — keep the E-stop nearby) |
| [examples/03_record_dataset.py](examples/03_record_dataset.py) | Record teleop episodes into a `LeRobotDataset` |

See [examples/README.md](examples/README.md) for runnable commands.
