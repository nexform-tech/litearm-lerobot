# litearm-lerobot

A [LeRobot](https://github.com/huggingface/lerobot) `Robot` adapter for the
**LiteArm** robotic arm. Use the LiteArm with LeRobot's standard `Robot`
interface: read observations, send actions, and record `LeRobotDataset` episodes
for policy training — no changes to LeRobot tooling required.

```text
LeRobot Robot API ──→ LiteArmRobot ──→ litearm.Arm ──→ litearm-server ──→ arm/CAN
```

> 📖 Full developer guide & API reference: [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
> · 中文文档：[README.zh-CN.md](README.zh-CN.md)

---

## Table of contents

- [Overview & architecture](#overview--architecture)
- [Requirements](#requirements)
- [Installation](#installation)
- [Quick start](#quick-start)
- [Configuration](#configuration)
- [Observation / action loop](#observation--action-loop)
- [Record a dataset](#record-a-dataset)
- [LeRobot CLI integration](#lerobot-cli-integration)
- [Examples](#examples)
- [Troubleshooting](#troubleshooting)
- [Package layout](#package-layout)
- [Documentation & license](#documentation--license)

---

## Overview & architecture

`litearm-lerobot` implements the abstract `lerobot.robots.robot.Robot` interface
on top of [litearm-python](https://pypi.org/project/litearm-python)'s remote
`Arm` client. Anything that drives a LeRobot robot — policy loops, dataset
pipelines, evaluation scripts — can drive a LiteArm unchanged.

| Concern | Implementation |
|---|---|
| Robot type name | `"litearm"` (registered in LeRobot's config registry) |
| Observation | `{"observation.state": [q0..q6]}` — 7 absolute joint positions |
| Action | `{"action": [q0..q6]}` — 7 joint position targets |
| Calibration | No-op — the arm reports absolute encoder positions, no homing |
| Motion | `Arm.movej()` via a non-blocking background commander (or blocking) |
| Backend | Remote `litearm.Arm` → **litearm-server** → hardware/CAN |

The LiteArm reports **absolute** joint positions from the arm controller, so
`calibrate()` is a no-op and `is_calibrated` is always `True`.

## Requirements

| Item | Requirement |
|---|---|
| Python | 3.10+ |
| LeRobot | `lerobot>=0.3.0,<0.5` |
| Base SDK | [litearm-python](https://pypi.org/project/litearm-python) `>=0.1.0` |
| Runtime | A reachable **litearm-server** (Zenoh endpoint, e.g. `tcp/192.168.31.237:7447`) |

> LeRobot is usually installed in its own conda/venv. Install this package into
> **that same environment** so both `lerobot` and `litearm` are importable
> together (see [Installation](#installation)).

## Installation

```bash
pip install litearm-lerobot            # release install
pip install -e .                       # development install (from this directory)
```

If you use a dedicated LeRobot environment, install into it instead:

```bash
conda run -n lerobot pip install -e .
```

Run the tests on any machine (no hardware — `litearm.Arm` is faked):

```bash
python -m pytest tests/ -q
```

## Quick start

```python
from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

robot = LiteArmRobot(LiteArmRobotConfig(
    endpoint="tcp/192.168.31.237:7447",   # address of the litearm-server
    arm_id="armA",
))
robot.connect()

obs = robot.get_observation()             # {"observation.state": [q0..q6]}
print(robot.observation_features)         # {"observation.state": (7,)}

robot.send_action({"action": [0.0] * 7})  # command the zero joint pose
robot.disconnect()
```

`LiteArmRobot` is registered with LeRobot's config registry under the name
**`litearm`** (`RobotConfig.register_subclass("litearm")`), so you can also
instantiate it from a YAML config or a `RobotConfig`:

```python
from litearm_lerobot import LiteArmRobot
robot = LiteArmRobot(LiteArmRobotConfig.from_yaml("config.yaml"))
```

## Configuration

`LiteArmRobotConfig(RobotConfig)` — all fields:

| Field | Default | Description |
|---|---|---|
| `endpoint` | `tcp/127.0.0.1:7447` | Zenoh endpoint of the litearm-server |
| `arm_id` | `armA` | Arm id registered on the server |
| `query_timeout` | `None` | Per-call RPC timeout in seconds (`None` = SDK default) |
| `num_joints` | `7` | Number of joints (drives observation/action feature shapes) |
| `control_frequency_hz` | `30.0` | Commander loop rate used by `send_action` |
| `movej_speed` | `0.5` | `movej` speed (0..1) applied to every commanded target |
| `settle_s` | `0.2` | Seconds to wait for the arm to settle after each `movej` |
| `use_commander` | `True` | Non-blocking background commander (see below) |
| `enable_on_connect` | `True` | Call `enable()` (motors on, hold pose) on `connect()` |

## Observation / action loop

Both vectors are the **seven absolute joint positions** reported by the arm
controller. The observation/action features are:

```text
observation_features = {"observation.state": (7,)}
action_features      = {"action": (7,)}
```

```python
while not done:
    obs = robot.get_observation()          # {"observation.state": [q0..q6]}
    action = policy(obs["observation.state"])
    robot.send_action({"action": action})
```

`send_action` validates the input (the `"action"` key must exist and hold exactly
`num_joints` values) and then behaves in one of two modes:

- **`use_commander=True` (default) — non-blocking.** A background thread
  continuously drives the arm toward the latest target with `movej` at
  `control_frequency_hz`. `send_action` only updates the target and returns
  immediately, so a policy/teleop loop can run at its own rate. Identical
  targets are skipped.
- **`use_commander=False` — blocking.** `send_action` calls `Arm.movej()`
  directly and blocks until the motion finishes.

> `Arm.movej` is a **blocking** RPC — it returns only once the arm has settled.
> That is exactly why the commander thread exists; without it, a LeRobot loop
> would stall on every step.

## Record a dataset

`LeRobotDataset` requires **full feature specs**, not the shorthand returned by
`observation_features`:

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
    root=Path("data") / "litearm_demo",          # this directory must NOT exist
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

Key points:

- `LeRobotDataset.create(...)` stores the dataset directly at `<root>/<repo_id>`
  and **refuses to overwrite** an existing directory — pick a fresh `repo_id` or
  delete the directory to re-record.
- Every frame must contain a `"task"` key and numpy **float32** arrays for each
  feature.
- The full loop is `create_episode_buffer()` → `add_frame(...)` × N →
  `save_episode()`.

The ready-made recording script is
[examples/03_record_dataset.py](examples/03_record_dataset.py).

## LeRobot CLI integration

LeRobot builds robots through `lerobot.robots.utils.make_robot_from_config`, an
internal if/elif chain over the built-in robot names — `litearm` is not in it by
default. `litearm_lerobot.utils.register()` monkey-patches that function to add
the `litearm` branch:

```python
from litearm_lerobot.utils import register
register()            # idempotent; call once at program startup
```

After that, any config whose `type` is `litearm` resolves to
`LiteArmRobot`. Programmatic use (the Quick Start path) needs no patching.

## Examples

| Example | Description |
|---|---|
| [examples/01_read_observation.py](examples/01_read_observation.py) | Read-only: connect and print observations |
| [examples/02_send_action.py](examples/02_send_action.py) | Drive a scripted sinusoidal trajectory (**motion** — keep the E-stop nearby) |
| [examples/03_record_dataset.py](examples/03_record_dataset.py) | Record teleop episodes into a `LeRobotDataset` |

Runnable commands:

```bash
python examples/01_read_observation.py --endpoint tcp/192.168.31.237:7447 --count 5
python examples/02_send_action.py --endpoint tcp/192.168.31.237:7447 \
  --duration 10 --amplitude 0.05 --speed 0.3
python examples/03_record_dataset.py --endpoint tcp/192.168.31.237:7447 \
  --repo-id litearm_demo --root data --episodes 1 --episode-length 50 --task push
```

> ⚠️ Examples 02 and 03 **drive the real arm**. Start with small
> `--amplitude`/`--speed` values and keep your hand near the emergency stop.
> Each example's options are described in [examples/README.md](examples/README.md).

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `ModuleNotFoundError: No module named 'lerobot'` | This package was installed into a different environment than LeRobot. `conda run -n lerobot pip install -e .`. |
| `ModuleNotFoundError: No module named 'litearm'` | `litearm-python` is missing from the python running the node. `pip install litearm-python`. |
| `RuntimeError: LiteArmRobot is not connected` | Call `robot.connect()` before `get_observation()`/`send_action()`. |
| `RuntimeError: No robot state received yet` | The server has not broadcast state yet. Make sure litearm-server is up and `endpoint`/`arm_id` are correct; retry after a moment. |
| `ValueError: action must have 7 joints, got N` | The action vector must contain exactly `num_joints` values. |
| `ValueError: action dict must contain an 'action' key` | `send_action` expects `{"action": [..]}`. |
| `enable()` fails on connect | The server may be read-only; the connection continues (logged as a warning). |
| Dataset already exists at `<root>/<repo_id>` | `LeRobotDataset.create` refuses to overwrite. Pick a new `--repo-id` or delete the directory. |

## Package layout

```text
litearm-lerobot/
├── pyproject.toml             package metadata + pytest config
├── src/litearm_lerobot/
│   ├── __init__.py            public API (LiteArmRobot, LiteArmRobotConfig, register)
│   ├── config.py              LiteArmRobotConfig (LeRobot RobotConfig subclass)
│   ├── robot.py               LiteArmRobot + background commander thread
│   └── utils.py               register() — LeRobot factory monkey-patch
├── examples/
│   ├── 01_read_observation.py
│   ├── 02_send_action.py
│   ├── 03_record_dataset.py
│   └── README.md / README.zh-CN.md
├── tests/                     mock unit tests (no hardware)
└── docs/
    ├── DEVELOPER_GUIDE.md
    └── DEVELOPER_GUIDE.zh-CN.md
```

## Documentation & license

- Developer guide & API reference (interface details, dataset recording, CLI
  integration internals): [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
- 中文使用说明：[README.zh-CN.md](README.zh-CN.md)
- License: see `pyproject.toml` (`LicenseRef-Proprietary`).
