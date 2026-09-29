# litearm-lerobot

`litearm-lerobot` is a [LeRobot](https://github.com/huggingface/lerobot) `Robot`
adapter for the **LiteArm** robotic arm, for engineers who record datasets or run
policies on the arm.

Use the LiteArm with LeRobot's standard `Robot` interface: read observations, send
actions, and record `LeRobotDataset` episodes for policy training — with no
changes to LeRobot tooling.

```text
LeRobot Robot API ──→ LiteArmRobot ──→ litearm.Arm ──→ USB CDC ──→ STM32 ──→ motors
```

> Full developer guide & API reference: [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
> · 中文文档：[README.zh-CN.md](README.zh-CN.md)

---

## Table of contents

- [Overview & architecture](#overview--architecture)
- [Requirements](#requirements)
- [Installation](#installation)
- [Quick start](#quick-start)
- [Configuration](#configuration)
- [Motion and safety](#motion-and-safety)
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
on top of [litearm-python](https://pypi.org/project/litearm-python)'s **direct USB
CDC** client (`litearm.Arm`). There is no `litearm-server` and no zenoh on this
path. Anything that drives a LeRobot robot — policy loops, dataset pipelines,
evaluation scripts — can drive a LiteArm unchanged.

| Concern | Implementation |
|---|---|
| Robot type name | `"litearm"` (registered in LeRobot's config registry) |
| Observation | `{"observation.state": [q0..q6]}` — 7 absolute joint positions |
| Action | `{"action": [q0..q6]}` — 7 joint position targets |
| Calibration | No-op — the arm reports absolute encoder positions, no homing |
| Motion | `arm.joint_follow (0x08)` via a background servo loop |
| Backend | `litearm.Arm` → USB CDC (`1d50:606f`) → STM32 → motors |

The LiteArm reports **absolute** joint positions from the arm controller, so
`calibrate()` is a no-op and `is_calibrated` is always `True`.

## Requirements

| Item | Requirement |
|---|---|
| Python | 3.10+ |
| LeRobot | `lerobot>=0.3.0,<0.5` |
| Base SDK | `litearm-python>=2.1.0` (not on PyPI — install from its git repository) |
| Firmware | `Litearm1.5.0` or newer, with `CMD_JOINT_FOLLOW` (`0x08`) |
| Hardware | The arm connected over USB CDC (`1d50:606f`) to this machine |

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

# port=None finds the CDC port by VID:PID 1d50:606f.
with LiteArmRobot(LiteArmRobotConfig(port=None)) as robot:
    obs = robot.get_observation()             # {"observation.state": [q0..q6]}
    print(robot.observation_features)         # {"observation.state": (7,)}

    q = list(obs["observation.state"])        # read the pose the arm is in
    q[0] += 0.05                              # then offset from it
    robot.send_action({"action": q})          # non-blocking
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
| `port` | `None` | CDC port. `None` = find it by VID:PID `1d50:606f`. Do not hard-code `/dev/ttyACM0`. |
| `move_timeout` | `15.0` | Timeout for one blocking `movej`, in seconds. `movej` returns early once it settles; this is only a ceiling. |
| `num_joints` | `7` | Joint count. Checked against `arm.n` on `connect()`; a mismatch raises. The 1J bench board reports `1`. |
| `servo_hz` | `250.0` | Servo loop rate in Hz. It feeds `slew_target`'s `dt`, so it should match the real loop period. Setting it **higher** than the loop can actually run does not over-slew: the loop re-anchors when it falls behind, so the reference advances less per real second and the arm moves slower than asked. The error direction is safe. |
| `actuator` | `"joint_follow"` | Which firmware channel to stream: `"joint_follow"` (`0x08`, default) or `"move_js"` (`0x03`). |
| `k_p` | `None` | Position gains, one per joint. `None` = the built-in table. Ignored when `actuator="move_js"`. |
| `k_d` | `None` | Damping gains, one per joint. `None` = the built-in table. Ignored when `actuator="move_js"`. |
| `speed_limit` | `None` | Speed ceilings (rad/s), one per joint. `None` = the conservative general table. |
| `accel_limit` | `None` | Acceleration ceilings (rad/s²), one per joint. `None` = the conservative general table. |
| `engage_sec` | `0.3` | Seconds the loop spends holding the measured pose at low gain before it starts following. This runs in the host loop, not in the firmware. |
| `limit_margin` | `0.01` | Inset applied to the soft limits, in rad. Do not raise it — see [Motion and safety](#motion-and-safety). |
| `enable_on_connect` | `True` | Call `enable()` on `connect()`. `True` (the default) starts the servo loop; a failure raises. `False` gives an **observation-only** session: the motors stay unpowered, `get_observation()` still works off the firmware's passive stream, and `send_action()` raises — the firmware rejects a servo frame on a disabled arm. |
| `disable_on_disconnect` | `False` | Call `disable()` instead of handing the arm back with a zero-motion `movej`. The arm goes limp. |

## Motion and safety

`send_action` is non-blocking. It clamps the target into the soft limits read
from the firmware and hands it to a background servo loop, which streams
`arm.joint_follow` (`0x08`) at `servo_hz`. The servo loop, the soft-limit wall
and the gravity feedforward all run in the firmware.

**Do not** assume the arm holds when your process dies. Kill the process and the
servo stream stops; the firmware's 0.1 s command watchdog then fail-softs the arm
— it goes limp and **slowly sags** under its own weight. This is inherent to the
continuous-servo channel, not a bug. Keep the workspace clear and run long jobs
with someone nearby.

**Do not** call `disable()` on exit. Disabling drops the arm under gravity and it
can drift out of the soft limits into a latched fault that only a person can
clear by pushing the joint back. `disconnect()` hands the arm back with a
zero-motion `movej`, which keeps it rigid. Set `disable_on_disconnect=True` only
when the arm is already resting on a support.

**Do not** raise `limit_margin`. J4's upper soft limit is only `+0.017547 rad`
(1°). A margin of `0.02` puts J4's upper bound at `-0.0025`, silently removing its
entire positive range — and nothing raises an error.

**Do not** hard-code `/dev/ttyACM0`. The two CDC ports on this machine swap
numbers between boots; leave `port=None` and let the SDK match `VID:PID 1d50:606f`.

**Do** run the session inside `with`, so `disconnect()` always runs.

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
`num_joints` values) and then does three things:

- Clamps the target into the soft limits read from the firmware. A clamped axis
  is logged with its name and the original value.
- Records the result as the servo loop's target. This is the only thing
  `send_action` does to the arm, and it returns immediately.
- Returns the clamped vector, so the caller can record what was actually
  commanded.

The background loop wakes at `servo_hz`, advances a rate-limited reference with
`slew_target`, and streams it through a single dispatch point to
`arm.joint_follow`. Because the loop is the only path to the motors, a dead loop
is loud: `get_observation()` and `send_action()` raise with the loop's original
error rather than return stale data.

That loop is why a LeRobot policy loop never stalls. No call blocks on motion.

## Record a dataset

`LeRobotDataset` requires **full feature specs**, not the shorthand returned by
`observation_features`:

```python
from pathlib import Path
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

robot = LiteArmRobot(LiteArmRobotConfig())
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

- `LeRobotDataset.create(...)` stores the dataset directly at `<root>`
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

Runnable commands (pass `--port` only when auto-detection picks the wrong device):

```bash
python examples/01_read_observation.py --count 5
python examples/02_send_action.py --duration 10 --amplitude 0.05
python examples/03_record_dataset.py \
  --repo-id litearm_demo --root data --episodes 1 --episode-length 50 --task push
```

Examples 02 and 03 **drive the real arm**. Start with small `--amplitude` values,
clear the workspace, and keep your hand near the emergency stop. Example 02
offsets from the pose the arm is in when it starts; **do not** change it to send
absolute joint angles, which would command the arm across its full range on the
first step. Each example's options are described in
[examples/README.md](examples/README.md).

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `ModuleNotFoundError: No module named 'lerobot'` | This package was installed into a different environment than LeRobot. `conda run -n lerobot pip install -e .`. |
| `ModuleNotFoundError: No module named 'litearm'` | `litearm-python` is missing from the python running the script. Install it from its git repository. |
| `litearm.TransportError: 未找到 STM32 CDC (VID:PID 1d50:606f), 请用 --port 指定` | No CDC device matches. Connect the arm, or pass `port=` explicitly. |
| `litearm.TransportError: 打开串口 /dev/ttyACMn 失败: ... Could not exclusively lock port` | Another process holds the port. Stop it. Do not run two processes against one CDC port: on Linux they split the same byte stream and both break. |
| `RuntimeError: LiteArmRobot is not connected` | Call `robot.connect()` before `get_observation()`/`send_action()`. |
| `RuntimeError: No robot state received yet` | The link is up but no state frame has arrived yet. Retry after a moment. |
| `RuntimeError: 伺服环已停（joint_follow 下发失败）：...` | The servo loop died and the arm is no longer under control. The message carries the original error. Reconnect and check the link. |
| `ValueError: action must have 7 joints, got N` | The action vector must contain exactly `num_joints` values. |
| `ValueError: action dict must contain an 'action' key` | `send_action` expects `{"action": [..]}`. |
| `ValueError: 关节数不符：固件报 1，配置是 7` | The firmware reports a different joint count than `num_joints`. The 1J bench board reports `1`; a 7J arm reports `7`. |
| `enable()` fails on connect | A direct link cannot be read-only, so this means the arm is not answering. Check the power and the CDC cable. |
| Dataset already exists at `<root>` | `LeRobotDataset.create` refuses to overwrite. Pick a new `--repo-id` or delete the directory. |

## Package layout

```text
litearm-lerobot/
├── pyproject.toml             package metadata + pytest config
├── src/litearm_lerobot/
│   ├── __init__.py            public API (LiteArmRobot, LiteArmRobotConfig, register)
│   ├── config.py              LiteArmRobotConfig (LeRobot RobotConfig subclass)
│   ├── robot.py               LiteArmRobot — the LeRobot Robot implementation
│   ├── servo.py               ServoLoop (background thread), the single _send()
│   ├── safety.py              soft-limit read + clamp + rate-limited reference
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
- 中文开发指南：[docs/DEVELOPER_GUIDE.zh-CN.md](docs/DEVELOPER_GUIDE.zh-CN.md)
- License: see `pyproject.toml` (`LicenseRef-Proprietary`).
