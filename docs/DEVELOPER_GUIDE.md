# litearm-lerobot Developer Guide & API Reference

`litearm-lerobot` exposes the LiteArm robotic arm through the standard LeRobot
`Robot` interface, so LeRobot tools (policies, dataset pipelines, evaluation
scripts) work with the arm unchanged.

```text
LeRobot Robot API ──→ LiteArmRobot ──→ litearm.Arm ──→ litearm-server ──→ arm/CAN
```

---

## 1. Requirements & Installation

| Item | Requirement |
|---|---|
| Python | 3.10+ |
| LeRobot | `lerobot>=0.3.0,<0.5` |
| Base SDK | `litearm-python>=0.1.0` |

```bash
pip install litearm-lerobot          # release install
pip install -e .                     # development install
```

Run the tests (no hardware needed — `litearm.Arm` is faked):

```bash
python -m pytest tests/ -q
```

## 2. Configuration

`LiteArmRobotConfig(RobotConfig)` is registered in LeRobot's config registry
under the name `litearm`. All fields:

| Field | Default | Description |
|---|---|---|
| `endpoint` | `tcp/127.0.0.1:7447` | `litearm-server` Zenoh endpoint |
| `arm_id` | `armA` | Arm id registered on the server |
| `query_timeout` | `None` | Per-call timeout in seconds (`None` = SDK default) |
| `num_joints` | `7` | Number of joints (drives feature shapes) |
| `control_frequency_hz` | `30.0` | Commander loop rate used by `send_action` |
| `movej_speed` | `0.5` | `movej` speed (0..1) used by the commander |
| `settle_s` | `0.2` | Wait after motion before returning |
| `use_commander` | `True` | Use the non-blocking background commander |
| `enable_on_connect` | `True` | Call `enable()` on connect |
| `disable_on_disconnect` | `False` | Call `disable()` instead of `hold()` on disconnect |

## 3. Robot interface

Implementing the abstract members of `lerobot.robots.robot.Robot`:

| Member | litearm-lerobot |
|---|---|
| `name` | `"litearm"` |
| `observation_features` | `{"observation.state": (7,)}` |
| `action_features` | `{"action": (7,)}` |
| `is_connected` | Whether the underlying `litearm.Arm` is open |
| `connect()` | Open the arm, warm the state, `enable()` if configured, start commander |
| `is_calibrated` | Always `True` (the arm uses absolute encoders) |
| `calibrate()` | No-op (nothing to calibrate) |
| `configure()` | No-op |
| `get_observation()` | `{"observation.state": [q0..q6]}` |
| `send_action(action)` | Non-blocking commander target, or blocking `movej` |
| `disconnect()` | Stop commander, `hold()` + `close()` |

### send_action — blocking vs commander

`litearm.Arm.movej` is a blocking RPC (it returns only when the motion completes),
so a raw call would stall the policy loop. Two modes:

- `use_commander=True` (default): a background thread always drives the latest
  target with `movej` at `control_frequency_hz`. `send_action` only updates the
  target and returns immediately. Repeated identical targets are skipped.
- `use_commander=False`: `send_action` calls `movej` directly and blocks until
  the motion finishes.

## 4. Dataset recording

`LeRobotDataset` requires **full feature specs** (not the shorthand returned by
`robot.observation_features`):

```python
features = {
    "observation.state": {"dtype": "float32", "shape": (7,)},
    "action":            {"dtype": "float32", "shape": (7,)},
}
```

`LeRobotDataset.create(repo_id, fps, root, ...)` stores the dataset directly at
`<root>` and **refuses to overwrite** an existing directory — pick a
fresh `repo_id` or delete the directory to re-record. Every frame must contain a
`"task"` key and numpy float32 arrays:

```python
dataset.add_frame({
    "observation.state": np.asarray(obs["observation.state"], dtype=np.float32),
    "action": np.asarray(action, dtype=np.float32),
    "task": "push",
})
```

The full loop is `create_episode_buffer()` → `add_frame(...)` × N →
`save_episode()`. See [examples/03_record_dataset.py](../examples/03_record_dataset.py).

## 5. LeRobot CLI integration (optional)

LeRobot's `make_robot_from_config` is an if/elif chain over the built-in robot
names, so `litearm` is not reachable through it without a small monkey-patch.
`litearm_lerobot.utils.register()` patches it to add the `litearm` branch:

```python
from litearm_lerobot.utils import register
register()            # idempotent; call once at program startup
```

Programmatic use (the main path, shown in the Quick Start) needs no patching.

## 6. Tests

`tests/` fakes `litearm.Arm` (via a `FakeArm` in `conftest.py`), so the full test
suite runs on any machine with `lerobot` + `litearm-python` installed. Coverage:
config type, feature shapes, connect/observe, commander vs blocking `send_action`,
dimension validation, disconnect lifecycle, and `register()` idempotency.
