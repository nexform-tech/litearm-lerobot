# litearm-lerobot developer guide

How to work on litearm-lerobot: the LeRobot Robot contract, the config surface,
the servo loop, and the failure modes you must not assume away. For developers
extending or debugging this driver.

```text
LeRobot Robot API ──→ LiteArmRobot ──→ litearm.Arm ──→ USB CDC ──→ STM32 ──→ motors
```

There is no `litearm-server` and no zenoh on this path. `litearm.Arm` opens the
arm's USB CDC interface (`1d50:606f`) directly.

---

## 1. Requirements and installation

| Item | Requirement |
|---|---|
| Python | 3.10+ |
| LeRobot | `lerobot>=0.3.0,<0.5` |
| Base SDK | `litearm-python>=2.1.0` (not on PyPI — install from its git repository) |
| Firmware | `Litearm1.5.0` or newer, with `CMD_JOINT_FOLLOW` (`0x08`) |
| Hardware | The arm connected over USB CDC (`1d50:606f`) to this machine |

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
| `port` | `None` | CDC port. `None` = find it by VID:PID `1d50:606f`. Do not hard-code `/dev/ttyACM0`; the port numbers swap between boots. |
| `move_timeout` | `15.0` | Timeout for one blocking `movej`, in seconds. `movej` returns early once it settles, so this is only a ceiling. |
| `num_joints` | `7` | Joint count, checked against `arm.n` on `connect()`. A mismatch raises before anything moves. The 1J bench board reports `1`. |
| `servo_hz` | `250.0` | Servo loop rate in Hz. It feeds `slew_target`'s `dt`, so it should match the real loop period. Setting it **higher** than the loop can actually run does not over-slew: the loop re-anchors when it falls behind, so the reference advances less per real second and the arm moves slower than asked. The error direction is safe. |
| `actuator` | `"joint_follow"` | Firmware channel: `"joint_follow"` (`0x08`) or `"move_js"` (`0x03`). See [send_action and the servo loop](#send_action-and-the-servo-loop). |
| `k_p` | `None` | Position gains, one per joint. `None` = the built-in table. Ignored, with a warning at `connect()`, when `actuator="move_js"`. |
| `k_d` | `None` | Damping gains, one per joint. `None` = the built-in table. Ignored the same way. |
| `speed_limit` | `None` | Speed ceilings (rad/s), one per joint. `None` = the conservative general table. |
| `accel_limit` | `None` | Acceleration ceilings (rad/s²), one per joint. `None` = the conservative general table. |
| `engage_sec` | `0.3` | Seconds the loop spends holding the measured pose at low gain before it starts following. This runs in the host loop, not in the firmware. |
| `limit_margin` | `0.01` | Inset applied to the soft limits, in rad. Do not raise it — see [Do not](#do-not). |
| `enable_on_connect` | `True` | Call `enable()` on `connect()`. `True` (the default) starts the servo loop; a failure raises and closes the port. `False` is an observation-only session: no servo loop, `get_observation()` still works, `send_action()` raises. |
| `disable_on_disconnect` | `False` | Call `disable()` instead of handing the arm back with a zero-motion `movej`. The arm goes limp. |

`validate()` rejects an unknown `actuator`, a non-positive `num_joints` or
`servo_hz`, a negative `limit_margin`, and any per-joint list whose length is not
`num_joints`. It runs at the top of `connect()`.

## 3. Robot interface

Implementing the abstract members of `lerobot.robots.robot.Robot`:

| Member | litearm-lerobot |
|---|---|
| `name` | `"litearm"` |
| `observation_features` | `{"observation.state": (7,)}` |
| `action_features` | `{"action": (7,)}` |
| `is_connected` | Whether the underlying `litearm.Arm` is open |
| `connect()` | Open the CDC link, check `arm.n`, read the soft limits, `enable()` if configured, start the servo loop |
| `is_calibrated` | Always `True` (the arm uses absolute encoders) |
| `calibrate()` | No-op (nothing to calibrate) |
| `configure()` | No-op (gains and limits live in the firmware) |
| `get_observation()` | `{"observation.state": [q0..q6]}` |
| `send_action(action)` | Clamp, set the servo loop's target, return the clamped vector |
| `disconnect()` | Stop the servo loop, hand the arm back, close the link |

### send_action and the servo loop

`send_action` is **non-blocking**. It clamps the target into the soft limits read
from the firmware, stores the result as the servo loop's target, and returns
immediately. Nothing in the call waits for motion.

The background `ServoLoop` thread wakes at `servo_hz` and, at each tick:

1. Advances a rate-limited reference toward that target with `slew_target`, which
   applies the speed and acceleration ceilings and brakes on the `v²/(2a)`
   stopping distance.
2. Computes the per-joint velocity reference.
3. Calls the single dispatch point `_send()`, which forwards to
   `arm.joint_follow(q, dq, k_p, k_d)`.

`_send()` is the only place in the package that talks to the arm's motion
channel. Changing actuators changes that one function plus one config value; no
other code knows which channel is in use.

The firmware owns the rest: the reference tracker, the soft-limit wall force, and
the gravity feedforward (`G(q_meas)`). The host never sends a torque.

The loop is the only path to the motors, so a dead loop must be loud. Any
exception inside it triggers a controlled `hold_at_current()` hand-off, is stored
on `ServoLoop.error`, and is re-raised by the next `get_observation()` or
`send_action()` call with the original exception. `ServoLoop.error` is the named
carrier of that invariant — do not "fix" a dead loop by returning stale state.

### Choosing an actuator

Both channels are implemented; `joint_follow` is the default. The trade is real
and offline-undecidable, which is why `actuator` is a first-class config field.

`joint_follow` (`0x08`):

- Gains are sent with every frame (clamped by the firmware to `kp ≤ 500`,
  `kd ≤ 5.0`), so what you configure is what the motors get.
- The reference slew uses the firmware's `s_jf_vel_max` table, but this
  repository's conservative `speed_limit` default is the binding constraint.
- The firmware exempts this channel from the position and over-speed latches,
  so a bad measured value gets no second opinion. The soft-limit wall and the
  entry clamp still bound the target, and a hardware fault (temperature, motor
  error, stale feedback) still trips the whole-arm emergency stop.
- The exemption ends the moment `hold` is set, so it does not survive a
  watchdog trip.

`move_js` (`0x03`):

- Gains come from the firmware's factory table, not from your config: `mit_kp`
  is 400 on J1–J4, twice the tuned 200, so the spring is stiffer. Your `k_p` and
  `k_d` are ignored and `connect()` logs a warning saying so.
- The firmware adds a torque-domain `kd_extra` (6.0 on J1–J4) that `0x08` cannot
  reach, so its effective damping on those axes is higher than the table reads.
- `dq` decides the reference slew *rate*, not just the feed-forward. A `dq` of
  zero freezes the reference: the arm does not move and nothing raises an error.
  `slew_target` never produces all-zero `dq` while moving, and a test asserts it.
- The position and over-speed latches are active here.

Which channel is likelier to false-trip depends on the trajectory and on the
load, so run a real A/B (same trajectory, both actuators, compare `joint_fault`,
the flag bits, and the tracking-error RMS) before changing the default.

### Do not

The five failure modes below are the ones a reader is most likely to assume
away. They are stated in full, with the reasons, in
[Motion and safety](../README.md#motion-and-safety). In short:

- **Do not** assume the arm holds when your process dies — it fail-softs after
  0.1 s and sags under its own weight.
- **Do not** call `disable()` on exit — the arm falls and can latch a fault that
  only a person can clear.
- **Do not** raise `limit_margin` — `0.02` silently erases J4's whole positive
  range.
- **Do not** hard-code `/dev/ttyACM0` — the port numbers swap between boots.
- **Do** run the session inside `with`, so `disconnect()` always runs.

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

`tests/` fakes the litearm SDK (via a `FakeArm` in `conftest.py`), so the full
test suite runs on any machine with `lerobot` + `litearm-python` installed.
Coverage: config type and validation, feature shapes, connect/observe, the servo
loop (convergence, non-zero `dq`, tick cost, the exception path), dimension
validation, disconnect lifecycle, and `register()` idempotency.

`tests/test_sdk_contract.py` is different: **every expectation in it comes from
the real litearm SDK**, never from this repository's own implementation. It
constructs a real `litearm.Arm` and inspects real signatures, so it is the guard
rail against the criteria and the implementation sharing one source — a suite
that is green only because it agrees with itself. When you change how this
repository calls the SDK, update that file in the same commit.
