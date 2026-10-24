Known gaps in the dataset recording path and the tooling around it, written for maintainers planning the next round of work.

# Improvement backlog: dataset recording

This list is based on the code at commit `dd2a594` on `main`. Items marked
"verified in code" were read directly; items marked "needs hardware" require a
bench check before they can be closed.

## How to read this list

Priorities describe what blocks real use, not how hard a fix is.

| Priority | Meaning |
|---|---|
| P0 | Blocks using the recorded data for policy training, or makes the docs wrong |
| P1 | Correctness, reproducibility, or long-session risk |
| P2 | Ergonomics and operator experience |

Each item has evidence, impact, a proposal, and a completion check. Do not treat
a proposal as approved design; it is a starting point for review.

## Current P0 safeguard

The bundled example now fails closed unless the operator passes
`--allow-state-replay-demo`. The opted-in diagnostic has no `action` feature;
it stores the accepted hold target as `diagnostic.state_replay_target`. This
prevents the old `(q_t, q_t)` stream from masquerading as policy-training data.
R1 remains open for the future training recorder, which still needs an
independent leader, teleoperator, planner, or policy action source.

## 1. Recording semantics

### R1. Recorded actions are identical to observations (P0)

**Status:** mitigated for the bundled example; still open for a future training
recorder.

**Baseline evidence:** at `dd2a594`, `examples/03_record_dataset.py:64-66` read
the measured state and used it as the action. The same pattern appeared in the
README snippet at `README.md:221-227`.

```python
obs = robot.get_observation()
action = list(obs["observation.state"])
robot.send_action({"action": action})
```

**Impact:** every frame is the pair `(q_t, q_t)`. A supervised policy trained on
this data learns the identity map `a = o` and will hold the current pose instead
of performing a task. Training loss falls to near zero, which makes the failure
look like success. This is the single largest defect in the recording path.

**Proposal:** record the command source as the action. For leader-follower
teleoperation, use the leader arm target; for scripted or policy rollouts, use
the commanded target from the planner or policy. Keep the follower's measured
state as `observation.state`.

**Done when:** a future training recorder accepts its requested action from an
independent source, stores the value returned by `send_action()`, and a test
injects distinct observation and action inputs to verify their provenance.

### R2. The script records the pre-clamp action, not the commanded action (P1)

**Status:** mitigated for the bundled diagnostic.

**Baseline evidence:** `send_action` clamps the target into the soft limits and
returns the clamped vector (`src/litearm_lerobot/robot.py:205-211`). At
`dd2a594`, the example ignored that return value and wrote the original list.

**Impact:** if a measured pose ever sits outside the soft limits, the dataset
records a value the arm never received. The mismatch is small and rare, but it
silently corrupts the exact pairing that imitation learning depends on.

**Implemented safeguard:** record the value returned by `send_action`:

```python
commanded = robot.send_action({"action": action})["action"]
```

Use `commanded` for `add_frame`, and optionally log when it differs from the
input.

**Verified by:** `tests/test_record_dataset_example.py` injects a return value
different from the measured state and checks the diagnostic stores that return
value without declaring an `action` feature.

### R3. The driver has no teleoperation or compliance mode (P1)

**Evidence:** `LiteArmRobotConfig.enable_on_connect` defaults to `True`
(`src/litearm_lerobot/config.py:50-52`). In that mode the servo loop actively
holds the measured pose. There is no config field that selects a zero-gravity,
gravity-compensated, or drag-teach mode.

**Impact:** dragging the arm by hand while recording may fight the servo loop.
Whether it does depends on the firmware mode and gains (needs hardware). The
example therefore cannot deliver the "zero-gravity, script, or teleop" workflow
that `examples/README.md:50` describes without extra steps that are not
documented.

**Proposal:** add an explicit mode (for example `control_mode` or
`zero_gravity`) that maps to the SDK/firmware call for compliant control, or
document the exact external procedure for entering zero-gravity before
recording. Do not infer the mode from `enable_on_connect`.

**Done when:** an operator can select compliant drag-teach from config, and the
recording example documents and uses it.

### R4. The docs describe the output as teleoperation data (P0)

**Status:** mitigated in the root README files, example guides, and developer
guides.

**Baseline evidence:** at `dd2a594`, `README.md:266` said example 03 recorded
"teleop episodes"; `examples/README.md:50` said it moved the arm in
"zero-gravity, script, or teleop". The recorded actions were measured
positions, not teleoperation commands.

**Impact:** readers will train on this data expecting demonstrations and get an
identity policy. The mismatch is a correctness problem, not a wording nit.

**Implemented safeguard:** the documentation calls the script a state-replay
diagnostic, says it has no independent action source, and states that its output
must not be used for policy training.

**Verified by:** `tests/test_record_dataset_example.py` enforces the runtime
guard; the documentation names the exact diagnostic feature written by the
script.

## 2. Sensor coverage

### R5. No camera or image features (P0)

**Evidence:** the recording script declares only `observation.state` and
`action` (`examples/03_record_dataset.py:38-41`), sets `use_videos=False`
(`:56`), and `LiteArmRobot` exposes no image feature
(`src/litearm_lerobot/robot.py:62-63`).

**Impact:** the dataset cannot train ACT, Diffusion Policy, or any vision-based
VLA model. This excludes the primary use case named in the project description.

**Proposal:** add camera configuration to the driver or a recording wrapper.
Declare `observation.images.<name>` with the correct `dtype` and `shape`, set
`use_videos=True`, and pass the image array in every `add_frame` call. Video
encoding needs `ffmpeg` in the environment; document that dependency.

**Done when:** a recorded episode loads in LeRobot with an image feature and a
policy can consume it.

### R6. No gripper or external sensor features (P2)

**Evidence:** the robot model is seven joints only. There is no gripper state,
force, or auxiliary sensor feature.

**Impact:** conditional. If the application uses a LiteGrip or any end effector,
the dataset cannot represent its state or command, so the policy cannot learn to
grasp. If the arm runs bare, this item does not apply.

**Proposal:** confirm the hardware scope. If a gripper is in scope, add its
state and action features to the driver and the recording script before building
a larger data collection effort.

**Done when:** the supported hardware matrix is documented, and every in-scope
end effector has a feature.

## 3. Reproducibility and timing

### R7. Frame timing uses `sleep` and drifts (P1)

**Evidence:** `examples/03_record_dataset.py:72` calls
`time.sleep(1.0 / args.fps)` after the I/O for each frame. The dataset is
created with the nominal `fps` (`:50-57`). The servo loop runs independently at
`config.servo_hz` (default 250 Hz).

**Impact:** the real sample period is `sleep` plus observation, send, and
`add_frame` time, so the achieved rate is below the declared `fps`. Consumers
interpret the data on the declared timeline, so the recorded motion can appear
faster or slower than it was. Jitter also adds noise to imitation learning.

**Proposal:** replace `sleep` with a monotonic deadline loop. Measure and log the
achieved rate per episode. If frames fall behind, decide explicitly whether to
drop, repeat, or accept them, and record that policy in the dataset metadata.

**Done when:** a ten-minute run reports achieved fps within a stated tolerance of
the configured value, and the policy for late frames is documented.

### R8. Datasets do not record the code and hardware state that produced them (P1)

**Evidence:** the recording script stores only the two feature streams and a
constant task string. It does not record the git commit, `litearm-python`
version, firmware version, config values (`servo_hz`, gains, limits), or port
selection.

**Impact:** two datasets recorded months apart cannot be compared or reproduced.
A policy trained on a dataset cannot be traced back to the driver behavior that
generated it.

**Proposal:** write a sidecar metadata file per recording session, for example
`recording_meta.json`, containing the git commit, installed versions, config
(with secrets excluded), start time, and achieved fps. Keep it next to the
dataset root.

**Done when:** every recording run produces the sidecar and its schema is
tested.

### R9. Dependency versions are not locked (P1)

**Evidence:** `pyproject.toml` declares `lerobot>=0.3.0,<0.5` and
`litearm-python>=2.1.0`; there is no lock file. CI installs `litearm-python`
from the repository default branch and the project with `pip install -e
".[dev]"` (`.github/workflows/ci.yml`). The local environment built for this
project uses Python 3.10; CI uses 3.12.

**Impact:** a fresh install today resolves `lerobot` 0.4.4; a future install can
resolve a different 0.4.x or 0.3.x. `litearm-python` tracks a moving git HEAD.
The LeRobot dataset API used by the example is version-sensitive (see R10), so
unpinned installs can break recording without any change in this repository.

**Proposal:** choose one pinning mechanism (`uv lock`, `pip-tools`, or a
`constraints.txt`) and commit it. Pin `litearm-python` to a commit or tag rather
than a branch. Add a CI job that installs from the lock file.

**Done when:** two fresh installs from the lock file produce the same versions,
and CI fails when the lock file is stale.

### R10. The recording path relies on LeRobot internal APIs (P1)

**Evidence:** the example and the README use
`dataset.episode_buffer = dataset.create_episode_buffer()`
(`examples/03_record_dataset.py:62`, `README.md:221`) and the
`create` → `add_frame` → `save_episode` sequence. These are not part of a stable
public contract across the allowed `0.3.x` to `0.4.x` range.

**Impact:** a minor LeRobot upgrade inside the allowed range can break recording
with no change to this package.

**Proposal:** wrap the sequence in a small helper in this repository, test it
against the oldest and newest allowed LeRobot versions, and document the tested
versions. Alternatively, narrow the dependency range to a tested minor.

**Done when:** CI runs the recording helper against the supported LeRobot
versions and fails loudly on API drift.

## 4. Episode and operator workflow

### R11. Episodes are fixed-length and tasks are constant (P2)

**Evidence:** `--episode-length` controls the frame count and `--task` is one
string for the whole run (`examples/03_record_dataset.py:28-31`, `:60-71`).

**Impact:** operators cannot mark task start, success, or failure. Episodes cut
mid-motion, and a run cannot contain more than one task. This limits the data to
scripted, uniform-length motions.

**Proposal:** add operator controls for start, stop, discard, and task label
(for example keyboard or a small control process). Store success or failure as
episode metadata rather than as a feature.

**Done when:** an operator can record variable-length episodes with per-episode
task labels and an outcome.

### R12. Interrupting the script loses the current episode (P2)

**Evidence:** `save_episode()` runs only after the inner loop completes
(`examples/03_record_dataset.py:73`). There is no `try`/`finally` around the
loop and no signal handling.

**Impact:** `Ctrl+C` or an exception discards all frames since the last
`save_episode()`, including data that may be expensive to re-record.

**Proposal:** catch `KeyboardInterrupt` and explicit stop signals around the
frame loop, then save or discard the partial buffer according to a documented
policy. Log which action was taken.

**Done when:** interrupting a run either saves the partial episode or clearly
reports that it was discarded, without leaving a corrupt dataset.

### R13. Dataset directory handling is minimal (P2)

**Evidence:** `--repo-id` is joined directly with `--root` and used as the
output directory (`examples/03_record_dataset.py:44-49`). The script refuses to
overwrite but offers no resume or append path.

**Impact:** operators must invent unique `repo-id` values by hand, and
re-recording means deleting data manually. A typo creates a new dataset instead
of failing.

**Proposal:** validate `repo-id` against the LeRobot naming rules, add explicit
`--overwrite` and `--resume` behavior, and print the resolved output path before
recording starts.

**Done when:** invalid IDs fail before connecting to the arm, and overwrite or
resume requires an explicit flag.

### R14. No CLI or YAML recording entry point (P2)

**Evidence:** `register()` patches `make_robot_from_config`
(`src/litearm_lerobot/utils.py:5-37`), but the repository ships no
`lerobot-record`-style command or example YAML. The only recording entry point
is the Python script.

**Impact:** users who expect the standard LeRobot CLI must write their own
entry point, and there is no tested configuration file to copy.

**Proposal:** provide an example YAML config and document how to run LeRobot's
recording CLI with `type: litearm`, or state explicitly that the script is the
supported path.

**Done when:** a user can record using only documented commands, with no custom
code.

## 5. Testing and operations

### R15. The recording path has no tests (P1)

**Evidence:** `tests/` contains driver, safety, servo, config, and contract
tests. No test references `LeRobotDataset`, `add_frame`, or the recording
example.

**Impact:** R1, R2, R7, and R10 can all regress without a failing test. The
project's main user-facing workflow is the least covered.

**Proposal:** add an integration test that uses the fake arm, a temporary root,
one short episode, and asserts the dataset can be reopened with the expected
features, frame count, dtype, and action source. Keep it hardware-free.

**Done when:** the test fails when the action source is changed back to the
measured state, and passes on both supported LeRobot boundary versions.

### R16. Long sessions have no supervision surface (P2)

**Evidence:** a servo-loop failure is stored on `ServoLoop.error` and raised by
the next `get_observation()` or `send_action()`
(`src/litearm_lerobot/robot.py:161-172`). The recording loop calls both every
frame, so it will notice the death, but there is no heartbeat, no preflight
check beyond `connect()`, and no operator-facing status.

**Impact:** failures surface as an exception in the middle of a session. There
is no early warning for a drooping arm, a saturated joint, or a degrading link.

**Proposal:** add a preflight that prints joint count, limits, port, enabled
state, and achieved state rate before recording starts. Add periodic logging of
saturation and rate. Do not add a background thread that silently swallows
errors.

**Done when:** an operator can see the preflight summary before motion and a
saturation warning during recording.

## Suggested order

1. R1 and R4 together. Fix the action source and make the docs match it.
2. R2, R15. Start recording the commanded value and lock the behavior with a
   test.
3. R5. Add cameras; without them the dataset cannot feed the intended models.
4. R7, R8, R9, R10. Make recordings comparable and installs reproducible.
5. R11-R14, R16. Improve the operator workflow once the data is useful.

## Constraints to preserve

Any fix must keep these existing invariants:

- `send_action` stays non-blocking; motion stays in the servo loop.
- `disconnect()` hands the arm back with a zero-motion `movej`, never a bare
  `disable()` unless the operator opted in.
- The soft-limit margin stays at `0.01`; raising it silently removes J4's
  positive range.
- Feature names stay `observation.state` and `action` for joint data so existing
  consumers keep working.
- A dead servo loop stays loud: no code may return stale state as if it were
  fresh.

## Do not

- **Do not** train or evaluate a manipulation policy on the current example's
  output. The actions are the measured states; the policy can only learn to hold.
- **Do not** change the action source silently. If the script starts recording a
  leader command or a planner target, say so in the README, the examples README,
  and the dataset metadata in the same change.
- **Do not** widen the LeRobot version range to make an internal API work.
  Pin and test instead.
