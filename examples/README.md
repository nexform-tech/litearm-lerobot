# Examples

Runnable examples for engineers bringing the arm up for the first time or
verifying a driver change. Each example is a plain script you run from the
repository root with the package installed (see [../README.md](../README.md)).

> **Warning:** examples 02 and 03 **drive the real arm**. Start with low speeds and keep
> your hand near the emergency stop.

## Prerequisites

The arm is connected to this machine over USB CDC, directly — there is no
litearm-server in this path. The port is auto-detected by VID:PID `1d50:606f`;
pass `--port /dev/ttyACM0` to pick one explicitly.

```bash
pip install -e .
```

## 01 — Read observations (read-only)

```bash
python examples/01_read_observation.py --count 5
```

Prints the observation features and the latest joint positions.

## 02 — Send a sinusoidal action (motion)

```bash
python examples/02_send_action.py \
  --duration 10 --amplitude 0.05
```

Drives a small sine trajectory through `robot.send_action()`. Keep `--amplitude`
small on first runs.

> **Do not** pass absolute joint angles as the trajectory. That commands the arm
> across its full range on the first step. Both example 02 and any policy
> rollout should offset from the pose read at start-up.

## 03 — Record a state-replay diagnostic

This example has no independent action source and cannot produce policy-training
data. It refuses to open the arm unless you explicitly opt into the diagnostic.

```bash
python examples/03_record_dataset.py \
  --allow-state-replay-demo \
  --repo-id litearm_state_replay_demo --root diagnostic_data \
  --episodes 1 --episode-length 50 --task bench_check
```

The example sends each measured pose back as a hold target. It records
`observation.state` and `diagnostic.state_replay_target`, but deliberately does
not create an `action` feature. **Do not** use the output for policy training.

Chinese version: [README.zh-CN.md](README.zh-CN.md)
