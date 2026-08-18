# Examples

Each example is a plain script you run from the repository root with the package
installed (see [../README.md](../README.md)).

> ⚠️ Examples 02 and 03 **drive the real arm**. Start with low speeds and keep
> your hand near the emergency stop.

## Prerequisites

```bash
pip install -e .
```

## 01 — Read observations (read-only)

```bash
python examples/01_read_observation.py \
  --endpoint tcp/192.168.31.237:7447 --count 5
```

Prints the observation features and the latest joint positions.

## 02 — Send a sinusoidal action (motion)

```bash
python examples/02_send_action.py \
  --endpoint tcp/192.168.31.237:7447 \
  --duration 10 --amplitude 0.05 --speed 0.3
```

Drives a small sine trajectory through `robot.send_action()`. Keep `--amplitude`
small on first runs.

## 03 — Record a LeRobotDataset episode

```bash
python examples/03_record_dataset.py \
  --endpoint tcp/192.168.31.237:7447 \
  --repo-id litearm_demo --root data \
  --episodes 1 --episode-length 50 --task push
```

Moves the arm (zero-gravity, script, or teleop) while recording
`observation.state` and `action` frames. The dataset lands at `data/litearm_demo/`,
usable directly by LeRobot training scripts.

Chinese version: [README.zh-CN.md](README.zh-CN.md)
