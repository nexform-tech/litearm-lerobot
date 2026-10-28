"""Regression tests for the non-training dataset example."""
from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


def _load_example():
    path = Path(__file__).parents[1] / "examples" / "03_record_dataset.py"
    spec = importlib.util.spec_from_file_location("record_dataset_example", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_recorder_refuses_before_opening_hardware(monkeypatch):
    module = _load_example()
    opened = False

    def fail_if_constructed(*args, **kwargs):
        nonlocal opened
        opened = True
        raise AssertionError("robot must not be constructed without explicit opt-in")

    monkeypatch.setattr(module, "LiteArmRobot", fail_if_constructed)

    with pytest.raises(SystemExit, match="no independent action source"):
        module.main([])

    assert opened is False


def test_state_replay_demo_has_no_training_action_feature(monkeypatch, tmp_path):
    module = _load_example()
    created = {}
    frames = []

    class FakeDataset:
        episode_buffer = None

        @classmethod
        def create(cls, **kwargs):
            created.update(kwargs)
            return cls()

        def create_episode_buffer(self):
            return {}

        def add_frame(self, frame):
            frames.append(frame)

        def save_episode(self):
            pass

    class FakeRobot:
        name = "litearm"
        config = SimpleNamespace(num_joints=2)

        def __init__(self, config):
            self.requested = None

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def get_observation(self):
            return {"observation.state": [0.1, 0.2]}

        def send_action(self, action):
            self.requested = action
            return {"action": [0.1, 0.15]}

    monkeypatch.setattr(module, "LeRobotDataset", FakeDataset)
    monkeypatch.setattr(module, "LiteArmRobot", FakeRobot)
    monkeypatch.setattr(module, "LiteArmRobotConfig", lambda **kwargs: kwargs)
    monkeypatch.setattr(module.time, "sleep", lambda _: None)

    module.main([
        "--allow-state-replay-demo",
        "--root", str(tmp_path),
        "--episode-length", "1",
    ])

    assert "action" not in created["features"]
    assert module.STATE_REPLAY_FEATURE in created["features"]
    assert len(frames) == 1
    assert "action" not in frames[0]
    assert frames[0][module.STATE_REPLAY_FEATURE].tolist() == pytest.approx([0.1, 0.15])
    assert frames[0]["task"].startswith("diagnostic.state_replay:")
