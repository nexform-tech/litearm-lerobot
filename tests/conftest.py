"""Shared fixtures: a fake LiteArm that records every call (no network)."""
from __future__ import annotations

import pytest

import litearm
from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig


class FakeArm:
    """Minimal stand-in for litearm.Arm — no zenoh, no hardware."""

    def __init__(self, q=None):
        self.calls = []
        self.q = q if q is not None else [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0]
        self.closed = False

    def get_state(self):
        self.calls.append("get_state")
        return {
            "q": list(self.q),
            "dq": [0.0] * 7,
            "tau": [0.0] * 7,
            "fault": [],
            "errs": [],
            "temps": [],
            "state": "ready",
        }

    def enable(self):
        self.calls.append("enable")

    def hold(self):
        self.calls.append("hold")

    def movej(self, q_target, speed=1.0, settle_s=1.0, **kwargs):
        self.calls.append(("movej", list(q_target), speed, settle_s))
        self.q = list(q_target)
        return True

    def close(self):
        self.calls.append("close")
        self.closed = True


@pytest.fixture
def fake_arm():
    return FakeArm()


@pytest.fixture
def make_robot(monkeypatch, fake_arm):
    """Build a LiteArmRobot whose litearm.Arm is the FakeArm."""

    def _make(**cfg):
        monkeypatch.setattr(litearm, "Arm", lambda **kw: fake_arm)
        defaults = {"endpoint": "tcp/127.0.0.1:7447", "arm_id": "armA"}
        defaults.update(cfg)
        robot = LiteArmRobot(LiteArmRobotConfig(**defaults))
        return robot, fake_arm

    return _make
