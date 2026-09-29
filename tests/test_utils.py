"""Tests for the LeRobot CLI integration helper."""
from __future__ import annotations

import pytest

from litearm_lerobot import register


def test_register_is_idempotent():
    import litearm_lerobot.robot  # noqa: F401  (imports LiteArmRobot class)

    from lerobot.robots import utils as robot_utils

    register()
    patched = robot_utils.make_robot_from_config
    assert getattr(patched, "_litearm_registered", False) is True
    register()  # second call must not break anything
    assert robot_utils.make_robot_from_config is patched


def test_register_returns_litearm_robot(monkeypatch):
    """`register()` 之后，`type == 'litearm'` 的配置能造出 LiteArmRobot。"""
    import litearm

    from litearm_lerobot import LiteArmRobot

    class _Arm:
        def __init__(self, **kw):
            self.n = 7
            self.params = self

        def connect(self):
            return self

        def get_state(self, refresh=False):
            from conftest import FakeMsg, FakeRobotState, FakeJointState
            return FakeMsg(FakeRobotState(
                joints=[FakeJointState() for _ in range(7)]))

        def all_joint_params(self):
            from conftest import FakeJointParam, FakeArm
            return [FakeJointParam(*lo_hi) for lo_hi in FakeArm.LIMITS]

        def enable(self, attempts=12):
            pass

        def close(self):
            pass

    monkeypatch.setattr(litearm, "Arm", lambda *a, **kw: _Arm())
    register()

    from lerobot.robots import utils as robot_utils
    from litearm_lerobot import LiteArmRobotConfig

    robot = robot_utils.make_robot_from_config(LiteArmRobotConfig())
    assert isinstance(robot, LiteArmRobot)


def test_register_falls_through_for_other_types(monkeypatch):
    from lerobot.robots import utils as robot_utils

    sentinel = object()

    def fake_original(config):
        return sentinel

    monkeypatch.setattr(robot_utils, "make_robot_from_config", fake_original)
    register()

    class _OtherConfig:
        type = "so100_follower"

    assert robot_utils.make_robot_from_config(_OtherConfig()) is sentinel
