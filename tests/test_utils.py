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
    import litearm
    from litearm_lerobot import LiteArmRobotConfig

    class _Arm:
        def get_state(self):
            return {"q": [0.0] * 7}

        def enable(self):
            pass

        def close(self):
            pass

    monkeypatch.setattr(litearm, "Arm", lambda **kw: _Arm())
    register()

    from lerobot.robots import utils as robot_utils

    robot = robot_utils.make_robot_from_config(
        LiteArmRobotConfig(endpoint="tcp/127.0.0.1:7447", use_commander=False)
    )
    from litearm_lerobot import LiteArmRobot

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
