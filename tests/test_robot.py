"""Tests for the litearm-lerobot LeRobot driver."""
from __future__ import annotations

import time

import pytest

from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig


def test_config_choice_type():
    cfg = LiteArmRobotConfig()
    assert cfg.type == "litearm"           # draccus choice name
    assert cfg.num_joints == 7
    assert cfg.endpoint == "tcp/127.0.0.1:7447"
    assert cfg.arm_id == "armA"


def test_features(make_robot):
    robot, _ = make_robot()
    assert robot.observation_features == {"observation.state": (7,)}
    assert robot.action_features == {"action": (7,)}


def test_not_connected_by_default(make_robot):
    robot, _ = make_robot()
    assert robot.is_connected is False
    assert robot.is_calibrated is True          # absolute encoders: no homing
    with pytest.raises(RuntimeError):
        robot.get_observation()
    with pytest.raises(RuntimeError):
        robot.send_action({"action": [0.0] * 7})


def test_connect_and_observe(make_robot):
    robot, arm = make_robot()
    robot.connect()
    assert robot.is_connected
    assert "enable" in arm.calls
    obs = robot.get_observation()
    assert list(obs["observation.state"]) == arm.q
    assert len(obs["observation.state"]) == 7
    robot.disconnect()


def test_send_action_blocking(make_robot):
    robot, arm = make_robot(use_commander=False)
    robot.connect()
    target = [0.0] * 7
    out = robot.send_action({"action": target})
    assert out == {"action": target}
    assert ("movej", target, robot.config.movej_speed, robot.config.settle_s) in arm.calls


def test_send_action_commander(make_robot):
    robot, arm = make_robot(use_commander=True, settle_s=0.01)
    robot.connect()
    target = [0.5] * 7
    out = robot.send_action({"action": target})
    assert out == {"action": target}                 # returns immediately
    deadline = time.monotonic() + 2.0
    while not any(c[0] == "movej" for c in arm.calls if isinstance(c, tuple)) \
            and time.monotonic() < deadline:
        time.sleep(0.02)
    assert any(c[0] == "movej" for c in arm.calls if isinstance(c, tuple))
    robot.disconnect()


def test_send_action_wrong_dimension(make_robot):
    robot, _ = make_robot(use_commander=False)
    robot.connect()
    with pytest.raises(ValueError):
        robot.send_action({"action": [0.0] * 6})
    with pytest.raises(ValueError):
        robot.send_action({})


def test_calibrate_is_noop(make_robot):
    robot, _ = make_robot()
    assert robot.calibrate() is None
    robot.configure()


def test_disconnect_holds_and_closes(make_robot):
    robot, arm = make_robot()
    robot.connect()
    robot.disconnect()
    assert arm.closed
    assert "hold" in arm.calls
    assert "disable" not in arm.calls
    assert robot.is_connected is False


def test_disconnect_disables_when_configured(make_robot):
    robot, arm = make_robot(disable_on_disconnect=True)
    robot.connect()
    robot.disconnect()
    assert arm.closed
    assert "disable" in arm.calls
    assert "hold" not in arm.calls
    assert robot.is_connected is False


def test_commander_backoff_on_errors(make_robot):
    robot, arm = make_robot(use_commander=True, settle_s=0.01)
    robot.connect()
    # Make movej always fail
    original_movej = arm.movej
    call_count = [0]

    def failing_movej(*args, **kwargs):
        call_count[0] += 1
        raise RuntimeError("simulated failure")

    arm.movej = failing_movej
    try:
        robot.send_action({"action": [0.5] * 7})
        # Wait for a few retries
        deadline = time.monotonic() + 3.0
        while call_count[0] < 3 and time.monotonic() < deadline:
            time.sleep(0.05)
        # After 5 consecutive errors the backoff should be at least 5 s,
        # so we should see at most a handful of calls (not hundreds).
        assert call_count[0] < 20  # tight loop would be hundreds
    finally:
        arm.movej = original_movej
        robot.disconnect()
