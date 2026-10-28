"""Offline lifecycle and safety tests for the shared LiteGrip controller."""
from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from litearm_lerobot import GripperError, LiteGripController, LiteGripOptions


def _event_names(events):
    return [event[0] if isinstance(event, tuple) else event for event in events]


def _wait_for(events, name, timeout=1.0):
    deadline = time.monotonic() + timeout
    while name not in _event_names(events) and time.monotonic() < deadline:
        time.sleep(0.005)
    return name in _event_names(events)


def _connected(fake_gripper, **overrides):
    options = LiteGripOptions(enabled=True, hold_hz=50.0, **overrides)
    controller = LiteGripController(options)
    controller.connect()
    return controller, fake_gripper.instances[-1]


def test_connect_loads_calibration_and_starts_the_hold_stream(fake_gripper):
    controller, gripper = _connected(fake_gripper)
    try:
        names = _event_names(gripper.events)
        assert names.index("connect") < names.index("enable")
        assert _wait_for(gripper.events, "mit")
    finally:
        controller.disconnect()
    names = _event_names(gripper.events)
    assert "stop" in names
    assert names[-1] == "disconnect"


def test_capture_state_reports_the_opening_in_radians(fake_gripper):
    controller, gripper = _connected(fake_gripper)
    try:
        gripper.state.position_mm = 42.5
        sample = controller.capture_state()
        assert sample.position_rad == pytest.approx(42.5 / gripper.config.rad_to_mm)
    finally:
        controller.disconnect()


@pytest.mark.parametrize(
    ("mutate", "match"),
    [
        (lambda gripper: setattr(gripper.state, "error_code", 0x9), "not enabled"),
        (lambda gripper: setattr(gripper.state, "data_age_s", 1.0), "stale"),
        (lambda gripper: setattr(gripper.state, "temperature_coil", 80), "temperature"),
        (lambda gripper: setattr(gripper.state, "position_mm", 100.0), "outside"),
    ],
)
def test_capture_fails_closed_on_unsafe_feedback(fake_gripper, mutate, match):
    controller, gripper = _connected(fake_gripper)
    try:
        mutate(gripper)
        with pytest.raises(GripperError, match=match):
            controller.capture_state()
    finally:
        controller.disconnect()


def test_enable_failure_names_the_can_link(fake_gripper, monkeypatch):
    def fail_enable(self, retries=None):
        return SimpleNamespace(
            ok=False, tries=3, state=SimpleNamespace(error_code=0)
        )

    monkeypatch.setattr(fake_gripper, "enable", fail_enable)
    controller = LiteGripController(LiteGripOptions(enabled=True, hold_hz=50.0))
    with pytest.raises(GripperError, match="can0 is UP"):
        controller.connect()
    assert fake_gripper.instances[-1].disconnected


def test_request_opening_below_the_midpoint_grabs(fake_gripper):
    controller, gripper = _connected(fake_gripper, grasp_hold_s=0.01)
    try:
        controller.request_opening_rad(0.0)
        assert _wait_for(gripper.events, "grasp")
    finally:
        controller.disconnect()


def test_request_opening_above_the_midpoint_opens(fake_gripper):
    controller, gripper = _connected(fake_gripper)
    try:
        controller.request_opening_rad(controller.travel_rad)
        assert _wait_for(gripper.events, "open")
    finally:
        controller.disconnect()


def test_request_opening_rejects_a_non_finite_target(fake_gripper):
    controller, _ = _connected(fake_gripper)
    try:
        with pytest.raises(GripperError, match="finite"):
            controller.request_opening_rad(float("nan"))
    finally:
        controller.disconnect()


def test_travel_rad_requires_a_loaded_calibration():
    controller = LiteGripController(LiteGripOptions(enabled=True))
    with pytest.raises(GripperError, match="calibration"):
        controller.travel_rad


@pytest.mark.parametrize("value", [0.0, -61.01, float("nan"), float("inf")])
def test_unusable_rad_to_mm_is_rejected(value):
    gripper = SimpleNamespace(config=SimpleNamespace(rad_to_mm=value))
    with pytest.raises(GripperError, match="rad_to_mm"):
        LiteGripController._calibrated_rad_to_mm(gripper)


@pytest.mark.parametrize(
    "options",
    [
        LiteGripOptions(enabled=True, speed_mm_s=0.0),
        LiteGripOptions(enabled=True, grasp_force_n=50.0),
        LiteGripOptions(enabled=True, open_key="x", close_key="x"),
        LiteGripOptions(enabled=True, mount="sideways"),
        LiteGripOptions(enabled=True, open_position_mm=0.0),
        LiteGripOptions(enabled=True, position_tolerance_mm=0.0),
    ],
)
def test_invalid_gripper_options_are_rejected(options):
    with pytest.raises(ValueError):
        options.validate()
