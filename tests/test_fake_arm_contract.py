"""FakeArm 必须与真 SDK 同形 —— 否则测试又会自洽地错（spec §1.2）。"""
from __future__ import annotations

import inspect

import litearm
import pytest

from conftest import FakeArm


def test_fake_arm_has_no_hold_because_the_real_one_has_none():
    """这条是那次事故的直接墓碑：旧 FakeArm 有 hold()，而真 Arm 没有。"""
    assert not hasattr(litearm.Arm, "hold"), "真 SDK 又有 hold() 了？先读 spec §1.1"
    assert not hasattr(FakeArm, "hold")


def test_fake_arm_constructor_mirrors_the_real_one():
    params = inspect.signature(FakeArm.__init__).parameters
    # ⚠ 只断言"不含旧 server 关键字"，不比对全签名 —— FakeArm 不需要复刻
    #   真构造函数的每一个可调项（transport_factory 等与测试无关）。
    for gone in ("endpoint", "arm_id", "query_timeout", "settle_s"):
        assert gone not in params, f"FakeArm 还在收 {gone} —— 那是旧 API"


def test_fake_arm_accepts_the_kwargs_we_pass_to_movej():
    """真 Arm.movej 的关键字，FakeArm 必须收得下，且**不许**收 settle_s。"""
    real = inspect.signature(litearm.Arm.movej).parameters
    fake = inspect.signature(FakeArm.movej).parameters
    for kw in ("q", "speed"):
        assert kw in real, f"真 SDK 的 movej 没有 {kw}"
        assert kw in fake, f"FakeArm.movej 没有 {kw}"
    assert "settle_s" not in real
    assert "settle_s" not in fake


def test_fake_arm_get_state_returns_an_envelope_not_a_dict():
    """真 get_state() 返回 Msg；旧的 FakeArm 返回 dict —— 正是那次静默漂移。"""
    msg = FakeArm().get_state()
    assert hasattr(msg, "value"), "FakeArm.get_state 必须回信封（Msg），不是 dict"
    st = msg.value
    for attr in ("q", "dq", "joints", "flags", "flag_names", "joint_fault", "faulted"):
        assert hasattr(st, attr), f"RobotState 缺 {attr}"


def test_fake_arm_exposes_params_all_joint_params():
    jps = FakeArm().params.all_joint_params()
    assert len(jps) == 7
    for p in jps:
        assert hasattr(p, "q_min") and hasattr(p, "q_max")
        assert p.q_min < p.q_max
