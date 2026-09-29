"""伺服环测试 —— 重点在**判别力**：判据必须能区分"循环跑了"与"臂真会动"。"""
from __future__ import annotations

import time

import pytest

import litearm
from conftest import FakeArm
from litearm_lerobot.safety import read_safe_limits
from litearm_lerobot.servo import (
    DEFAULT_ACCEL_LIMIT,
    DEFAULT_K_D,
    DEFAULT_K_P,
    DEFAULT_SPEED_LIMIT,
    ServoLoop,
    hold_at_current,
)


def _loop(arm, **kw):
    kw.setdefault("hz", 200.0)
    kw.setdefault("engage_sec", 0.0)
    return ServoLoop(arm, read_safe_limits(arm), **kw)


def _wait(predicate, timeout=2.0, interval=0.005):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


# ── prime / engage ─────────────────────────────────────────────────────────

def test_prime_happens_before_any_target(fake_arm):
    """不喂目标也必须先 prime —— 固件不支持 0x08 时要在这一步就响。"""
    loop = _loop(fake_arm)
    loop.start()
    try:
        assert _wait(lambda: bool(fake_arm.calls_named("joint_follow"))), "prime 没发"
    finally:
        loop.stop()


def test_unsupported_firmware_surfaces_at_start(fake_arm):
    """固件没有 0x08 ⇒ prime 那一步就抛，不等到第一次 send_action。"""
    fake_arm.raise_on["joint_follow"] = litearm.UnsupportedByFirmwareError("no 0x08")
    loop = _loop(fake_arm)
    loop.start()
    try:
        assert _wait(lambda: loop.error is not None)
        assert isinstance(loop.error, litearm.UnsupportedByFirmwareError)
    finally:
        loop.stop()


def test_engage_holds_at_current_pose_with_soft_gains(fake_arm):
    loop = _loop(fake_arm, hz=200.0, engage_sec=0.05)
    loop.start()
    try:
        _wait(lambda: len(fake_arm.calls_named("joint_follow")) >= 2)
    finally:
        loop.stop()
    kps = [c[3][0] for c in fake_arm.calls_named("joint_follow")]
    # ⚠ 第 0 拍是 **prime**（用跟随增益托住），engage 从第 1 拍起。
    assert kps[0] == pytest.approx(DEFAULT_K_P[0]), "prime 应使用跟随增益"
    assert kps[1] == pytest.approx(15.0), "engage 应从第 1 拍起用低刚度"


# ── 目标跟踪 ───────────────────────────────────────────────────────────────

def test_reference_converges_to_target(fake_arm):
    loop = _loop(fake_arm)
    loop.start()
    try:
        target = [0.5] * 7
        loop.set_target(target)
        assert _wait(
            lambda: loop.q_cmd is not None
            and max(abs(a - b) for a, b in zip(loop.q_cmd, target)) < 1e-3,
            timeout=3.0,
        ), f"没收敛：{loop.q_cmd}"
    finally:
        loop.stop()


def test_dq_is_never_all_zero_while_moving(fake_arm):
    """⚠ 判别力判据。

    `move_js` 上 `dq=0` 会让固件参考冻结、臂纹丝不动且**不报错**；`joint_follow`
    上 `dq` 是速度前馈，全 0 意味着内环少了速度项。两种都属"循环在跑但臂不对"。
    所以：移动中必须出现过非零 dq。
    """
    loop = _loop(fake_arm)
    loop.start()
    try:
        loop.set_target([0.5] * 7)
        assert _wait(lambda: any(
            any(abs(v) > 1e-6 for v in c[2])
            for c in fake_arm.calls_named("joint_follow")
        ), timeout=2.0), "整段里 dq 恒为 0 —— 参考根本没在推进"
    finally:
        loop.stop()


def test_tick_rate_is_close_to_the_configured_hz(fake_arm):
    """⚠ 判据的判据是**耗时**：线程没真跑起来时上面几条仍可能成立。"""
    loop = _loop(fake_arm, hz=100.0)
    loop.start()
    try:
        loop.set_target([0.3] * 7)
        time.sleep(0.5)
        n = len(fake_arm.calls_named("joint_follow"))
    finally:
        loop.stop()
    # 0.5 s @100 Hz ≈ 50 拍；放宽到 0.5× 以容忍 CI 抖动，但足以挡住"没跑"
    assert n > 25, f"0.5 s 只发了 {n} 拍 —— 节拍没跑起来"


def test_one_send_per_tick(fake_arm):
    """spec §9.5-2：每拍**恰好**一次下发（往返数 == 拍数，允许 1 拍）。

    ⚠ 与上一条互补：一个每拍发两次的循环能通过"节拍不低于下限"，但过不了这里。
    """
    loop = _loop(fake_arm, hz=100.0)
    loop.start()
    try:
        loop.set_target([0.3] * 7)
        n0 = len(fake_arm.calls_named("joint_follow"))
        t0 = time.monotonic()
        time.sleep(0.5)
        elapsed = time.monotonic() - t0
        n1 = len(fake_arm.calls_named("joint_follow"))
    finally:
        loop.stop()
    expected = elapsed * 100.0
    assert n1 - n0 == pytest.approx(expected, abs=2.0), (
        f"{elapsed:.3f} s 内发了 {n1 - n0} 拍，期望约 {expected:.0f} 拍"
    )


def test_before_first_target_it_holds_the_measured_pose(fake_arm):
    loop = _loop(fake_arm)
    loop.start()
    try:
        _wait(lambda: len(fake_arm.calls_named("joint_follow")) >= 3)
    finally:
        loop.stop()
    for _, q, dq, _, _ in fake_arm.calls_named("joint_follow"):
        assert max(abs(v) for v in dq) < 1e-9, "还没给目标就不许动"
        assert max(abs(a - b) for a, b in zip(q, fake_arm.q)) < 1e-6


# ── 下发点 ─────────────────────────────────────────────────────────────────

def test_joint_follow_actuator_sends_kp_kd(fake_arm):
    loop = _loop(fake_arm, actuator="joint_follow")
    loop.start()
    try:
        _wait(lambda: fake_arm.calls_named("joint_follow"))
    finally:
        loop.stop()
    _, _, _, kp, kd = fake_arm.calls_named("joint_follow")[0]
    assert kp == DEFAULT_K_P
    assert kd == DEFAULT_K_D


def test_move_js_actuator_does_not_send_gains(fake_arm):
    """`move_js` 没有随帧增益通道 —— 必须**只**传 (q, dq)，不传 kp/kd。"""
    loop = _loop(fake_arm, actuator="move_js")
    loop.start()
    try:
        _wait(lambda: fake_arm.calls_named("move_js"))
    finally:
        loop.stop()
    calls = fake_arm.calls_named("move_js")
    assert calls, "move_js 没被调用"
    assert not fake_arm.calls_named("joint_follow")
    name, q, dq = calls[0]
    assert len(q) == 7
    assert dq is not None and len(dq) == 7


# ── 异常路径 ───────────────────────────────────────────────────────────────

def test_send_failure_takes_over_and_records_the_error(fake_arm):
    loop = _loop(fake_arm)
    loop.start()
    try:
        _wait(lambda: fake_arm.calls_named("joint_follow"))
        fake_arm.raise_on["joint_follow"] = RuntimeError("链路坏了")
        assert _wait(lambda: loop.error is not None)
        assert isinstance(loop.error, RuntimeError)
        assert fake_arm.calls_named("movej"), "失败后必须受控接管（movej 回当前位姿）"
    finally:
        loop.stop()


def test_stop_is_idempotent(fake_arm):
    loop = _loop(fake_arm)
    loop.start()
    loop.stop()
    loop.stop()


def test_stop_without_start_is_a_noop(fake_arm):
    loop = _loop(fake_arm)
    loop.stop()


# ── hold_at_current ────────────────────────────────────────────────────────

def test_hold_at_current_moves_to_the_measured_pose(fake_arm):
    fake_arm.q = [0.4] * 7
    hold_at_current(fake_arm)
    call = fake_arm.calls_named("movej")[-1]
    assert call[1] == pytest.approx([0.4] * 7)
    assert call[2] == pytest.approx(0.3)


def test_hold_at_current_never_disables(fake_arm):
    hold_at_current(fake_arm)
    assert not fake_arm.calls_named("disable")
