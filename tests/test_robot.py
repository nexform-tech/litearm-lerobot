"""LiteArmRobot 生命周期测试。"""
from __future__ import annotations

import threading
import time

import pytest

import litearm
from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig


def _wait(predicate, timeout=2.0, interval=0.005):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


# ── 元数据 ─────────────────────────────────────────────────────────────────

def test_features(make_robot):
    robot, _ = make_robot()
    assert robot.observation_features == {"observation.state": (7,)}
    assert robot.action_features == {"action": (7,)}


def test_not_connected_by_default(make_robot):
    robot, _ = make_robot()
    assert robot.is_connected is False
    assert robot.is_calibrated is True          # 绝对式编码器，无需回零
    with pytest.raises(RuntimeError):
        robot.get_observation()
    with pytest.raises(RuntimeError):
        robot.send_action({"action": [0.0] * 7})


def test_calibrate_and_configure_are_noops(make_robot):
    robot, _ = make_robot()
    assert robot.calibrate() is None
    assert robot.configure() is None


# ── 连接 ───────────────────────────────────────────────────────────────────

def test_connect_passes_port_and_move_timeout(make_robot):
    robot, arm = make_robot(port="/dev/ttyACM9", move_timeout=7.5)
    robot.connect()
    try:
        assert arm.connected
        assert arm.port == "/dev/ttyACM9"
        # 逐个断言我们**实际传**的两个关键字（多一个少一个都算契约变了）
        assert arm.ctor_kwargs == {"port": "/dev/ttyACM9", "move_timeout": 7.5}
        assert arm.calls_named("enable"), "enable_on_connect 默认 True"
        assert robot.is_connected
    finally:
        robot.disconnect()


def test_connect_is_idempotent(make_robot):
    robot, arm = make_robot()
    robot.connect()
    n_before = len(arm.calls_named("connect"))
    robot.connect()
    assert len(arm.calls_named("connect")) == n_before, "第二次 connect 不该重连"
    robot.disconnect()


def test_connect_rejects_joint_count_mismatch(make_robot):
    """1J 台架板上 `arm.n == 1` —— 配错必须响亮。"""
    robot, arm = make_robot()
    arm.n = 1
    with pytest.raises(ValueError, match="关节数"):
        robot.connect()
    assert arm.closed


def test_connect_raises_when_limits_unreadable(monkeypatch, fake_arm):
    from conftest import _NoParams

    fake_arm.params = _NoParams()
    monkeypatch.setattr(litearm, "Arm", lambda *a, **kw: fake_arm)
    robot = LiteArmRobot(LiteArmRobotConfig())
    with pytest.raises(Exception, match="关节参数|限位|limits"):
        robot.connect()
    assert fake_arm.closed, "连接失败必须把链路收干净"


def test_enable_failure_is_loud(monkeypatch, fake_arm):
    """直连下 enable 失败 = 臂不会动 ⇒ 必须抛，不再吞异常（旧代码吞了）。"""
    fake_arm.raise_on["enable"] = litearm.CommandRejectedError("emergency")
    monkeypatch.setattr(litearm, "Arm", lambda *a, **kw: fake_arm)
    robot = LiteArmRobot(LiteArmRobotConfig())
    with pytest.raises(Exception):
        robot.connect()
    assert fake_arm.closed


def test_move_js_plus_explicit_gains_warns(make_robot, caplog):
    """`k_p`/`k_d` 在 `move_js` 下会被忽略 —— 不许静默。"""
    robot, _ = make_robot(actuator="move_js", k_p=[1.0] * 7)
    with caplog.at_level("WARNING"):
        robot.connect()
    try:
        assert any("k_p" in r.message or "增益" in r.message
                   for r in caplog.records), "该告警没发"
    finally:
        robot.disconnect()


# ── 只读会话（enable_on_connect=False） ────────────────────────────────────

def test_no_enable_connect_never_commands_the_arm(make_robot):
    """`enable_on_connect=False` 是**真·只读**会话：伺服环不起，一拍都不发。

    ⚠ 回归墓碑：旧实现照样起伺服环，而 `0x08` 首帧在未使能时被固件整帧拒
    （`control_loop.c:966` `if (!g_arm.enabled) return 0x03;`）⇒ 线程第一拍就死，
    之后每个 `get_observation` / `send_action` 都抛。只读模式必须真的不动臂。
    """
    robot, arm = make_robot(enable_on_connect=False)
    robot.connect()
    try:
        assert not arm.calls_named("enable"), "只读模式不许调 enable"
        assert not arm.enabled
        time.sleep(0.25)          # 250 Hz 下够发 ~60 拍
        assert not arm.calls_named("joint_follow"), "只读模式发了伺服帧"
        assert not arm.calls_named("move_js"), "只读模式发了 move_js"
        assert not arm.calls_named("movej"), "只读模式发了 movej"
    finally:
        robot.disconnect()


def test_no_enable_connect_still_reads_observations(make_robot):
    """读观测**不需要**使能 —— `get_state()` 吃的是固件的被动状态流。"""
    robot, arm = make_robot(enable_on_connect=False)
    arm.q = [0.11, 0.22, 0.33, 0.0, 0.0, 0.0, 0.0]
    robot.connect()
    try:
        obs = robot.get_observation()
        assert obs["observation.state"] == pytest.approx(arm.q)
        assert robot.is_connected
    finally:
        robot.disconnect()


def test_send_action_without_enable_raises_and_says_how(make_robot):
    """没有伺服环就不能静默收下动作 —— 必须说明是**没使能**、以及怎么改。"""
    robot, arm = make_robot(enable_on_connect=False)
    robot.connect()
    try:
        with pytest.raises(RuntimeError) as ei:
            robot.send_action({"action": [0.0] * 7})
        msg = str(ei.value)
        assert "enable" in msg.lower() or "使能" in msg, f"消息没说清原因：{msg}"
        time.sleep(0.15)
        assert not arm.calls_named("joint_follow"), "抛异常后仍发了帧"
    finally:
        robot.disconnect()


def test_disconnect_without_enable_issues_no_movej_and_closes(make_robot):
    """未使能时 `movej` 会被固件同一条 0x03 拒掉 ⇒ 收尾不许试，但链路要关。"""
    robot, arm = make_robot(enable_on_connect=False)
    robot.connect()
    robot.disconnect()
    assert arm.closed, "只读会话也必须把链路收干净"
    assert not arm.calls_named("movej"), "未使能时的 movej 必被拒，不该发"
    assert robot.is_connected is False


# ── 观测 ───────────────────────────────────────────────────────────────────

def test_get_observation_reads_the_envelope(make_robot):
    robot, arm = make_robot()
    arm.q = [0.11, 0.22, 0.33, 0.0, 0.0, 0.0, 0.0]
    robot.connect()
    try:
        obs = robot.get_observation()
        assert obs["observation.state"] == pytest.approx(arm.q)
        assert len(obs["observation.state"]) == 7
    finally:
        robot.disconnect()


# ── 动作 ───────────────────────────────────────────────────────────────────

def test_send_action_returns_the_clamped_value(make_robot):
    """LeRobot 契约：返回**实际发出的**动作。"""
    robot, arm = make_robot()
    robot.connect()
    try:
        # J4 上端 0.0175，给 5.0 必被钳
        target = [0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0]
        out = robot.send_action({"action": target})
        assert out["action"][3] < 0.0175
        assert out["action"][:3] == pytest.approx([0.0, 0.0, 0.0])
    finally:
        robot.disconnect()


def test_send_action_warns_on_saturation(make_robot, caplog):
    robot, _ = make_robot()
    robot.connect()
    try:
        with caplog.at_level("WARNING"):
            robot.send_action({"action": [0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0]})
        assert any("钳" in r.message or "J4" in r.message for r in caplog.records)
    finally:
        robot.disconnect()


def test_send_action_reaches_the_servo(make_robot):
    """断言对着 `send_action` 的**返回值**（即钳位后的实际目标）——
    对着原始输入断言会在 J4 上必挂：J4 上端只有 0.0175，任何大目标都被钳。"""
    robot, arm = make_robot()
    robot.connect()
    try:
        sent = robot.send_action({"action": [0.2] * 7})["action"]
        assert _wait(lambda: any(
            max(abs(a - b) for a, b in zip(c[1], sent)) < 1e-3
            for c in arm.calls_named("joint_follow")
        )), "目标没到伺服环"
    finally:
        robot.disconnect()


def test_send_action_validates_shape_and_finiteness(make_robot):
    robot, _ = make_robot()
    robot.connect()
    try:
        with pytest.raises(ValueError):
            robot.send_action({"action": [0.0] * 6})
        with pytest.raises(ValueError):
            robot.send_action({})
        with pytest.raises(ValueError):
            robot.send_action({"action": [float("nan")] * 7})
        with pytest.raises(ValueError):
            robot.send_action({"action": [float("inf")] * 7})
    finally:
        robot.disconnect()


def test_servo_death_makes_later_calls_raise(make_robot):
    """伺服线程死了以后，调用方必须**显式**拿到异常，不许静默回旧数据。"""
    robot, arm = make_robot()
    robot.connect()
    try:
        arm.raise_on["joint_follow"] = RuntimeError("链路坏")
        assert _wait(lambda: robot._servo.error is not None)
        with pytest.raises(RuntimeError, match="链路坏"):
            robot.send_action({"action": [0.0] * 7})
        with pytest.raises(RuntimeError, match="链路坏"):
            robot.get_observation()
    finally:
        robot.disconnect()


# ── 收尾 ───────────────────────────────────────────────────────────────────

def test_disconnect_holds_instead_of_disabling(make_robot):
    robot, arm = make_robot()
    robot.connect()
    robot.disconnect()
    assert arm.closed
    assert arm.calls_named("movej"), "收尾必须 movej 回当前位姿接管"
    assert not arm.calls_named("disable"), "默认绝不 disable（失能会自由落体）"
    assert robot.is_connected is False


def test_disconnect_disables_when_configured(make_robot):
    robot, arm = make_robot(disable_on_disconnect=True)
    robot.connect()
    robot.disconnect()
    assert arm.closed
    assert arm.calls_named("disable")
    assert robot.is_connected is False


def test_disconnect_is_idempotent(make_robot):
    robot, _ = make_robot()
    robot.connect()
    robot.disconnect()
    robot.disconnect()


def test_disconnect_does_not_hold_twice_after_the_servo_took_over(make_robot):
    """伺服环死时会**自己**接管（`movej` 回当前实测位姿）。

    收尾再接管一次 = 两条 `movej` 抢同一个 `(id, echo_cmd)` 应答队列、各自清掉
    对方的 ACK，而 `close()` 落在飞行中 —— 最坏是臂**仍在伺服态而链路已关**，
    0.1 s 看门狗 fail-soft。`servo.error` 非 None 就是"接管已经做过"的信号。
    """
    robot, arm = make_robot()
    robot.connect()
    arm.raise_on["joint_follow"] = RuntimeError("链路坏")
    assert _wait(lambda: robot._servo.error is not None), "伺服环没死"
    assert len(arm.calls_named("movej")) == 1, "伺服环自己应已接管一次"
    robot.disconnect()
    assert len(arm.calls_named("movej")) == 1, "收尾**不该**再接管一次"
    assert arm.closed
    assert robot.is_connected is False


def test_disconnect_does_not_race_a_wedged_servo_thread(make_robot):
    """线程还活着（没停下、`error` 也还没置）时收尾**同样**不许发第二条 `movej`。

    ⚠ 这一格 `servo.error is not None` 判据盖不住：`error` 是线程**做完接管之后**
    才置的，而那之前它可能正卡在自己的 `hold_at_current` 里。判据是"线程活着 ⇒
    两条 movej 会共用一条 `(id, echo_cmd)` 应答队列"。
    """
    robot, arm = make_robot(move_timeout=0.05)     # join 预算 = 0.05 + 2.0 s
    release = threading.Event()
    entered = threading.Event()

    def _blocked(*args, **kwargs):
        entered.set()
        release.wait(5.0)                          # 装成"卡在一次阻塞下发里"

    arm.joint_follow = _blocked                    # 实例属性盖住类方法
    robot.connect()
    try:
        assert entered.wait(2.0), "伺服线程没进到那次下发里"
        robot.disconnect()
        assert not arm.calls_named("movej"), "线程还活着却发了第二条 movej"
        assert arm.closed
        assert robot.is_connected is False
    finally:
        release.set()


def test_disconnect_survives_a_failing_hold(make_robot):
    """接管失败也必须把链路关掉 —— 否则串口句柄泄漏。"""
    robot, arm = make_robot()
    robot.connect()
    arm.raise_on["movej"] = RuntimeError("接管失败")
    robot.disconnect()
    assert arm.closed
    assert robot.is_connected is False


def test_context_manager_connects_and_disconnects(make_robot):
    robot, arm = make_robot()
    with robot:
        assert robot.is_connected
    assert arm.closed
