"""对真 `litearm` SDK 的契约判据。

⚠⚠ 这个文件的**全部**期望值都取自真 SDK（`inspect.signature` / 真实构造），
**不从本仓实现取** —— 旧测试之所以 14 条全绿而真机一跑就响，就是因为判据与
实现同源。这里不许出现"本仓定义的关键字列表"再拿来比对。
"""
from __future__ import annotations

import inspect

import pytest

import litearm


# ── 构造函数：真刀真枪构造一次 ─────────────────────────────────────────────
#
# `Arm(port=..., move_timeout=...)` 只存字段、不碰串口（`connect()` 才碰），
# 所以这个断言**零风险**且直接判别"我们传的关键字真 SDK 收不收"。

def test_real_arm_accepts_the_kwargs_we_pass():
    arm = litearm.Arm(port="/dev/definitely-not-a-port", move_timeout=15.0)
    assert arm is not None
    arm.close()          # 幂等空操作，没连接时不该抛


def test_real_arm_rejects_the_old_server_kwargs():
    """墓碑：这三个是旧 server 语义的关键字，本仓正是被它们打死的。"""
    for kw in ("endpoint", "arm_id", "query_timeout"):
        with pytest.raises(TypeError):
            litearm.Arm(**{kw: "x"})


def test_real_arm_has_no_hold():
    """旧 `robot.py` 在收尾调 `arm.hold()` —— 真 SDK 从来没有这个方法。"""
    assert not hasattr(litearm.Arm, "hold")


# ── 方法签名：本仓调用的每个关键字都必须在真签名里 ─────────────────────────
#
# ⚠ 这里的列表是「本仓**写死要调**的关键字」，不是"从本仓源码 grep 出来的"。
#   它是显式的契约声明：改了调用点就要改这里，改不了说明你在偷偷改契约。

_METHOD_KWARGS = {
    "connect": ["port"],
    "get_state": ["refresh", "timeout"],
    "enable": ["attempts"],
    "movej": ["q", "speed"],
    "joint_follow": ["q", "dq", "kp", "kd"],
    "move_js": ["q", "dq"],
    "close": [],
}


@pytest.mark.parametrize("meth,kwargs", sorted(_METHOD_KWARGS.items()))
def test_real_methods_carry_the_kwargs_we_pass(meth, kwargs):
    fn = getattr(litearm.Arm, meth)
    params = inspect.signature(fn).parameters
    for kw in kwargs:
        assert kw in params, f"真 SDK 的 Arm.{meth} 没有参数 {kw}"


def test_real_movej_has_no_settle_s():
    """旧代码传 `settle_s=` —— 新 SDK 的 `movej` 自身阻塞到位，没有这个参数。"""
    assert "settle_s" not in inspect.signature(litearm.Arm.movej).parameters


def test_real_joint_follow_is_positional_q_dq_kp_kd():
    """`0x08` 的四个量都是位置参数，本仓按位置传。"""
    kinds = [
        p.kind for p in inspect.signature(litearm.Arm.joint_follow).parameters.values()
        if p.name != "self"
    ]
    assert kinds[:4] == [inspect.Parameter.POSITIONAL_OR_KEYWORD] * 4


# ── 返回信封与状态对象 ─────────────────────────────────────────────────────

def test_msg_envelope_has_value_hz_timestamp():
    import dataclasses

    fields = {f.name for f in dataclasses.fields(litearm.Msg)}
    assert {"value", "hz", "timestamp"} <= fields


def test_robot_state_exposes_what_we_read():
    from litearm.state import RobotState

    for attr in ("q", "dq", "joints", "flags", "flag_names", "joint_fault",
                 "faulted"):
        assert hasattr(RobotState, attr) or attr in getattr(
            RobotState, "__dataclass_fields__", {}), f"RobotState 缺 {attr}"


def test_joint_param_exposes_limits():
    from litearm.params import JointParam

    for attr in ("q_min", "q_max"):
        assert attr in getattr(JointParam, "__dataclass_fields__", {})


def test_find_cdc_port_is_exported():
    """`port=None` 时我们要靠它自动找口。"""
    assert callable(litearm.find_cdc_port)


def test_needed_exceptions_are_exported():
    for name in ("UnsupportedByFirmwareError", "CommandRejectedError",
                 "LiteArmError", "TransportError"):
        assert hasattr(litearm, name), f"litearm 没导出 {name}"
