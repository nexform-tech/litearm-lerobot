"""共享 fixture：一个**与真 SDK 同形**的假臂（无串口、无硬件）。

⚠⚠ FakeArm 的每个形状都必须对着真 SDK 写，不许照着自己方便写 ——
`tests/test_fake_arm_contract.py` 就是为这条守的。旧版 FakeArm 有 `hold()`、
收 `settle_s`、`get_state()` 回 dict，于是 14 条测试全绿而真机一跑就响。
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import List, Optional

import pytest

import litearm
from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig


@dataclass
class FakeJointState:
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    t_mos: float = 0.0
    t_coil: float = 0.0
    err: int = 0


@dataclass
class FakeRobotState:
    """`litearm.state.RobotState` 里本仓读到的那些字段。"""

    mode: int = 0
    mode_name: str = "INIT"
    flags: int = 0
    flag_names: List[str] = field(default_factory=list)
    seq: int = 0
    joints: List[FakeJointState] = field(default_factory=list)
    joint_fault: int = 0

    @property
    def q(self) -> List[float]:
        return [j.q for j in self.joints]

    @property
    def dq(self) -> List[float]:
        return [j.dq for j in self.joints]

    @property
    def faulted(self) -> bool:
        return bool(self.flags & 1) or self.mode == 6 or self.joint_fault != 0


@dataclass
class FakeMsg:
    """`litearm.Msg`：11 个"读一帧"型 getter 的返回信封。"""

    value: object
    hz: float = 100.0
    timestamp: float = 0.0


@dataclass
class FakeJointParam:
    q_min: float
    q_max: float


class FakeArm:
    """`litearm.Arm` 的 stand-in。**形状对齐真 SDK**（见模块 docstring）。"""

    #: 固件 7J 出厂软限位 —— 抄自 `litearm-stm32` `params/joint_limit_macros.h`。
    #: ⚠ J4 的上端只有 0.017547（1°），是本表最窄的一根。
    LIMITS = [
        (-2.809547, 2.809547),
        (-1.727547, 1.727547),
        (-2.809547, 2.809547),
        (-3.071547, 0.017547),
        (-2.809547, 2.809547),
        (-1.553547, 1.553547),
        (-1.553547, 1.553547),
    ]

    def __init__(self, port: Optional[str] = None, **kwargs):
        self.port = port
        self.ctor_kwargs = kwargs
        self.calls: list = []
        self.n = 7
        self.firmware = "Litearm1.5.0-7J"
        self.q = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0]
        self.closed = False
        self.connected = False
        self.enabled = False
        self.raise_on: dict = {}          # 方法名 -> 异常实例
        self.params = _FakeParams(self)

    # ── 会话 ────────────────────────────────────────────────────────────
    def connect(self, port: Optional[str] = None) -> "FakeArm":
        self.calls.append(("connect", port))
        self._maybe_raise("connect")
        self.connected = True
        return self

    def close(self) -> None:
        self.calls.append("close")
        self.closed = True
        self.connected = False

    # ── 状态 ────────────────────────────────────────────────────────────
    def get_state(self, refresh: bool = False, timeout: float = 0.5) -> FakeMsg:
        self.calls.append(("get_state", refresh))
        self._maybe_raise("get_state")
        st = FakeRobotState(joints=[
            FakeJointState(q=v, dq=0.0) for v in self.q
        ])
        return FakeMsg(value=st)

    # ── 使能 ────────────────────────────────────────────────────────────
    def enable(self, attempts: int = 12) -> None:
        self.calls.append(("enable", attempts))
        self._maybe_raise("enable")
        self.enabled = True

    def disable(self) -> None:
        self.calls.append("disable")
        self._maybe_raise("disable")
        self.enabled = False

    # ── 运动 ────────────────────────────────────────────────────────────
    def movej(self, q, speed: float = 1.0):
        q = [float(v) for v in q]
        self.calls.append(("movej", q, speed))
        self._maybe_raise("movej")
        self.q = list(q)
        return self.get_state().value

    def joint_follow(self, q, dq, kp, kd) -> None:
        self.calls.append((
            "joint_follow",
            [float(v) for v in q], [float(v) for v in dq],
            [float(v) for v in kp], [float(v) for v in kd],
        ))
        self._maybe_raise("joint_follow")
        self.q = [float(v) for v in q]

    def move_js(self, q, dq=None, tau_ff=None) -> None:
        self.calls.append((
            "move_js", [float(v) for v in q],
            None if dq is None else [float(v) for v in dq],
        ))
        self._maybe_raise("move_js")
        self.q = [float(v) for v in q]

    # ── 内部 ────────────────────────────────────────────────────────────
    def _maybe_raise(self, name: str) -> None:
        exc = self.raise_on.get(name)
        if exc is not None:
            raise exc

    def calls_named(self, name: str) -> list:
        return [c for c in self.calls
                if (c == name) or (isinstance(c, tuple) and c[0] == name)]


class _FakeParams:
    def __init__(self, arm: FakeArm):
        self._arm = arm

    def get_joint_param(self, idx: int, timeout: float = 1.0) -> FakeMsg:
        lo, hi = FakeArm.LIMITS[idx]
        return FakeMsg(value=FakeJointParam(q_min=lo, q_max=hi))

    def all_joint_params(self) -> list:
        return [self.get_joint_param(i).value for i in range(self._arm.n)]


class _NoParams:
    """`all_joint_params()` 读不到的情形。"""

    def all_joint_params(self):
        return []


@pytest.fixture
def fake_arm():
    return FakeArm()


@pytest.fixture
def make_robot(monkeypatch):
    """造一个 `litearm.Arm` 已被 FakeArm 顶掉的 LiteArmRobot。

    ⚠ 工厂**必须把构造关键字记到臂上** —— 否则 `Arm(port=..., move_timeout=...)`
    传进去的东西在假臂上无迹可寻，测试只能断言"connect 被调了"，断言不了"连的是
    哪个口、超时是多少"。
    """

    def _make(**cfg):
        arm = FakeArm()

        def _factory(*args, **kwargs):
            arm.ctor_kwargs = dict(kwargs)
            arm.port = kwargs.get("port")
            return arm

        monkeypatch.setattr(litearm, "Arm", _factory)
        robot = LiteArmRobot(LiteArmRobotConfig(**cfg))
        return robot, arm

    return _make
