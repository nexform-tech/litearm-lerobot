"""安全层纯函数测试。无臂、无线程、无 I/O。"""
from __future__ import annotations

import math

import pytest

from litearm_lerobot.safety import (
    DEFAULT_LIMIT_MARGIN,
    Limits,
    LimitsError,
    NonFiniteTarget,
    clamp_to_limits,
    read_limits_ok,
    read_safe_limits,
    slew_target,
)


class _JP:
    """Duck-typed `litearm.params.get_joint_param().value`。"""

    def __init__(self, q_min, q_max):
        self.q_min = q_min
        self.q_max = q_max


class _Params:
    def __init__(self, jps):
        self._jps = jps

    def all_joint_params(self):
        return self._jps


class _Arm:
    def __init__(self, jps):
        self.params = _Params(jps)


#: 固件 7J 出厂软限位 —— 从 `litearm-stm32` `params/joint_limit_macros.h` 抄来。
#: ⚠ J4 的上端只有 +0.017547 rad（1°），是本文件里最窄的一根行程。
FW_LIMITS = [
    (-2.809547, 2.809547),   # J1
    (-1.727547, 1.727547),   # J2
    (-2.809547, 2.809547),   # J3
    (-3.071547, 0.017547),   # J4  <- 上端 1°，本表最窄
    (-2.809547, 2.809547),   # J5
    (-1.553547, 1.553547),   # J6
    (-1.553547, 1.553547),   # J7
]


def _arm():
    return _Arm([_JP(lo, hi) for lo, hi in FW_LIMITS])


# ── Limits / read_limits_ok ────────────────────────────────────────────────

def test_limits_rejects_inverted_bounds():
    with pytest.raises(LimitsError):
        Limits(lo=(1.0,), hi=(0.0,), n=1)


def test_limits_rejects_zero_width():
    with pytest.raises(LimitsError):
        Limits(lo=(0.5,), hi=(0.5,), n=1)


def test_read_limits_ok_rejects_nan():
    with pytest.raises(LimitsError):
        read_limits_ok([math.nan], [1.0], 1)


def test_read_limits_ok_rejects_zero_joint_count():
    with pytest.raises(LimitsError):
        read_limits_ok([], [], 0)


# ── read_safe_limits ───────────────────────────────────────────────────────

def test_read_safe_limits_insets_by_margin():
    lim = read_safe_limits(_arm(), margin=0.01)
    assert lim.n == 7
    assert lim.lo[0] == pytest.approx(FW_LIMITS[0][0] + 0.01)
    assert lim.hi[0] == pytest.approx(FW_LIMITS[0][1] - 0.01)


def test_read_safe_limits_raises_when_params_unreadable():
    with pytest.raises(LimitsError):
        read_safe_limits(_Arm([]))


def test_limit_margin_keeps_j4_usable():
    """⚠ 这条是 J4 陷阱的具名载体。

    J4 的固件上端只有 +0.017547 rad。内缩 margin 后上界是 `0.017547 - margin`：
    margin=0.01 -> +0.0075（可用）；margin=0.02 -> -0.0025（**正半轴消失**）。
    `read_limits_ok` 只检查 lo < hi，所以后者**静默通过** —— 必须在这里挡住。
    """
    lim = read_safe_limits(_arm(), margin=DEFAULT_LIMIT_MARGIN)
    assert DEFAULT_LIMIT_MARGIN == 0.01, "改这个值先读本用例的 docstring"
    # J4 仍有正行程，且总行程保住了 99% 以上
    assert lim.hi[3] > 0.0
    fw_travel = FW_LIMITS[3][1] - FW_LIMITS[3][0]
    assert (lim.hi[3] - lim.lo[3]) > 0.99 * fw_travel


def test_limit_margin_of_002_would_destroy_j4():
    """反面对照：证明上一条判据**有判别力**（不是恒真）。"""
    lim = read_safe_limits(_arm(), margin=0.02)
    assert lim.hi[3] < 0.0        # 上界为负 = 正半轴没了
    assert lim.hi[3] > lim.lo[3]  # 而且不报错


# ── clamp_to_limits ────────────────────────────────────────────────────────

def test_clamp_saturates_and_reports_axes():
    lim = Limits(lo=(-1.0, -1.0), hi=(1.0, 1.0), n=2)
    out, sat = clamp_to_limits([2.0, 0.5], lim)
    assert out == [1.0, 0.5]
    assert sat == [True, False]


def test_clamp_rejects_nan():
    lim = Limits(lo=(-1.0,), hi=(1.0,), n=1)
    with pytest.raises(NonFiniteTarget):
        clamp_to_limits([math.nan], lim)


def test_clamp_rejects_wrong_length():
    lim = Limits(lo=(-1.0,), hi=(1.0,), n=1)
    with pytest.raises(LimitsError):
        clamp_to_limits([0.0, 0.0], lim)


# ── slew_target ────────────────────────────────────────────────────────────

def test_slew_is_rate_limited():
    sp, ac, dt = [1.0], [10.0], 0.01
    q_cmd, dq_cmd = [0.0], [0.0]
    q_cmd, dq_cmd = slew_target([1.0], q_cmd, dq_cmd, sp, ac, dt)
    # dv_max = accel_limit * dt = 10.0 * 0.01 = 0.1 ⇒ 本拍速度只到 0.1，
    # 位置只推进 0.1 * 0.01 = 0.001。⚠ 别把这拍当成"一步到 v_limit"。
    assert q_cmd[0] == pytest.approx(0.001, abs=1e-12)
    assert dq_cmd[0] == pytest.approx(0.1, abs=1e-12)


def test_slew_never_exceeds_speed_limit():
    sp, ac, dt = [0.7], [5.0], 0.01
    q_cmd, dq_cmd = [0.0], [0.0]
    for _ in range(500):
        q_cmd, dq_cmd = slew_target([5.0], q_cmd, dq_cmd, sp, ac, dt)
        assert abs(dq_cmd[0]) <= 0.7 + 1e-12


def test_slew_converges_and_snaps():
    sp, ac, dt = [1.0], [10.0], 0.01
    q_cmd, dq_cmd = [0.0], [0.0]
    for _ in range(2000):
        q_cmd, dq_cmd = slew_target([0.3], q_cmd, dq_cmd, sp, ac, dt)
    assert q_cmd[0] == pytest.approx(0.3, abs=1e-12)
    assert dq_cmd[0] == 0.0


def test_slew_decelerates_before_target():
    """制动距离：快到目标时必须先减速，而不是冲过去再回拉。"""
    sp, ac, dt = [2.0], [4.0], 0.01
    q_cmd, dq_cmd = [0.0], [0.0]
    for _ in range(1000):
        q_cmd, dq_cmd = slew_target([0.3], q_cmd, dq_cmd, sp, ac, dt)
        assert q_cmd[0] <= 0.3 + 1e-12, "冲过了目标"
    assert q_cmd[0] == pytest.approx(0.3, abs=1e-12)
