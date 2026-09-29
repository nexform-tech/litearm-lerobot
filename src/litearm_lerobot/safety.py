"""安全层 —— 纯逻辑：软限位钳位 + 跟随平滑（`slew_target`）。

本模块**不 import litearm**：`read_safe_limits` 只吃一个 duck-typed `arm`，
所以纯函数部分能在无臂机器上单测。

## ⚠⚠ 这是安全关键代码的第三份副本

`slew_target` / `clamp_to_limits` / `read_safe_limits` 逐字移植自：

    pylitearm/src/pylitearm/control/joint_follow.py:45-100   （原版，唯一权威）
    litearm-teleop-isomorphic/liteteleop/safety.py           （第二份）
    litearm-lerobot/src/litearm_lerobot/safety.py            （本文件）

**任一份改动，三份必须对拍。** 对拍判据在 `tests/test_safety.py` 的
`test_slew_matches_pylitearm_reference`（Task 2 加）—— 它拿**原版**当参照，
不拿第二份当参照，这样两边都对着同一个第三方。

不共享代码的理由：`litearm-teleop-isomorphic` 拖 PyQt5，而本仓是个库。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

__all__ = [
    "LimitsError", "NonFiniteTarget", "Limits",
    "read_limits_ok", "clamp_to_limits", "read_safe_limits", "slew_target",
    "DEFAULT_LIMIT_MARGIN",
]

#: 软限位内缩量（rad）。照抄 litearm-server 的 0.01。
#:
#: ## ⛔ 上界约束：不得压没任何一根轴的行程 —— **J4 的固件上端只有 +0.017547 rad**
#:
#: `litearm-stm32` `params/joint_limit_macros.h:16` 的 `JL_QMAX_J4 = 0.017547f`。
#: 内缩后 J4 的上界是 `0.017547 − margin`：
#:
#:     margin = 0.01  -> +0.0075 rad，仍可用
#:     margin = 0.02  -> -0.0025 rad  ⇒ J4 整个正半轴消失
#:
#: ⚠ 而且这个错误**不会报错**：`read_limits_ok` 只检查 `lo < hi`，两者仍成立。
#: 判据 = `test_limit_margin_keeps_j4_usable`。
#:
#: ## 下界方向别搞反
#:
#: 固件 `safety_check.c` 的位置锁存条件是（已使能态）`q_meas > q_max + 0.05`。而目标
#: 已被本模块钳到 `q_max − margin`，实测最多再冲过
#: `overshoot ≈ vel_max × 2 × RTT ≈ 2.0 × 2 × 0.003333 ≈ 0.013 rad`：
#:
#:     q_meas ≤ q_max − margin + overshoot
#:
#: ⇒ **不锁存的条件是 `overshoot ≤ 0.05 + margin`**，即 **`margin ≥ overshoot − 0.05`**
#: ⇒ 任何 `margin ≥ 0` 都不会锁存。⛔ 不是 `margin ≥ 0.05 + overshoot`。
#: 取 0.01 只是浮点/标定余量。
DEFAULT_LIMIT_MARGIN = 0.01


class LimitsError(ValueError):
    """限位配置不合法 —— **一律拒启动**，绝不静默退化。"""


class NonFiniteTarget(LimitsError):
    """本拍目标值里出现 NaN/Inf ⇒ 本拍不许下发。"""


@dataclass(frozen=True)
class Limits:
    """一组关节软限位（rad）。`lo`/`hi` 逐轴成对，长度 = 轴数。"""

    lo: Tuple[float, ...]
    hi: Tuple[float, ...]
    n: Optional[int] = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "lo", tuple(float(v) for v in self.lo))
        object.__setattr__(self, "hi", tuple(float(v) for v in self.hi))
        if self.n is None:
            object.__setattr__(self, "n", len(self.lo))
        if len(self.lo) != len(self.hi) or len(self.lo) != self.n:
            raise LimitsError(
                f"长度不符: lo={len(self.lo)} hi={len(self.hi)} n={self.n}"
            )
        for i, (a, b) in enumerate(zip(self.lo, self.hi)):
            if not a < b:
                raise LimitsError(f"第 {i} 轴软限位不合法（零宽度或反了）: lo={a} hi={b}")


def _check_finite(vals: Sequence[float], what: str) -> None:
    for i, v in enumerate(vals):
        if not math.isfinite(v):
            raise LimitsError(f"非有限值: {what}[{i}] = {v}")


def read_limits_ok(lo: Sequence[float], hi: Sequence[float], n: int) -> Limits:
    """校验并固化一对软限位；任何一处不合法都拒启动。"""
    if n <= 0:
        raise LimitsError(f"轴数非法: n={n}")
    if len(lo) != n or len(hi) != n:
        raise LimitsError(f"长度不符: lo={len(lo)} hi={len(hi)} n={n}")
    _check_finite(lo, "lo")
    _check_finite(hi, "hi")
    for i, (a, b) in enumerate(zip(lo, hi)):
        if not a < b:
            raise LimitsError(f"第 {i} 轴上下界反了: lo={a} hi={b}")
    return Limits(lo=tuple(lo), hi=tuple(hi), n=n)


def clamp_to_limits(q: Sequence[float], limits: Limits) -> Tuple[List[float], List[bool]]:
    """把 `q` 逐轴钳进软限位，返回 `(q_clamped, saturated)`。

    `saturated` 显式返回「哪些轴被钳了」，供调用方告警 —— 静默钳位会让
    "策略输出的目标超限"这件事消失得无声无息。
    """
    if len(q) != limits.n:
        raise LimitsError(f"长度不符: q={len(q)} 限位轴数={limits.n}")
    for i, v in enumerate(q):
        if not math.isfinite(v):
            raise NonFiniteTarget(f"q[{i}] 非有限值: {v}")
    out: List[float] = []
    sat: List[bool] = []
    for v, a, b in zip(q, limits.lo, limits.hi):
        if v < a:
            out.append(a)
            sat.append(True)
        elif v > b:
            out.append(b)
            sat.append(True)
        else:
            out.append(float(v))
            sat.append(False)
    return out, sat


def read_safe_limits(arm, margin: float = DEFAULT_LIMIT_MARGIN) -> Limits:
    """读**固件里的**软限位并内缩 `margin`。

    对应 litearm-server 的 `teleop_manager._read_safe_limits()`。本 SDK 没有
    `arm.kin`，改读 `arm.params.all_joint_params()` 的 `q_min`/`q_max` ——
    那是**固件真正用来钳 `movej`/`move_js` 的那一对值**，比配置文件更权威。

    ⚠ 拿不到就**抛**，绝不退回 ±9 那种兜底哨兵：限位不可信 ⇒ 不启动。
    """
    jp = arm.params.all_joint_params()
    if not jp:
        raise LimitsError("读不到关节参数（all_joint_params 为空）")
    lo = [float(p.q_min) + margin for p in jp]
    hi = [float(p.q_max) - margin for p in jp]
    return read_limits_ok(lo, hi, len(jp))


def slew_target(raw_target, q_cmd, dq_cmd, speed_limit, accel_limit, dt):
    """速度/加速度限制的目标位置平滑（梯形速度曲线）。

    ⚠ **逐字移植自 `pylitearm/src/pylitearm/control/joint_follow.py:45-100`**
    （唯一偏离：原版循环上界是模块常量 `N = 7`，这里取 `len(raw_target)`；
    7 轴输入下两者**完全等价**）。对每个关节：

    - 限制最大速度为 `speed_limit[i]`
    - 限制最大加速度为 `accel_limit[i]`
    - 当接近目标时自动减速（基于制动距离 `v²/(2a)`）
    - 落在死区（`|diff| < 1e-5` 且 `|v| < dv_max`）时吸附到目标并停住

    ⚠ **移植已知边界（照抄，不修）**：`diff` 恰为 0 或落在 `±1e-5` 死区内、
    而 `|v| ≥ dv_max` 时，`math.copysign(v_limit, diff)` 在 `diff == 0` 会返回
    `+v_limit` ⇒ 该轴可能继续正向加速而非停住（原版同样如此）。实践中 `diff`
    极少恰为 0，且 `abs(diff) < 1e-5 and abs(v) < dv_max` 已挡掉绝大多数情形。
    """
    dt = max(dt, 1e-4)
    for i in range(len(raw_target)):
        v_limit = max(1e-4, speed_limit[i])
        a_limit = max(1e-4, accel_limit[i])
        dv_max = a_limit * dt

        diff = raw_target[i] - q_cmd[i]
        v = dq_cmd[i]

        # 已到达目标
        if abs(diff) < 1e-5 and abs(v) < dv_max:
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
            continue

        # 期望速度（考虑制动距离）
        stopping_dist = (v * v) / (2.0 * a_limit) if a_limit > 0.0 else 0.0
        moving_toward = diff * v > 0.0
        if moving_toward and abs(diff) <= stopping_dist:
            desired_v = 0.0  # 开始减速
        else:
            desired_v = math.copysign(v_limit, diff)

        # 限加速度
        v += max(-dv_max, min(dv_max, desired_v - v))
        v = max(-v_limit, min(v_limit, v))

        # 更新位置
        step = v * dt
        if diff * step > 0.0 and abs(step) >= abs(diff):
            q_cmd[i] = raw_target[i]
            dq_cmd[i] = 0.0
        else:
            q_cmd[i] += step
            dq_cmd[i] = v

    return q_cmd, dq_cmd
