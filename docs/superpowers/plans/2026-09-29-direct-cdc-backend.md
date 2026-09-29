# litearm-lerobot 直连后端实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `litearm-lerobot` 从「litearm-server / zenoh 远程客户端」换到
「litearm-python STM32 直连 + 固件内 `0x08` 伺服环」，使之能在真机上工作，
并满足 `nexform-tech/repo-template` 的仓库规范。

**Architecture:** `LiteArmRobot`（LeRobot `Robot` 适配器）持有一个 `ServoLoop` 后台线程；
线程以 `servo_hz` 节拍跑 `slew_target` 生成限速参考，经唯一的 `_send()` 下发点到
`arm.joint_follow(q, dq, K, B)`。限位、重力前馈、伺服环三者都在固件里，PC 只做
钳位、限速与目标更新。`safety.py` 是纯函数层（无线程无 I/O），可与上游逐拍对拍。

**Tech Stack:** Python 3.10+、`litearm-python` 2.1.0（直连 USB CDC，只有 `pyserial` 一个依赖）、`lerobot>=0.3.0,<0.5`、pytest。

**Spec:** `docs/superpowers/specs/2026-09-29-direct-cdc-backend-design.md`

**Branch:** `feat/port-to-litearm-python-direct-cdc`（已存在，已含 2 个 docs 提交）。**本计划全程在此分支提交，不 push。**

---

## 实现前必读（零上下文的工程师看这里）

1. **`import litearm` 现在落在 `/home/llx/litearm-python/src/litearm/`**，是 **STM32 直连** SDK（2.1.0）。旧代码写的 `Arm(endpoint=..., arm_id=...)` / `arm.hold()` / `movej(..., settle_s=...)` 全部不存在 —— 这正是本计划要修的。
2. **官方解释器是 `/usr/bin/python3`**（3.10）。`which python3` 指向 conda 且没装 `lerobot`。跑测试一律用 `/usr/bin/python3 -m pytest`。
3. **不要改 `litearm-python` 与 `litearm-stm32`**。它们是参照物，不是交付物。本计划只改 `litearm-lerobot`。
4. **`safety.py` 里的 `slew_target` 是逐字移植**，不是"实现一个类似的"。对拍判据在 Task 2。
5. **`limit_margin = 0.01` 不能调大**：J4 的固件上端只有 `+0.017547 rad`，取 0.02 就把 J4 正半轴抹掉，而且**不报错**（见 Task 1 的判据）。
6. 提交信息一律 Conventional Commits、**英文**、72 字符以内 subject、**绝不出现 `Co-Authored-By` 或任何工具署名**。

**这份计划的代码已经真跑过。** 把本文档里的代码块逐字抽到 `/tmp` 下、对着真
`litearm` 2.1.0 与 `lerobot` 0.4.4 执行：**全绿，86 条**（逐文件 5+5+20+21+17+15+3）。
抽取方式与两个已知盲区见 Task 12 Step 1。

⚠ 这条断言曾经写成 **87**，那是错的：抽取脚本有盲区，把 Task 8 的两条速度判据
**同时**写进了 `test_safety.py` 与 `test_utils.py`（后者本该是
`test_register_returns_litearm_robot`）⇒ 重复计数多出一条。**以仓里
`pytest --collect-only` 的实际条数为准**，不是以抽取树为准。

---

## 文件结构

| 文件 | 动作 | 职责 |
|---|---|---|
| `src/litearm_lerobot/safety.py` | 新建 | 纯函数：`Limits` / `read_limits_ok` / `clamp_to_limits` / `read_safe_limits` / `slew_target`。无 `litearm` import、无线程 |
| `src/litearm_lerobot/servo.py` | 新建 | `ServoLoop`（后台线程）+ `hold_at_current()` + `_send()` 单点分派 |
| `src/litearm_lerobot/robot.py` | 重写 | LeRobot 适配器 |
| `src/litearm_lerobot/config.py` | 重写 | `LiteArmRobotConfig` |
| `src/litearm_lerobot/__init__.py` | 改 | 文档字符串与 `__version__` |
| `src/litearm_lerobot/utils.py` | 不动 | `register()` |
| `tests/conftest.py` | 重写 | `FakeArm` 按**真 SDK 形状**重造（拿掉 `hold` / `settle_s` / dict 状态） |
| `tests/test_safety.py` | 新建 | 纯函数 + 逐拍对拍 + J4 行程判据 |
| `tests/test_servo.py` | 新建 | 伺服环：收敛、`dq` 非零、节拍耗时、异常路径 |
| `tests/test_sdk_contract.py` | 新建 | **对真 SDK 取签名**（本次事故的补丁） |
| `tests/test_robot.py` | 重写 | 适配器生命周期 |
| `tests/test_utils.py` | 改 | 去掉旧配置字段 |
| `examples/01_read_observation.py` | 改 | `--endpoint/--arm-id` → `--port` |
| `examples/02_send_action.py` | 改 | 从**当前实测位姿**偏移，不是把全轴设成同一绝对值 |
| `examples/03_record_dataset.py` | 改 | 参数同上 |
| `examples/README.md` / `.zh-CN.md` | 改 | 参数与前置条件 |
| `README.md` / `README.zh-CN.md` | 改 | 架构图与前置条件 |
| `docs/DEVELOPER_GUIDE.md` / `.zh-CN.md` | 改 | 配置表、执行器、失败方向 |
| `pyproject.toml` | 改 | version 占位符、依赖下限 |
| `.github/workflows/ci.yml` | 查 | 确认 `litearm-python` 的 git 安装行仍然对 |

---

## Task 1: `safety.py` —— 纯函数层

**Files:**

- Create: `src/litearm_lerobot/safety.py`
- Test: `tests/test_safety.py`

- [ ] **Step 1: 写失败测试**

创建 `tests/test_safety.py`：

```python
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
```

- [ ] **Step 2: 跑测试确认失败**

Run: `/usr/bin/python3 -m pytest tests/test_safety.py -q`
Expected: FAIL —— `ModuleNotFoundError: No module named 'litearm_lerobot.safety'`

- [ ] **Step 3: 写实现**

创建 `src/litearm_lerobot/safety.py`：

```python
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
```

- [ ] **Step 4: 跑测试确认通过**

Run: `/usr/bin/python3 -m pytest tests/test_safety.py -q`
Expected: PASS（15 条）

- [ ] **Step 5: 提交**

```bash
git add src/litearm_lerobot/safety.py tests/test_safety.py
git commit -m "feat: add the pure safety layer ported from pylitearm"
```

---

## Task 2: `slew_target` 与 pylitearm 原版逐拍对拍

**Files:**

- Test: `tests/test_safety.py`（追加）

**为什么单独一个 Task：** Task 1 验的是"行为合理"，这条验的是"**与既有真机验证过的实现逐位相同**"。两者不是一回事 —— 一个自己写对了的斜坡，其制动距离仍可能和原版差一拍。

- [ ] **Step 1: 写失败测试**

在 `tests/test_safety.py` 末尾追加：

```python
# ── 与 pylitearm 原版逐拍对拍 ──────────────────────────────────────────────
#
# ⚠ 参照物必须是**原版**（`pylitearm`），不是 `litearm-teleop-isomorphic`
#   —— 后者与本仓是同一个算法的两份副本，拿它当参照等于自己对自己。

PYLITEARM_SLEW = "/home/llx/pylitearm/src/pylitearm/control/joint_follow.py"


def _load_reference_slew():
    """从 pylitearm 原版源码里把 `slew_target` 抠出来编译。

    不 import pylitearm（它会拖一整套 SDK）。用 `ast` 取那个函数节点单独编译，
    这样参照物是**原版的源码本身**，不是我抄的诗。
    """
    import ast

    with open(PYLITEARM_SLEW) as f:
        tree = ast.parse(f.read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "slew_target":
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {"math": math, "N": 7, "clamp": lambda v, lo, hi: max(lo, min(hi, v))}
            exec(compile(mod, PYLITEARM_SLEW, "exec"), ns)
            return ns["slew_target"]
    raise AssertionError(f"在 {PYLITEARM_SLEW} 里找不到 slew_target")


@pytest.mark.parametrize(
    "targets",
    [
        [0.3] * 7,                       # 阶跃
        [0.0, 0.5, -0.4, 0.2, 0.0, 0.0, 0.0],
        [-1.2, 0.9, 0.0, -0.7, 0.3, -0.1, 0.0],
    ],
)
def test_slew_matches_pylitearm_reference(targets):
    ref = _load_reference_slew()
    sp = [2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0]
    ac = [8.0, 8.0, 7.0, 7.0, 9.0, 9.0, 9.0]
    dt = 1.0 / 250.0

    q_mine, dq_mine = [0.0] * 7, [0.0] * 7
    q_ref, dq_ref = [0.0] * 7, [0.0] * 7
    for tick in range(600):
        q_mine, dq_mine = slew_target(list(targets), q_mine, dq_mine, sp, ac, dt)
        q_ref, dq_ref = ref(list(targets), q_ref, dq_ref, sp, ac, dt)
        assert q_mine == pytest.approx(q_ref, abs=1e-12), f"第 {tick} 拍位置漂了"
        assert dq_mine == pytest.approx(dq_ref, abs=1e-12), f"第 {tick} 拍速度漂了"


def test_reference_slew_is_actually_loaded():
    """判别力：证明上一条不是"两个空函数互相对拍"。

    ⚠ 原版的循环上界是模块常量 `N = 7`（`pylitearm/hal/hardware.py`），
    传 1 元素列表会 `IndexError` —— 所以这里必须给 7 个。
    """
    ref = _load_reference_slew()
    q, dq = [0.0] * 7, [0.0] * 7
    q, dq = ref([1.0] * 7, q, dq, [1.0] * 7, [10.0] * 7, 0.01)
    assert q[0] > 0.0
    assert dq[0] == pytest.approx(0.1)     # 同上：本拍只到 dv_max = 10 * 0.01
```

- [ ] **Step 2: 跑测试确认失败**

Run: `/usr/bin/python3 -m pytest tests/test_safety.py -k "pylitearm or actually_loaded" -q`
（⚠ 只写 `-k pylitearm` **选不中** `test_reference_slew_is_actually_loaded` ——
名字里没有那个词。实测：`-k pylitearm` 只见 3 条，漏掉第 4 条。）
Expected: 应能通过。**若 FAIL** —— 说明我的移植与原版有差异，**修 `safety.py` 去对齐原版**，不要改这条测试。若报 `FileNotFoundError`，说明本机没有 `pylitearm` 仓，此时**把这条测试标成 `@pytest.mark.skipif(not os.path.exists(...))`** 并在提交信息里写明"对拍判据在本机跳过"。

- [ ] **Step 3: 提交**

```bash
git add tests/test_safety.py
git commit -m "test: pin slew_target against the pylitearm original tick by tick"
```

---

## Task 3: `FakeArm` 按真 SDK 形状重造

**Files:**

- Rewrite: `tests/conftest.py`

**为什么先做这个：** 现在 14 条测试全绿的根因就是 `FakeArm` 照着旧 API 造的。先把它改成真形状，后面 Task 4/5 的测试才可能是真的。

- [ ] **Step 1: 写失败测试**

创建 `tests/test_fake_arm_contract.py`：

```python
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
```

- [ ] **Step 2: 跑测试确认失败**

Run: `/usr/bin/python3 -m pytest tests/test_fake_arm_contract.py -q`
Expected: FAIL（`ImportError` 或断言失败 —— 现 `FakeArm` 有 `hold`、收 `settle_s`、`get_state` 回 dict）

- [ ] **Step 3: 写实现**

把 `tests/conftest.py` 整个替换为：

```python
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
```

- [ ] **Step 4: 跑测试确认通过**

Run: `/usr/bin/python3 -m pytest tests/test_fake_arm_contract.py -q`
Expected: PASS（5 条）

> ⚠ 此刻 `tests/test_robot.py` 与 `tests/test_utils.py` 会因为新配置字段还没做而**红**
> —— 那是预期的，Task 5/6 会修。本步只跑 `test_fake_arm_contract.py`。

- [ ] **Step 5: 提交**

```bash
git add tests/conftest.py tests/test_fake_arm_contract.py
git commit -m "test: reshape FakeArm after the real direct-CDC SDK"
```

---

## Task 4: `servo.py` —— 伺服环与下发点

**Files:**

- Create: `src/litearm_lerobot/servo.py`
- Test: `tests/test_servo.py`

- [ ] **Step 1: 写失败测试**

创建 `tests/test_servo.py`：

```python
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
```

- [ ] **Step 2: 跑测试确认失败**

Run: `/usr/bin/python3 -m pytest tests/test_servo.py -q`
Expected: FAIL —— `ModuleNotFoundError: No module named 'litearm_lerobot.servo'`

- [ ] **Step 3: 写实现**

创建 `src/litearm_lerobot/servo.py`：

```python
"""伺服环 —— 本仓**唯一**让机械臂动起来的地方。

## 执行器：`arm.joint_follow`（`CMD_JOINT_FOLLOW 0x08`，伺服环在**固件**里）

PC 只喂 `(q_cmd, dq_cmd, K, B)`；重力前馈 `G(q_meas)` 与限位墙力由固件每拍自算
（`τ_ff = clamp(G + wall, ±tau_max)`，照 litearm-server 的 `compute_tau_ff`）
⇒ 每拍 **1 次**往返。

参考生成 `slew_target`、参数真值、钳位、对齐与收尾**逐条照 litearm-server**，
见 `safety.py` 与 spec §5 / §7。

## `_send()` 是唯一的下发点

换执行器 = 换这一个函数 + 一个配置值。`actuator="move_js"` 已实现但**非默认**，
理由是它放弃了随帧增益通道、且它的 `dq` 语义是静默失败雷区（`dq=0` ⇒ 参考冻结、
臂纹丝不动）。见 spec §5.1 / §5.4。

## ⚠ `_send()` 只发 q/dq/K/B —— **不发 tau**

`0x08` 这一条专用通道由固件补 `G` 与墙力。⛔ 别把这条待遇记到通用路径上：
`move_mit` / `move_mit_all` / `move_js + 用户 tau_ff` 那几条**永不叠加内置前馈**。
`move_js` 不带 `tau_ff` 时走 `builtin_mode`，由固件补 `G(q_d)` —— 见 spec §5.1。
"""
from __future__ import annotations

import logging
import threading
import time
from typing import List, Optional, Sequence

from .safety import Limits, slew_target

log = logging.getLogger(__name__)

__all__ = [
    "ServoLoop", "hold_at_current", "ACTUATORS",
    "DEFAULT_K_P", "DEFAULT_K_D", "DEFAULT_SPEED_LIMIT", "DEFAULT_ACCEL_LIMIT",
    "ENGAGE_KP", "ENGAGE_KD", "HOLD_SPEED",
]

# ── 参数真值 ────────────────────────────────────────────────────────────────
#: 伺服增益。**来源 = `litearm-teleop-isomorphic/liteteleop/servo.py` 的
#: `SETUP_K` / `SETUP_B`（2026-09-29 那版，即台账 P3 行）。**
#:
#: ⚠⚠ **不是本仓实测**，而且**不是被"↓55%/↓86%"验证过的那一版** —— 那两组数字
#:   属于台账 P2 行（K 133 / B 2.4）。P3 那行自己的记录是「本轮不是干净对照实验…
#:   绝对值不可跨轮直比」，实测结论是用户在**两版之间手感无差别**。选 P3 的理由
#:   是它更贴固件出厂值（腕部 J5~J7 只给 80 / 1.5，出厂 `mit_kp` 是 50）。
#: ⚠ 那份调参的场景是**主从遥操**（人手拖主臂、目标连续平滑），本仓还有策略回放
#:   （目标可能跳变）⇒ 这批值在本仓只能算**起点**。
DEFAULT_K_P = [200.0, 200.0, 200.0, 200.0, 80.0, 80.0, 80.0]
#: 阻尼。J2 = 5.0 是**顶到固件上限** `MIT_KD_MAX`（`control_loop.c:45`）——
#: 写 6.0 会被固件**静默钳成 5.0**，表上看着满足等比、实际值不符。故写真值。
DEFAULT_K_D = [3.0, 5.0, 3.0, 3.0, 1.5, 1.5, 1.5]

#: 走位速度 / 加速度上限。**取固件通用档（保守）**，不取同构遥操仓那份。
#:
#: ⚠⚠ **改这张表要连固件一起想**：本仓的 PC 侧 slew 与固件 `slew_linear` 是
#:   **两级串联、谁小谁算**（`control_loop.c:2417`）。固件那张专用表
#:   `s_jf_vel_max`（`control_loop.c:238`）= `[2.8,3.4,5.0,5.0,10.0,8.0,13.0]`，
#:   同构遥操仓的 PC 表逐值等于它 ⇒ 两级同时顶格。本仓取通用档
#:   `[2.0,2.0,1.75,1.75,2.0,2.0,2.0]` ⇒ PC 侧成为唯一约束，
#:   "速度放开"在效果上消失，且**不需要动固件**。见 spec §5.3。
#: ⚠ 判据：`speed_limit[i] <= s_jf_vel_max[i]`（`tests/test_safety.py`）。
DEFAULT_SPEED_LIMIT = [2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0]
DEFAULT_ACCEL_LIMIT = [8.0, 8.0, 7.0, 7.0, 9.0, 9.0, 9.0]

#: `joint_follow.engage()` 的托举增益（照 litearm-server 的
#: `engage_kp=15.0, engage_kd=0.8`）—— **比跟随增益软得多**：接管瞬间"轻轻接住"，
#: 而不是猛拉过去。
ENGAGE_KP = 15.0
ENGAGE_KD = 0.8

#: 收尾 `movej` 的速度（0..1）。慢 —— 它只走"当前位姿到当前位姿"这段零位移。
HOLD_SPEED = 0.3


def _send_joint_follow(arm, q, dq, kp, kd) -> None:
    """`CMD_JOINT_FOLLOW 0x08`：伺服环在固件，`τ_ff` 由固件算。

    ⚠ 只发 q/dq/K/B，**不发 tau**（帧里根本没有这个字段）。
    ⚠ `kp`/`kd` 会被固件钳到 `MIT_KP_MAX=500` / `MIT_KD_MAX=5.0`。
    """
    arm.joint_follow(list(q), list(dq), list(kp), list(kd))


def _send_move_js(arm, q, dq, kp, kd) -> None:
    """`CMD_MOVE_JS 0x03`：通用路径，**K/B 由固件出厂参数定死，这里收不到**。

    ⚠ `dq` 在这里**不是**"速度上限"，而是参考 slew 的**速率来源**
    （`v_lim = clamp(abs(target_dq), 0, speed_limit×gov)`）⇒ **`dq=0` 会让
    参考冻结、臂纹丝不动，且不报错**。本函数的 `dq` 由 `slew_target` 产出，
    所以天然非零（`tests/test_servo.py::test_dq_is_never_all_zero_while_moving`）。
    ⚠ 不传 `tau_ff` ⇒ 固件 `builtin_mode` 生效，补 `G(q_d)`。一旦传了 `tau_ff`，
       内置前馈会**整段关掉**。
    """
    arm.move_js(list(q), list(dq))


#: 执行器表。`_send()` 是唯一的下发点 —— 换执行器只改这里 + 一个配置值。
ACTUATORS = {
    "joint_follow": _send_joint_follow,
    "move_js": _send_move_js,
}


def hold_at_current(arm) -> None:
    """受控接管 —— 用 `movej` 把臂**交回固件的持位语义**。

    ⚠ **用 `movej`（会把固件切回 `MOVE_J` 模式）** —— 这正是收尾想要的：交给固件的
    S 曲线 + 到位增刚。目标 = **当前实测位姿**（零位移），所以它只做"接管"。
    ⛔ **绝不 `disable()`**：失能会让臂在自重下落，可能漂出软限位并锁存成那个
    只能人工推回去的状态。
    ⚠ `movej` 在 `MOVE_J` 下**自己 kick 看门狗**（`control_loop.c:1938`），
    所以它不会中途 fail-soft。
    """
    st = arm.get_state(refresh=True).value
    if st is None:
        raise RuntimeError("收尾取不到状态帧")
    arm.movej(list(st.q), speed=HOLD_SPEED)


def _q_meas(arm) -> List[float]:
    """读**实测**关节角。`refresh=False` 走 SDK 缓存（实测 0.001 ms）；
    `refresh=True` 是 10 ms，进了循环就等于把节拍钉死。"""
    st = arm.get_state(refresh=False).value
    if st is None:
        raise RuntimeError("状态帧取不到（链路静默）")
    return [float(v) for v in st.q]


class ServoLoop:
    """后台伺服线程。

    ## 生命周期

        loop = ServoLoop(arm, limits, ...); loop.start()
        loop.set_target(q)        # 非阻塞；只更新目标
        loop.stop()               # join；幂等

    ## 状态

    * `q_cmd` / `dq_cmd` —— 当前参考（供测试与诊断读，**不参与控制**）
    * `error` —— 线程里抛出的异常对象。非 `None` ⇒ 线程已退出，调用方必须
      **显式抛**（见 `robot.py` 的 `_raise_if_servo_died`）

    ## ⚠ 失败方向（必须写进用户文档）

    线程消失（进程被 `kill -9`）⇒ 固件 0.1 s 看门狗 fail-soft ⇒ **臂缓慢下垂**。
    这是 `0x08` 通道的固有性质，不是缺陷。`0x03` 同理。两者走**同一个**
    `watchdog_check()`，超时都是 0.1 s。
    """

    def __init__(
        self,
        arm,
        limits: Limits,
        *,
        actuator: str = "joint_follow",
        hz: float = 250.0,
        k_p: Optional[Sequence[float]] = None,
        k_d: Optional[Sequence[float]] = None,
        speed_limit: Optional[Sequence[float]] = None,
        accel_limit: Optional[Sequence[float]] = None,
        engage_sec: float = 0.3,
    ) -> None:
        if actuator not in ACTUATORS:
            raise ValueError(f"未知执行器 {actuator!r}，可选 {sorted(ACTUATORS)}")
        self._arm = arm
        self._limits = limits
        self._send = ACTUATORS[actuator]
        self.actuator = actuator
        self._hz = float(hz)
        self._dt = 1.0 / max(self._hz, 1.0)
        self._k_p = [float(v) for v in (k_p if k_p is not None else DEFAULT_K_P)]
        self._k_d = [float(v) for v in (k_d if k_d is not None else DEFAULT_K_D)]
        self._sp = [float(v) for v in (
            speed_limit if speed_limit is not None else DEFAULT_SPEED_LIMIT)]
        self._ac = [float(v) for v in (
            accel_limit if accel_limit is not None else DEFAULT_ACCEL_LIMIT)]
        self._engage_sec = float(engage_sec)

        self._lock = threading.Lock()
        self._target: Optional[List[float]] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        #: 当前参考（测试/诊断读；控制循环自己持有权威副本）
        self.q_cmd: Optional[List[float]] = None
        self.dq_cmd: Optional[List[float]] = None
        #: 线程抛出的异常。非 None ⇒ 线程已死，调用方必须抛。
        self.error: Optional[BaseException] = None

    # ── 对外 ────────────────────────────────────────────────────────────
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self.error = None
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="litearm-servo")
        self._thread.start()

    def stop(self) -> None:
        """停线程并 join。**幂等**。"""
        self._stop.set()
        th = self._thread
        if th is not None:
            th.join(timeout=2.0)
            if th.is_alive():
                log.warning(
                    "伺服线程 2 s 内没停下 —— 臂可能仍在伺服态。"
                    "固件 0.1 s 看门狗会在断流后 fail-soft。")
        self._thread = None

    def set_target(self, q: Sequence[float]) -> None:
        """更新目标（非阻塞）。"""
        with self._lock:
            self._target = [float(v) for v in q]

    # ── 线程体 ──────────────────────────────────────────────────────────
    def _run(self) -> None:
        try:
            self._serve()
        except BaseException as exc:          # noqa: BLE001 —— 要存下来给调用方
            log.error("伺服线程退出：%s", exc, exc_info=True)
            # ⚠⚠ 顺序是 **先接管、后记错**（spec §7.5）：`error` 一置，调用方就会
            #   在下一个 `send_action` 上抛 —— 那意味着"这条臂已经没人管了"这句
            #   话必须**在接管做过之后**才成立。反过来写会让调用方在臂还没被接管
            #   时就收到死讯，也会让"等 error 再断言 movej"的判据变成竞态。
            try:
                hold_at_current(self._arm)
                log.warning("已受控接管（movej 回当前实测位姿）")
            except Exception:                 # noqa: BLE001
                log.exception(
                    "受控接管也失败了 —— 臂会在 0.1 s 后进入 fail-soft（下垂）")
            self.error = exc

    def _serve(self) -> None:
        kp, kd = self._k_p, self._k_d
        sp, ac = self._sp, self._ac
        n = self._limits.n

        # ── prime：托住实测位姿。兼作 0x08 能力探针（固件没这条命令时这里就抛）。
        q_cmd = _q_meas(self._arm)
        dq_cmd = [0.0] * n
        q_target = list(q_cmd)
        self._publish(q_cmd, dq_cmd)
        self._send(self._arm, q_cmd, dq_cmd, kp, kd)

        # ── engage：`engage_sec` 内用**低刚度**托住，减轻接管冲击
        if self._engage_sec > 1e-6:
            kp_e = [ENGAGE_KP] * n
            kd_e = [ENGAGE_KD] * n
            t_end = time.monotonic() + self._engage_sec
            while time.monotonic() < t_end and not self._stop.is_set():
                self._send(self._arm, q_cmd, [0.0] * n, kp_e, kd_e)
                self._sleep_to_next(time.monotonic())
            q_cmd = _q_meas(self._arm)
            dq_cmd = [0.0] * n
            q_target = list(q_cmd)

        base = time.monotonic()
        next_tick = base + self._dt
        while not self._stop.is_set():
            t0 = time.monotonic()
            got = self._take_target()
            if got is not None:
                q_target = got

            # 参考生成：逐字照抄 joint_follow（梯形速度曲线 + 制动距离）
            q_cmd, dq_cmd = slew_target(q_target, q_cmd, dq_cmd, sp, ac, self._dt)
            self._publish(q_cmd, dq_cmd)
            self._send(self._arm, q_cmd, dq_cmd, kp, kd)
            self._watch_firmware(q_cmd)

            # 节拍：主动 sleep 对齐到点，不是"能跑多快"
            next_tick += self._dt
            r = next_tick - time.monotonic()
            if r > 0:
                time.sleep(r)
            elif next_tick < time.monotonic():
                # 掉帧了：重锚，别让 next_tick 越欠越多
                next_tick = time.monotonic() + self._dt

    # ── 内部 ────────────────────────────────────────────────────────────
    def _take_target(self) -> Optional[List[float]]:
        with self._lock:
            return list(self._target) if self._target is not None else None

    def _publish(self, q_cmd: List[float], dq_cmd: List[float]) -> None:
        # 单次赋值，读方（测试/诊断）不需要锁
        self.q_cmd = list(q_cmd)
        self.dq_cmd = list(dq_cmd)

    def _sleep_to_next(self, t0: float) -> None:
        r = (t0 + self._dt) - time.monotonic()
        if r > 0:
            time.sleep(r)

    def _watch_firmware(self, q_cmd: List[float]) -> None:
        """固件状态**一变就记一行**（读缓存，不进循环开销）。

        ⚠ 故障的**先后顺序**只能从这里看出来：杀完进程再查状态看到的是"结果"，
        而且会混进自己 kill 造成的 `WD_TRIPPED` —— 永远抓不到现场。
        """
        snap = self._arm.get_state(refresh=False).value
        if snap is None or not hasattr(snap, "joints"):
            return
        cur = (tuple(getattr(snap, "flag_names", ()) or ()),
               int(getattr(snap, "joint_fault", 0) or 0))
        if cur == getattr(self, "_prev_fw", None):
            return
        self._prev_fw = cur
        err = max(abs(a - b) for a, b in zip(q_cmd, snap.q)) if snap.q else 0.0
        log.warning(
            "固件状态变化 flags=%s joint_fault=0x%X 最大跟踪误差=%.4f rad q=%s",
            list(cur[0]), cur[1], err,
            [round(float(v), 3) for v in snap.q])
```

- [ ] **Step 4: 跑测试确认通过**

Run: `/usr/bin/python3 -m pytest tests/test_servo.py -q`
Expected: PASS（15 条）

> 若 `test_tick_rate_is_close_to_the_configured_hz` 在 CI 上偶发失败，把阈值从
> `n > 25` 放宽到 `n > 15`，**不要删这条** —— 它是"判据的判据"。

- [ ] **Step 5: 提交**

```bash
git add src/litearm_lerobot/servo.py tests/test_servo.py
git commit -m "feat: add the servo loop driving arm.joint_follow"
```

---

## Task 5: `config.py` 重写

**Files:**

- Rewrite: `src/litearm_lerobot/config.py`
- Test: `tests/test_config.py`

- [ ] **Step 1: 写失败测试**

创建 `tests/test_config.py`：

```python
"""配置契约 —— 字段名与默认值都是对外契约，改名即破坏消费者。"""
from __future__ import annotations

import pytest

from litearm_lerobot import LiteArmRobotConfig
from litearm_lerobot.servo import (
    DEFAULT_ACCEL_LIMIT,
    DEFAULT_K_D,
    DEFAULT_K_P,
    DEFAULT_SPEED_LIMIT,
)


def test_registered_under_litearm():
    assert LiteArmRobotConfig().type == "litearm"


def test_old_server_fields_are_gone():
    """六个字段消失或改名 —— 见 spec §11.1，PR 里要明说破坏消费者。"""
    cfg = LiteArmRobotConfig()
    for gone in ("endpoint", "arm_id", "use_commander", "movej_speed", "settle_s",
                 "query_timeout"):
        assert not hasattr(cfg, gone), f"{gone} 是旧 server 语义的字段"


def test_new_defaults():
    cfg = LiteArmRobotConfig()
    assert cfg.port is None              # None = find_cdc_port() 自动找
    assert cfg.move_timeout == 15.0
    assert cfg.num_joints == 7
    assert cfg.servo_hz == 250.0
    assert cfg.actuator == "joint_follow"
    assert cfg.enable_on_connect is True
    assert cfg.disable_on_disconnect is False
    assert cfg.limit_margin == 0.01      # ⚠ 不是 0.02（会毁掉 J4 正半轴）
    assert cfg.engage_sec == 0.3


def test_servo_table_defaults_come_from_servo_module():
    cfg = LiteArmRobotConfig()
    assert cfg.k_p is None and cfg.k_d is None
    assert cfg.speed_limit is None and cfg.accel_limit is None
    assert DEFAULT_K_P == [200.0, 200.0, 200.0, 200.0, 80.0, 80.0, 80.0]
    assert DEFAULT_K_D == [3.0, 5.0, 3.0, 3.0, 1.5, 1.5, 1.5]
    assert DEFAULT_SPEED_LIMIT == [2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0]
    assert DEFAULT_ACCEL_LIMIT == [8.0, 8.0, 7.0, 7.0, 9.0, 9.0, 9.0]


def test_actuator_only_accepts_known_names():
    with pytest.raises(ValueError):
        LiteArmRobotConfig(actuator="nope").validate()
```

- [ ] **Step 2: 跑测试确认失败**

Run: `/usr/bin/python3 -m pytest tests/test_config.py -q`
Expected: FAIL —— `litearm_lerobot.servo` 还不存在 / 旧字段还在

- [ ] **Step 3: 写实现**

创建 `src/litearm_lerobot/config.py`：

```python
"""LeRobot config for the LiteArm robotic arm (litearm-python direct CDC)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from lerobot.robots.config import RobotConfig

from .servo import (
    ACTUATORS,
    DEFAULT_ACCEL_LIMIT,
    DEFAULT_K_D,
    DEFAULT_K_P,
    DEFAULT_SPEED_LIMIT,
)


@RobotConfig.register_subclass("litearm")
@dataclass(kw_only=True)
class LiteArmRobotConfig(RobotConfig):
    # -- 连接 -----------------------------------------------------------------
    #: CDC 端口。`None` = `litearm.find_cdc_port()` 按 VID:PID 1d50:606f 自动找。
    #: ⚠ 别写死 `/dev/ttyACM0` —— 两个 ACM 口的编号会互换。
    port: Optional[str] = None
    #: 单次阻塞运动的超时（秒）。`movej` 到位即提前返回，这只是上限。
    move_timeout: float = 15.0

    # -- 运动 ----------------------------------------------------------------
    #: 关节数。连接后与 `arm.n` 对账，不符即抛（1J 台架板的 n 是 1）。
    num_joints: int = 7
    #: 伺服环节拍（Hz）。它直接进 `slew_target` 的 `dt`，**必须接近真实周期**。
    servo_hz: float = 250.0
    #: 执行器。`joint_follow`（默认）或 `move_js`，见 spec §5.4。
    actuator: str = "joint_follow"
    #: 伺服增益。`None` ⇒ 用 servo 模块的默认表。
    #: ⚠ 只在 `actuator="joint_follow"` 时有效 —— `move_js` 没有随帧增益通道，
    #:   传了会被忽略（连接时会告警，不静默）。
    k_p: Optional[List[float]] = None
    k_d: Optional[List[float]] = None
    #: 速度/加速度上限。`None` ⇒ 用 servo 模块的**通用档**（保守）。
    #: ⚠ 放开到固件 `s_jf_vel_max` 那张表就是同构遥操仓的配置，那时响应最快。
    speed_limit: Optional[List[float]] = None
    accel_limit: Optional[List[float]] = None
    #: 接管托举时长（秒）。照 litearm-server 的 `engage_sec`。
    engage_sec: float = 0.3

    # -- 安全 ----------------------------------------------------------------
    #: 软限位内缩量（rad）。⚠⚠ **不能调大**：J4 的固件上端只有 +0.017547，
    #: 取 0.02 就把 J4 正半轴抹掉（且不报错）。见 safety.DEFAULT_LIMIT_MARGIN。
    limit_margin: float = 0.01
    #: 连接时使能电机。直连下 enable 失败 = 臂不会动 ⇒ 抛（不再吞异常）。
    enable_on_connect: bool = True
    #: 收尾时失能。`False`（默认）⇒ `movej` 回当前实测位姿，交回固件持位。
    #: ⚠ 设 `True` 会让臂在自重下落 —— 只有在臂已落在支撑面上时才这么设。
    disable_on_disconnect: bool = False

    def validate(self) -> None:
        """把不合法的配置挡在连接之前。"""
        if self.actuator not in ACTUATORS:
            raise ValueError(
                f"actuator 只能是 {sorted(ACTUATORS)}，收到 {self.actuator!r}"
            )
        if self.num_joints <= 0:
            raise ValueError(f"num_joints 必须为正，收到 {self.num_joints}")
        if self.servo_hz <= 0.0:
            raise ValueError(f"servo_hz 必须为正，收到 {self.servo_hz}")
        if self.limit_margin < 0.0:
            raise ValueError(f"limit_margin 不能为负，收到 {self.limit_margin}")
        for name in ("k_p", "k_d", "speed_limit", "accel_limit"):
            vals = getattr(self, name)
            if vals is not None and len(vals) != self.num_joints:
                raise ValueError(
                    f"{name} 需要 {self.num_joints} 个值，收到 {len(vals)}"
                )
        if self.actuator == "move_js" and (self.k_p is not None or self.k_d is not None):
            # ⚠ move_js 没有随帧增益通道。这里**不抛**（抛会让一个本来能跑的
            #   配置起不来），但必须让调用方知道自己的值被忽略了 —— 由 robot.py
            #   在 connect() 时 log.warning（见 Task 6）。
            pass
```

- [ ] **Step 4: 跑测试确认通过**

Run: `/usr/bin/python3 -m pytest tests/test_config.py -q`
Expected: PASS（5 条）

- [ ] **Step 5: 提交**

```bash
git add src/litearm_lerobot/config.py tests/test_config.py
git commit -m "feat: rewrite the config for the direct-CDC backend"
```

---

## Task 6: `robot.py` 重写

**Files:**

- Rewrite: `src/litearm_lerobot/robot.py`
- Test: `tests/test_robot.py`（重写）

- [ ] **Step 1: 写失败测试**

创建 `tests/test_robot.py`（替换原文件）：

```python
"""LiteArmRobot 生命周期测试。"""
from __future__ import annotations

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
```

- [ ] **Step 2: 跑测试确认失败**

Run: `/usr/bin/python3 -m pytest tests/test_robot.py -q`
Expected: FAIL（大量 —— 旧 `robot.py` 收 `endpoint`、调 `hold()`）

- [ ] **Step 3: 写实现**

把 `src/litearm_lerobot/robot.py` 整个替换为：

```python
"""LeRobot-compatible driver for the LiteArm robotic arm.

Implementing the abstract ``lerobot.robots.Robot`` interface on top of
litearm-python's **direct USB CDC** client (``litearm.Arm``). There is no
litearm-server and no zenoh in this path.

The arm reports absolute joint positions from its controller, so no homing or
joint calibration is needed — ``calibrate()`` is a no-op and ``is_calibrated``
is always ``True``.

Motion is driven by a background :class:`~litearm_lerobot.servo.ServoLoop`
thread. ``send_action`` only updates the loop's target and returns immediately.
See the design spec for why the servo loop lives in the firmware and what that
costs when the host process dies.
"""
from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

from lerobot.robots.robot import Robot

import litearm

from .config import LiteArmRobotConfig
from .safety import clamp_to_limits, read_safe_limits
from .servo import ServoLoop, hold_at_current

log = logging.getLogger(__name__)


class LiteArmRobot(Robot):
    """Drive a LiteArm through the LeRobot ``Robot`` interface.

    Observation / action vectors are the seven joint positions reported by the
    arm controller::

        observation = {"observation.state": [q0, q1, q2, q3, q4, q5, q6]}
        action      = {"action":                [q0, q1, q2, q3, q4, q5, q6]}

    ``send_action`` is **non-blocking**: it clamps the target into the soft
    limits and hands it to a background servo loop running at
    ``config.servo_hz``.
    """

    name = "litearm"
    config_class = LiteArmRobotConfig

    def __init__(self, config: LiteArmRobotConfig) -> None:
        super().__init__(config)
        self.config = config  # Robot.__init__ does not store the config
        self._arm: Optional[litearm.Arm] = None
        self._servo: Optional[ServoLoop] = None
        self._limits = None

    # ── LeRobot feature metadata ─────────────────────────────────────────────

    @property
    def observation_features(self) -> Dict[str, Any]:
        return {"observation.state": (self.config.num_joints,)}

    @property
    def action_features(self) -> Dict[str, Any]:
        return {"action": (self.config.num_joints,)}

    @property
    def is_connected(self) -> bool:
        return self._arm is not None

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def connect(self, calibrate: bool = True) -> None:
        """Open the CDC link, read the limits, enable, and start the servo loop.

        Raises on any failure and leaves the link closed — a half-open session
        with an arm that will not move is worse than a loud exception.
        """
        if self.is_connected:
            return
        self.config.validate()
        if self.config.actuator == "move_js" and (
            self.config.k_p is not None or self.config.k_d is not None
        ):
            log.warning(
                "actuator=%r 没有随帧增益通道：配置里的 k_p/k_d 会被**忽略**。"
                "要调增益请用 actuator='joint_follow'（0x08 通道的 kp/kd 随帧下发）。",
                self.config.actuator,
            )

        arm = litearm.Arm(port=self.config.port,
                          move_timeout=self.config.move_timeout)
        try:
            arm.connect()
            if arm.n != self.config.num_joints:
                raise ValueError(
                    f"关节数不符：固件报 {arm.n}，配置是 {self.config.num_joints}。"
                    f"（1J 台架板报 1；本仓默认是 7J 整臂）"
                )
            limits = read_safe_limits(arm, self.config.limit_margin)
            if self.config.enable_on_connect:
                arm.enable()
        except BaseException:
            # ⚠ 收干净再抛 —— 否则串口句柄留在一个半开的会话里
            arm.close()
            raise

        self._arm = arm
        self._limits = limits
        self._servo = ServoLoop(
            arm,
            limits,
            actuator=self.config.actuator,
            hz=self.config.servo_hz,
            k_p=self.config.k_p,
            k_d=self.config.k_d,
            speed_limit=self.config.speed_limit,
            accel_limit=self.config.accel_limit,
            engage_sec=self.config.engage_sec,
        )
        self._servo.start()
        if calibrate:
            self.calibrate()

    @property
    def is_calibrated(self) -> bool:
        # Absolute encoders are read directly from the arm controller: no homing.
        return True

    def calibrate(self) -> None:
        """No-op — the LiteArm reports absolute joint positions."""
        return None

    def configure(self) -> None:
        """No-op — gains and limits live in the firmware."""
        return None

    # ── LeRobot observation / action ─────────────────────────────────────────

    def _raise_if_servo_died(self) -> None:
        """把伺服线程的死因**显式**抛给调用方。

        ⚠ 不做"线程死了、调用方照旧拿到旧数据" —— 那会让一条已经失控的臂看起来
        一切正常。`ServoLoop.error` 是那个不变量的具名载体。
        """
        if self._servo is not None and self._servo.error is not None:
            # ⚠ 把死因**写进消息**，不只是 `from` —— 调用方（以及 pytest 的
            #   `match=`）只看得到这一层，链式异常看不见。
            raise RuntimeError(
                f"伺服环已停（{self._servo.actuator} 下发失败）：{self._servo.error}"
            ) from self._servo.error

    def get_observation(self) -> Dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("LiteArmRobot is not connected")
        self._raise_if_servo_died()
        state = self._arm.get_state(refresh=False).value
        if state is None:
            raise RuntimeError("No robot state received yet")
        return {"observation.state": [float(v) for v in state.q]}

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("LiteArmRobot is not connected")
        self._raise_if_servo_died()
        if "action" not in action:
            raise ValueError("action dict must contain an 'action' key")
        q = [float(v) for v in action["action"]]
        if len(q) != self.config.num_joints:
            raise ValueError(
                f"action must have {self.config.num_joints} joints, got {len(q)}"
            )
        clamped, saturated = clamp_to_limits(q, self._limits)
        if any(saturated):
            axes = [f"J{i + 1}" for i, s in enumerate(saturated) if s]
            log.warning(
                "目标超出软限位，已钳位：%s（原值 %s）",
                axes, [round(v, 4) for v in q],
            )
        self._servo.set_target(clamped)      # 非阻塞
        return {"action": clamped}

    def disconnect(self) -> None:
        """Stop the servo loop, hand the arm back to the firmware, close the link.

        The hand-off is a ``movej`` to the measured pose, **not** a ``disable``:
        a disabled arm falls under its own weight and can drift out of the soft
        limits into a latched fault. Set ``disable_on_disconnect`` only when the
        arm is already resting on a support.

        Idempotent. A failing hand-off is logged and does not block closing.
        """
        servo, self._servo = self._servo, None
        arm, self._arm = self._arm, None
        if servo is not None:
            servo.stop()
        if arm is None:
            return
        try:
            if self.config.disable_on_disconnect:
                arm.disable()
            else:
                hold_at_current(arm)
        except Exception:                     # noqa: BLE001
            log.exception(
                "收尾接管失败 —— 链路即将关闭，臂会在 0.1 s 后 fail-soft（下垂）")
        finally:
            arm.close()

    def __str__(self) -> str:
        return f"{self.id} LiteArmRobot"
```

- [ ] **Step 4: 跑测试确认通过**

Run: `/usr/bin/python3 -m pytest tests/test_robot.py -q`
Expected: PASS（20 条）

- [ ] **Step 5: 提交**

```bash
git add src/litearm_lerobot/robot.py tests/test_robot.py
git commit -m "feat: drive LiteArmRobot through the servo loop"
```

---

## Task 7: 对真 SDK 的签名对账判据

**Files:**

- Test: `tests/test_sdk_contract.py`

**这是本次事故的直接补丁。** 它的期望值取自**真 SDK**，不与本仓实现同源 ⇒ 不会自洽地错。

- [ ] **Step 1: 写测试**

```python
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
```

- [ ] **Step 2: 跑测试确认通过**

Run: `/usr/bin/python3 -m pytest tests/test_sdk_contract.py -q`
Expected: PASS（17 条）

> **若这里 FAIL** —— 说明本仓对 SDK 的假设有错，**先改实现，不要改这些判据**。
> 这正是这个文件存在的意义。

- [ ] **Step 3: 提交**

```bash
git add tests/test_sdk_contract.py
git commit -m "test: pin the litearm SDK contract from the real signatures"
```

---

## Task 8: 默认值判据与 `test_utils.py` 收尾

**Files:**

- Test: `tests/test_safety.py`（追加）、`tests/test_utils.py`（改）

- [ ] **Step 1: 追加默认值判据**

在 `tests/test_safety.py` 末尾追加：

```python
# ── 默认值判据：PC 侧速度表不得越过固件那张专用表 ──────────────────────────

#: 固件 `0x08` 会话的专用走位速度表。**硬编码并注明来源** ——
#: ⚠ 它是 `static const float`，定义在 `litearm-stm32` `control_loop.c:238`，
#:   **没有任何头文件声明**；而本仓 CI 从不 checkout 固件仓 ⇒ 只能硬编码。
#: ⚠ 改固件那张表时必须同步改这里。
FW_S_JF_VEL_MAX = [2.8, 3.4, 5.0, 5.0, 10.0, 8.0, 13.0]


def test_default_speed_limit_stays_under_the_firmware_table():
    """⚠ 判别力判据。

    PC 侧 slew 与固件 `slew_linear` 是两级串联、**谁小谁算**。PC 侧一旦调到固件
    表之上，固件那张表就重新成为约束 —— 也就是"速度放开"悄悄回来了。
    """
    from litearm_lerobot.servo import DEFAULT_SPEED_LIMIT

    assert len(DEFAULT_SPEED_LIMIT) == len(FW_S_JF_VEL_MAX)
    for i, (pc, fw) in enumerate(zip(DEFAULT_SPEED_LIMIT, FW_S_JF_VEL_MAX)):
        assert pc <= fw, f"J{i + 1}: PC 侧 {pc} 越过固件 {fw}"


def test_firmware_table_values_are_the_ones_we_think_they_are():
    """判别力：证明上一条不是"两个空表比大小"。"""
    assert FW_S_JF_VEL_MAX[0] == 2.8
    assert FW_S_JF_VEL_MAX[-1] == 13.0
    from litearm_lerobot.servo import DEFAULT_SPEED_LIMIT

    assert max(DEFAULT_SPEED_LIMIT) < max(FW_S_JF_VEL_MAX), "默认档应当更保守"
```

- [ ] **Step 2: 改 `tests/test_utils.py`**

把 `test_register_returns_litearm_robot` 里的假臂与配置换掉（其余两条不动）：

```python
def test_register_returns_litearm_robot(monkeypatch):
    """`register()` 之后，`type == 'litearm'` 的配置能造出 LiteArmRobot。"""
    import litearm

    from litearm_lerobot import LiteArmRobot

    class _Arm:
        def __init__(self, **kw):
            self.n = 7
            self.params = self

        def connect(self):
            return self

        def get_state(self, refresh=False):
            from conftest import FakeMsg, FakeRobotState, FakeJointState
            return FakeMsg(FakeRobotState(
                joints=[FakeJointState() for _ in range(7)]))

        def all_joint_params(self):
            from conftest import FakeJointParam, FakeArm
            return [FakeJointParam(*lo_hi) for lo_hi in FakeArm.LIMITS]

        def enable(self, attempts=12):
            pass

        def close(self):
            pass

    monkeypatch.setattr(litearm, "Arm", lambda *a, **kw: _Arm())
    register()

    from lerobot.robots import utils as robot_utils
    from litearm_lerobot import LiteArmRobotConfig

    robot = robot_utils.make_robot_from_config(LiteArmRobotConfig())
    assert isinstance(robot, LiteArmRobot)
```

- [ ] **Step 3: 跑全量测试**

Run: `/usr/bin/python3 -m pytest -q`
Expected: PASS（全绿，**86 passed**）

- [ ] **Step 4: 提交**

```bash
git add tests/ && git commit -m "test: pin the default speed table and refresh the register tests"
```

---

## Task 9: `__init__.py` 与示例

**Files:**

- Modify: `src/litearm_lerobot/__init__.py`
- Modify: `examples/01_read_observation.py`、`02_send_action.py`、`03_record_dataset.py`
- Modify: `examples/README.md`、`examples/README.zh-CN.md`

- [ ] **Step 1: 改 `__init__.py`**

```python
"""litearm-lerobot: LeRobot driver for the LiteArm robotic arm.

Wraps **litearm-python**'s direct USB CDC client behind the LeRobot ``Robot``
interface, so the arm can be driven with LeRobot-style observation/action
loops, recording and teleoperation. There is no litearm-server in this path.

Usage::

    from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

    robot = LiteArmRobot(LiteArmRobotConfig())      # port=None: auto-detect
    robot.connect()
    obs = robot.get_observation()                   # {"observation.state": [7 floats]}
    robot.send_action({"action": [0.0] * 7})        # non-blocking
    robot.disconnect()                              # hands the arm back to the firmware

``send_action`` does not block: it clamps the target into the soft limits and
hands it to a background servo loop. Run the whole session inside ``with`` so
``disconnect()`` always runs — without it the link is reclaimed only at garbage
collection, and the arm's pose is not handed back at all.
"""

__version__ = "0.1.0"

from .config import LiteArmRobotConfig
from .robot import LiteArmRobot
from .utils import register

__all__ = [
    "LiteArmRobot",
    "LiteArmRobotConfig",
    "register",
    "__version__",
]
```

- [ ] **Step 2: 改 `examples/01_read_observation.py`**

整个替换为：

```python
#!/usr/bin/env python3
"""Read LiteArm observations through the LeRobot Robot interface.

Read-only: connects, prints the observation features and a few state vectors.
Safe to run with the arm powered but idle.
"""
import argparse

from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None,
                    help="CDC port, e.g. /dev/ttyACM0 "
                         "(default: auto-detect VID:PID 1d50:606f)")
    ap.add_argument("--count", type=int, default=5, help="number of samples")
    ap.add_argument("--no-enable", action="store_true",
                    help="do not enable the motors on connect")
    args = ap.parse_args()

    cfg = LiteArmRobotConfig(port=args.port,
                             enable_on_connect=not args.no_enable)
    with LiteArmRobot(cfg) as robot:
        print("observation_features:", robot.observation_features)
        for i in range(args.count):
            obs = robot.get_observation()
            print(f"[{i}] observation.state = "
                  f"{[round(v, 4) for v in obs['observation.state']]}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: 改 `examples/02_send_action.py`**

整个替换为：

```python
#!/usr/bin/env python3
"""Drive a small sine offset through robot.send_action().

MOTION EXAMPLE — this drives the real arm. Keep --amplitude small, put the
emergency stop within reach, and clear the workspace first.

The trajectory is an offset **from the pose the arm is in when you start**, not
an absolute joint vector: commanding absolute angles would fling the arm across
its whole range on the first step.
"""
import argparse
import math
import time

from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None,
                    help="CDC port (default: auto-detect)")
    ap.add_argument("--duration", type=float, default=10.0, help="seconds")
    ap.add_argument("--amplitude", type=float, default=0.05,
                    help="sine amplitude in rad (keep small on first runs)")
    ap.add_argument("--hz", type=float, default=30.0,
                    help="how often send_action() is called (the servo loop "
                         "itself runs at config.servo_hz)")
    args = ap.parse_args()

    cfg = LiteArmRobotConfig(port=args.port)
    with LiteArmRobot(cfg) as robot:
        q0 = list(robot.get_observation()["observation.state"])   # start pose
        print(f"start pose: {[round(v, 4) for v in q0]}")
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            t = time.monotonic() - start
            # Offset from the MEASURED start pose. Passing an absolute joint
            # vector would send the arm across its whole range on the first step.
            q = list(q0)
            q[0] += args.amplitude * math.sin(0.5 * t)
            robot.send_action({"action": q})
            time.sleep(1.0 / args.hz)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 改 `examples/03_record_dataset.py`**

把参数与构造部分替换（其余数据集逻辑不动）：

```python
    ap.add_argument("--port", default=None,
                    help="CDC port (default: auto-detect)")
    ap.add_argument("--task", default="push", help="task label")
    ...
    cfg = LiteArmRobotConfig(port=args.port)
    with LiteArmRobot(cfg) as robot:
        ...
```

> 保留原有的 `LeRobotDataset` 逻辑与 `"task"` 键写法不变。

- [ ] **Step 5: 改 `examples/README.md` 与 `README.zh-CN.md`**

- 删掉 `--endpoint` / `--arm-id`，改为 `--port`（默认自动找口）
- 前置条件从 "litearm-server 可达" 改为 "机械臂通过 USB CDC 直连本机"
- 在 02 那条下面加一行 **do not**：

```markdown
> **Do not** pass absolute joint angles as the trajectory. That commands the arm
> across its full range on the first step. Both example 02 and any policy
> rollout should offset from the pose read at start-up.


`README.zh-CN.md` 用对应的中文块（**不要**照抄英文那份）：

```markdown
> **不要**把绝对关节角当成轨迹传进去 —— 那会在第一步就把机械臂甩过整个行程。
> 示例 02 与任何策略回放都应从**启动时读到的位姿**偏移。
``````

- [ ] **Step 6: 跑一遍只读示例自检（错误必须是**单行清晰**的 `TransportError`）**

Run: `/usr/bin/python3 examples/01_read_observation.py --count 1`

Expected 是**两支之一**，取决于本机此刻插没插臂：

| 本机状态 | 期望 |
|---|---|
| 没有 STM32 CDC 设备 | `TransportError: 未找到 STM32 CDC (VID:PID 1d50:606f), 请用 --port 指定` |
| 有设备、但被别的进程占着 | `TransportError: 打开串口 /dev/ttyACMn 失败: ... Could not exclusively lock port ...` |

判断的要点是**错误清晰**（单行 `TransportError`，而不是 traceback 里一堆 `None`），
不是命中哪一支。

⛔⛔ **别为了"让这一支跑出来"去拔/占用别人的口。** STM32 CDC 口在 Linux 上
**不独占** —— 第二个进程会**分吃同一个字节流**，两边一起坏（本组织实测过）。若设备
存在但被占，看到的就是上面第二支；**直接放过这一步，不要去抢**。

⛔ **也不要拿这一步当真机验收**。它只证明"连接失败时错误可读"。真机验收在
spec §11.2，需要人在场、手边有急停。

- [ ] **Step 7: 提交**

```bash
git add src/litearm_lerobot/__init__.py examples/
git commit -m "docs: point the examples at the CDC port and offset from the start pose"
```

---

## Task 10: README 与开发者指南

**Files:**

- Modify: `README.md`、`README.zh-CN.md`、`docs/DEVELOPER_GUIDE.md`、`docs/DEVELOPER_GUIDE.zh-CN.md`

**规范要求（`AGENTS.md` §4）：** 每份文档**开头一句**说清"这是什么、给谁看"；标题 sentence-case、不超过三级；`do not` 行；**无 emoji**；英文为主 + `zh-CN` 双份。

- [ ] **Step 1: 改 `README.md`**

逐项：

- 架构图换成：

```text
LeRobot Robot API ──→ LiteArmRobot ──→ litearm.Arm ──→ USB CDC ──→ STM32 ──→ motors
```

- `Requirements` 表：删掉 "Runtime: a reachable litearm-server"，改为：

| Item | Requirement |
|---|---|
| Python | 3.10+ |
| LeRobot | `lerobot>=0.3.0,<0.5` |
| Base SDK | `litearm-python>=2.1.0` (not on PyPI — install from its git repository) |
| Firmware | `Litearm1.5.0` or newer, with `CMD_JOINT_FOLLOW` (`0x08`) |
| Hardware | The arm connected over USB CDC (`1d50:606f`) to this machine |

- `Configuration` 一节整表换成 `config.py` 里的字段（`port` / `move_timeout` /
   `num_joints` / `servo_hz` / `actuator` / `k_p` / `k_d` / `speed_limit` /
   `accel_limit` / `engage_sec` / `limit_margin` / `enable_on_connect` /
   `disable_on_disconnect`）。
- 清掉所有 emoji（`📖`）。
- 加一节 **Motion and safety**，至少包含这些 **do not**：

```markdown
## Motion and safety

`send_action` is non-blocking. It clamps the target into the soft limits read
from the firmware and hands it to a background servo loop, which streams
`arm.joint_follow` (`0x08`) at `servo_hz`. The servo loop, the soft-limit wall
and the gravity feedforward all run in the firmware.

**Do not** assume the arm holds when your process dies. Kill the process and the
servo stream stops; the firmware's 0.1 s command watchdog then fail-softs the arm
— it goes limp and **slowly sags** under its own weight. This is inherent to the
continuous-servo channel, not a bug. Keep the workspace clear and run long jobs
with someone nearby.

**Do not** call `disable()` on exit. Disabling drops the arm under gravity and it
can drift out of the soft limits into a latched fault that only a person can
clear by pushing the joint back. `disconnect()` hands the arm back with a
zero-motion `movej`, which keeps it rigid. Set `disable_on_disconnect=True` only
when the arm is already resting on a support.

**Do not** raise `limit_margin`. J4's upper soft limit is only `+0.017547 rad`
(1°). A margin of `0.02` puts J4's upper bound at `-0.0025`, silently removing its
entire positive range — and nothing raises an error.

**Do not** hard-code `/dev/ttyACM0`. The two CDC ports on this machine swap
numbers between boots; leave `port=None` and let the SDK match `VID:PID 1d50:606f`.

**Do** run the session inside `with`, so `disconnect()` always runs.
```

- `Motion` 那一行的实现从 `Arm.movej() via a non-blocking background commander`
   改为 `arm.joint_follow (0x08) via a background servo loop`。

- [ ] **Step 2: 同步 `README.zh-CN.md`**

与上一步逐节对应（同一套表、同一套 do not 行）。

- [ ] **Step 3: 改 `docs/DEVELOPER_GUIDE.md`**

按顺序改：

- 开头一句（规范要求）：`How to work on litearm-lerobot: the LeRobot Robot contract, the config surface, the servo loop, and the failure modes you must not assume away. For developers extending or debugging this driver.`
- 架构图同上。
- §2 Configuration 表换成新字段。
- §3 的 `send_action — blocking vs commander` 整节替换为 `send_action and the servo loop`，说明：非阻塞、单点下发 `_send()`、`actuator` 两个取值及各自代价。
- §6 Tests 增加一句：`tests/test_sdk_contract.py` 的期望值取自**真 SDK**，是防止判据与实现同源的护栏；改本仓对 SDK 的调用方式时必须同步改它。
- 加 **Do not** 小节，内容与 README 的 Motion and safety 一致（不重复长文，链接过去）。

- [ ] **Step 4: 同步 `docs/DEVELOPER_GUIDE.zh-CN.md`**

- [ ] **Step 5: 孤儿检查**

逐个确认每份文档都被引用到：

```bash
grep -rn "DEVELOPER_GUIDE\|examples/README\|TROUBLESHOOTING" README.md README.zh-CN.md docs/ examples/
```

Expected: `docs/DEVELOPER_GUIDE.md` 与 `docs/DEVELOPER_GUIDE.zh-CN.md` 都能从
`README.md` / `README.zh-CN.md` 找到；`examples/README.md` 能从两份 README 找到。
**任何一个找不到 ⇒ 补上链接。**

- [ ] **Step 6: lint**

Run: `npx --no-install markdownlint-cli2 "*.md" "docs/**/*.md" "examples/**/*.md"`
Expected: 0 issues（若报行宽/表格列数，按 `.markdownlint.json` 的配置修）

- [ ] **Step 7: 提交**

```bash
git add README.md README.zh-CN.md docs/DEVELOPER_GUIDE.md docs/DEVELOPER_GUIDE.zh-CN.md
git commit -m "docs: rewrite the guides for the direct-CDC backend"
```

---

## Task 11: 仓库规范收口

**Files:**

- Modify: `pyproject.toml`
- Check: `.github/workflows/ci.yml`

- [ ] **Step 1: 改 `pyproject.toml`**

```diff
 [project]
 name = "litearm-lerobot"
-version = "0.1.0"
+version = "0.0.0-semantic-release"
 description = "LeRobot driver for the LiteArm robotic arm (wraps litearm-python)"
 readme = "README.md"
 requires-python = ">=3.10"
 license = "LicenseRef-Proprietary"
 license-files = ["LICENSE"]
 authors = [{ name = "luochun" }]
 keywords = ["robotics", "robot-arm", "litearm", "lerobot"]
 dependencies = [
-    "litearm-python>=0.1.0",
+    "litearm-python>=2.1.0",
     "lerobot>=0.3.0,<0.5",
 ]
```

> ⚠ `version` 改成占位符是 `AGENTS.md` §3 的硬规矩：清单里的版本号是占位符，
> **唯一真源是 git tag**。不要参考 `litearm-python` 那份（它保留了真实版本号，
> 那是它自己的偏离，不是本仓的范例）。
> ⚠ `__init__.py` 里的 `__version__ = "0.1.0"` **保留**：它是包自述，不是清单的
> version 字段。若要严格对齐，改成读 `importlib.metadata` —— 本次不做（YAGNI）。

- [ ] **Step 2: 核对 `ci.yml`**

Run: `cat .github/workflows/ci.yml`
Expected: 安装步骤里是
`python -m pip install "litearm-python @ git+https://github.com/nexform-tech/litearm-python"`
且 `pytest -q` 是真测试命令、**没有 `exit 1` 占位**。**保持原样**（`litearm-python` 不在 PyPI 上，必须走 git）。

- [ ] **Step 3: 跑全量测试 + 模拟 CI 安装**

Run: `/usr/bin/python3 -m pytest -q`
Expected: PASS

- [ ] **Step 4: 提交**

```bash
git add pyproject.toml
git commit -m "build: use the release placeholder version and require SDK 2.1"
```

---

## Task 12: 交付前收口

- [ ] **Step 1: 全量测试**

Run: `/usr/bin/python3 -m pytest -q`
Expected: 全绿 **86 passed**。把**逐字的输出**记下来。

> **复现"计划代码已跑过"这个说法**：把本计划的代码块逐字抽出来独立跑一遍，
> 与"按计划实现"是两条独立路径 —— 前者证明计划本身可执行，后者证明实现落了地。
> 抽取脚本（把 `创建/追加/替换为 \`path\`` 后面的 ```python 块写到 `path`）：

```bash
/usr/bin/python3 - "$PWD/docs/superpowers/plans/2026-09-29-direct-cdc-backend.md" <<'EXTRACT'
import io, os, re, shutil, sys
plan, root = sys.argv[1], "/tmp/plancheck"
lines = io.open(plan, encoding="utf-8").read().split("\n")
shutil.rmtree(root, ignore_errors=True)
os.makedirs(f"{root}/src/litearm_lerobot", exist_ok=True)
os.makedirs(f"{root}/tests", exist_ok=True)
A = re.compile(r"(创建|追加|整个替换为|替换为)[^\n]*?`((?:src|tests)/[^`]+\.py)`")
B = re.compile(r"`((?:src|tests)/[^`]+\.py)`[^\n]*?(整个替换为|追加)")
last, written = None, []
for ln in lines:
    m = A.search(ln) or B.search(ln)
    if m:
        last = m.group(2) if m.re is A else m.group(1)
    if last and ln.strip() == "```python":
        j = i = lines.index(ln); j += 1; buf = []
        while lines[j].strip() != "```":
            buf.append(lines[j]); j += 1
        lines[i] = ""
        dest = os.path.join(root, last)
        mode = "a" if last in written else "w"
        with io.open(dest, mode, encoding="utf-8") as f:
            if mode == "a":
                f.write("\n\n")
            f.write("\n".join(buf) + "\n")
        written.append(last); lines[j] = ""; last = None
EXTRACT
# ⚠ 两个已知的抽取盲区，需要手补（脚本只认"创建/追加/替换为 + 反引号路径"）：
#   · tests/test_sdk_contract.py（该块无"创建 `path`"前缀）
#   · tests/test_safety.py 的两段"在 `...` 末尾追加："（语序不同）
#   · src/litearm_lerobot/utils.py 与 tests/test_utils.py 的另两条用例（计划说"不动"）
# 补齐后：
cp /home/llx/litearm-lerobot/src/litearm_lerobot/utils.py /tmp/plancheck/src/litearm_lerobot/
cd /tmp/plancheck && PYTHONPATH=/tmp/plancheck/src /usr/bin/python3 -m pytest -q
```

Expected: `86 passed`（⚠ 抽取树可能因盲区多算一条，见上方警告）。

- [ ] **Step 2: 确认没有残留的旧 API 引用**

```bash
grep -rn "endpoint\|arm_id\|query_timeout\|use_commander\|movej_speed\|settle_s\|\.hold()\|litearm-server\|zenoh" \
  src/ tests/ examples/ README.md README.zh-CN.md docs/DEVELOPER_GUIDE.md docs/DEVELOPER_GUIDE.zh-CN.md
```

Expected: **零命中**。任何命中都是没改干净。例外：本计划与 spec 文档可以命中
（它们是历史记录），所以上面这个命令**不含** `docs/superpowers/`。

- [ ] **Step 3: 确认提交历史干净**

```bash
git log --oneline main..HEAD
```

Expected: 一条 docs 提交 + 若干 feat/test/docs/build 提交，**没有** `Co-Authored-By`、
没有工具署名。逐条核对：

```bash
git log main..HEAD --format='%H%n%B' | grep -in "claude\|co-authored\|generated with" || echo "干净"
```

- [ ] **Step 4: 不要 push**

⛔ **本计划到此为止，不 push、不开 PR。** 分支留在本地，由仓库所有者决定。

- [ ] **Step 5: 报告**

给用户的报告必须包含：

1. 分支名与提交哈希列表
2. `pytest` 的**逐字输出**
3. 六个消失/改名的配置字段（破坏消费者提示）
4. 他需要做的两件事：
   - 把 §8.2 的台账条目落到 `/home/llx/litearm-stm32/tools/PARAM_STATE.md`
   - 真机验收（spec §11.2 六条）
5. **明说**：离线判据全绿 ≠ 真机可用。`0x08` 的两条锁存豁免、K/B 的手感、
   `actuator` 的 A/B —— 这三件都只能真机判。

---

## 自检清单（写完计划后逐条核对）

- [ ] 每个 Task 的文件路径都是**绝对可定位的**（相对仓根），不是"某个文件"
- [ ] 每个 Task 的代码块都是**可直接粘贴**的完整代码，不是"加个校验"
- [ ] 每个 Task 都以**提交**结尾，提交信息是 Conventional Commits
- [ ] 判据都有**判别力**：对拍判据拿原版当参照、负控判据证明判据非常真（`test_limit_margin_of_002_would_destroy_j4`、`test_reference_slew_is_actually_loaded`、`test_firmware_table_values_are_the_ones_we_think_they_are`）
- [ ] 没有推送、没有开 PR
- [ ] 真机验收单独列出，且明说它离线替代不了
