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

from .safety import Limits, clamp_to_limits, slew_target

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

    def stop(self, timeout: float = 2.0) -> bool:
        """停线程并 join，返回**线程是否真的停了**。**幂等**。

        ⚠ 返回 `False` 时线程还在跑 —— 它可能正卡在自己的 `hold_at_current`
        里，而那条 `movej` 最长阻塞 `move_timeout`。收尾方**必须**看这个返回值：
        再发一条 `movej` 就是两条命令抢同一个 `(id, echo_cmd)` 应答队列。

        ⚠ `timeout` 是**每次** join 的上限，不是"总预算"；在没停下之前
        `self._thread` 保持非空，所以重复调用会如实重复报 `False`。
        """
        self._stop.set()
        th = self._thread
        if th is None:
            return True
        th.join(timeout=timeout)
        if th.is_alive():
            log.warning(
                "伺服线程 %.1f s 内没停下 —— 臂可能仍在伺服态。"
                "固件 0.1 s 看门狗会在断流后 fail-soft。", timeout)
            return False
        self._thread = None
        return True

    def set_target(self, q: Sequence[float]) -> None:
        """更新目标（非阻塞）。

        ⚠ 这里**也**过第一层软限位（spec §7.2）—— 限位不该只在 `send_action`
        那条路上成立，`set_target` 是公开 API（见 `__all__`）。这层是**防御
        纵深**，静默：告警由 `send_action` 发（那里看得见调用方的原始输入）。
        """
        clamped, _ = clamp_to_limits(q, self._limits)
        with self._lock:
            self._target = clamped

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
