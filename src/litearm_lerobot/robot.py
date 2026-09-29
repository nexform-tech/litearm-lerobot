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
        #: 本次会话是否**真的使能过**电机。`False` ⇒ 只读会话：没有伺服环，
        #: 也**不能**发 `movej`（未使能时同样被固件拒）。见 `connect()`。
        self._enabled = False

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

        ``enable_on_connect=False`` opens a genuine **observation-only** session:
        the motors are left disabled, nothing is ever commanded, and the servo
        loop is not started (reading ``get_observation()`` needs the firmware's
        passive status stream, not an enabled arm). ``send_action`` raises in
        that mode.

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
        # ⚠⚠ 只有**真的使能过**才起伺服环。
        #   `0x08`(joint_follow) / `0x03`(move_js) 的首帧在未使能时会被固件**整帧
        #   拒**（`litearm-stm32` `control/control_loop.c:966`：
        #   `if (!g_arm.enabled) return 0x03;`）⇒ 线程第一拍就抛，之后每个
        #   `get_observation()` / `send_action()` 都跟着炸。
        #   ⛔ 别再无条件 `start()`：那会让"只读会话"变成一个必定失败的会话。
        self._enabled = bool(self.config.enable_on_connect)
        if self._enabled:
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

    @property
    def is_enabled(self) -> bool:
        """本次会话是否使能过电机（即是否存在伺服环）。"""
        return self._enabled

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
        if self._servo is None:
            # ⛔ 不静默收下：没有伺服环 ⇒ 这条动作永远不会被执行。也**不**在这里
            #   偷偷 enable + 起环 —— 那正好绕开用户显式选的只读模式。
            raise RuntimeError(
                "LiteArmRobot 未使能（enable_on_connect=False）—— 伺服环没有启动，"
                "send_action 无处可发。要下发动作请用 enable_on_connect=True 重建"
                "会话（此时 connect() 会 arm.enable() 并起伺服环）；只读观测不需要"
                "使能，get_observation() 照常可用。"
            )
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
        enabled, self._enabled = self._enabled, False
        stopped = True
        if servo is not None:
            # ⚠ join 必须**盖住**交接：线程可能正卡在自己的 hold_at_current 里，
            #   而那条 movej 最长阻塞 `move_timeout`。2 s 的 join 会在那种情形下
            #   直接放弃，然后我们就在**同一个会话**上再发一条 movej —— 两条命令
            #   抢同一个 `(id, echo_cmd)` 应答队列、各自清掉对方的 ACK，而
            #   `close()` 落在飞行中 ⇒ 最坏是臂仍在伺服态而链路已关（0.1 s 后
            #   看门狗 fail-soft），正是"绝不 disable"这套设计要躲的结果。
            stopped = servo.stop(timeout=self.config.move_timeout + 2.0)
            if not stopped:
                log.error(
                    "伺服线程在 %.1f s 内没有停下 —— 臂**可能仍在伺服态**，"
                    "本函数不再重复接管（那会与它的 movej 抢应答队列）。"
                    "链路即将关闭，固件 0.1 s 看门狗会把它 fail-soft（下垂）。",
                    self.config.move_timeout + 2.0)
        if arm is None:
            return
        try:
            if self.config.disable_on_disconnect:
                arm.disable()
            elif not enabled:
                # 未使能 ⇒ `movej` 会被固件同一条 0x03 整帧拒，接管本就做不到。
                # 只读会话的臂从未被指挥过，也没什么可交还的。
                log.debug("只读会话（未使能）：跳过收尾接管")
            elif servo is not None and servo.error is not None:
                # ⚠ `error` 是**在接管之后**才置的（见 ServoLoop._run）⇒ 它非 None
                #   就是"伺服环已经 movej 交还过了"的信号。再接管一次 = 上面那条
                #   并发 movej 的坏结局。不重复交还。
                log.info("伺服环已自行受控接管（movej 回当前实测位姿），收尾不重复接管")
            elif not stopped:
                # ⚠ 上面那两条判据**都不覆盖**"线程还活着、正卡在自己的
                #   hold_at_current 里"这一刻（那时 `error` 还没置）。判据还是同一个：
                #   线程活着 ⇒ 本函数与它共用一条 `(id, echo_cmd)` 应答队列 ⇒
                #   不发第二条 movej。不接管只是让它 fail-soft（下垂），
                #   接管则是下垂 **加** 应答错乱。
                log.error("伺服线程未停 —— 跳过收尾接管（见上一条错误日志）")
            else:
                hold_at_current(arm)
        except Exception:                     # noqa: BLE001
            log.exception(
                "收尾接管失败 —— 链路即将关闭，臂会在 0.1 s 后 fail-soft（下垂）")
        finally:
            arm.close()

    def __str__(self) -> str:
        return f"{self.id} LiteArmRobot"
