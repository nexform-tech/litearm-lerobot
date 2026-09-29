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
