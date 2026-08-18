"""LeRobot config for the LiteArm robotic arm."""
from __future__ import annotations

from dataclasses import dataclass

from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("litearm")
@dataclass(kw_only=True)
class LiteArmRobotConfig(RobotConfig):
    # -- LiteArm connection ---------------------------------------------------
    #: Zenoh endpoint of the litearm-server (e.g. "tcp/192.168.1.100:7447").
    endpoint: str = "tcp/127.0.0.1:7447"
    #: Arm identifier; must match the server-side setting.
    arm_id: str = "armA"
    #: Per-call RPC timeout in seconds (None = never time out).
    query_timeout: float | None = None

    # -- Control --------------------------------------------------------------
    #: Number of arm joints (fixed at 7 for the LiteArm).
    num_joints: int = 7
    #: Desired control frequency in Hz (used by the commander thread).
    control_frequency_hz: float = 30.0
    #: movej speed (0..1) applied to every commanded target.
    movej_speed: float = 0.5
    #: Seconds to wait for the arm to settle after each movej.
    settle_s: float = 0.2
    #: Run a background commander thread so send_action() returns immediately
    #: while the arm keeps tracking the latest goal. Set False for blocking
    #: movej on every send_action().
    use_commander: bool = True
    #: Enable motors and hold the current pose on connect().
    enable_on_connect: bool = True
