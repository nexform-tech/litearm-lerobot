"""litearm-lerobot: LeRobot driver for the LiteArm robotic arm.

Wraps litearm-python (remote client of litearm-server) behind the
LeRobot ``Robot`` interface so the arm can be driven with LeRobot-style
observation/action loops, recording and teleoperation.

Usage::

    from litearm_lerobot import LiteArmRobot, LiteArmRobotConfig

    robot = LiteArmRobot(LiteArmRobotConfig(endpoint="tcp/192.168.1.100:7447"))
    robot.connect()
    obs = robot.get_observation()          # {"observation.state": [7 floats]}
    robot.send_action({"action": [0.0] * 7})
    robot.disconnect()
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
