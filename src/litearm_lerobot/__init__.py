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
