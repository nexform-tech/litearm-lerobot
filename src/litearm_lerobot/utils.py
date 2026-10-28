"""Integration helpers for the LeRobot toolchain."""
from __future__ import annotations


def register() -> None:
    """Register both adapters with LeRobot's robot factory.

    LeRobot builds robots through ``lerobot.robots.utils.make_robot_from_config``,
    an internal if/elif chain that has no branch for our drivers. Calling
    ``register()`` patches that function so ``litearm`` resolves to
    :class:`litearm_lerobot.LiteArmRobot` and ``litearm_grip_camera`` resolves
    to :class:`litearm_lerobot.LiteArmGripCameraRobot`.

    This enables the standard LeRobot CLI against a LiteArm, e.g.::

        import litearm_lerobot
        litearm_lerobot.register()
        # then run your own entrypoint that parses a RobotConfig with robot.type: litearm

    Idempotent: repeated calls are safe.
    """
    from .config import LiteArmRobotConfig  # noqa: F401  (registers the draccus choice)
    from .grip_camera_robot import LiteArmGripCameraRobotConfig  # noqa: F401

    import lerobot.robots.utils as robot_utils

    original = robot_utils.make_robot_from_config
    if getattr(original, "_litearm_registered", False):
        return

    def patched(config):
        if getattr(config, "type", None) == "litearm":
            from .robot import LiteArmRobot

            return LiteArmRobot(config)
        if getattr(config, "type", None) == "litearm_grip_camera":
            from .grip_camera_robot import LiteArmGripCameraRobot

            return LiteArmGripCameraRobot(config)
        return original(config)

    patched._litearm_registered = True  # type: ignore[attr-defined]
    patched._litearm_grip_camera_registered = True  # type: ignore[attr-defined]
    robot_utils.make_robot_from_config = patched
