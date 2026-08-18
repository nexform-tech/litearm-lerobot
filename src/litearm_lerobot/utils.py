"""Integration helpers for the LeRobot toolchain."""
from __future__ import annotations


def register() -> None:
    """Register ``litearm`` with LeRobot's robot factory.

    LeRobot builds robots through ``lerobot.robots.utils.make_robot_from_config``,
    an internal if/elif chain that has no branch for our driver. Calling
    ``register()`` patches that function so a config whose ``type == "litearm"``
    resolves to :class:`litearm_lerobot.LiteArmRobot`.

    This enables the standard LeRobot CLI against a LiteArm, e.g.::

        import litearm_lerobot
        litearm_lerobot.register()
        # then run your own entrypoint that parses a RobotConfig with robot.type: litearm

    Idempotent: repeated calls are safe.
    """
    from .config import LiteArmRobotConfig  # noqa: F401  (registers the draccus choice)

    import lerobot.robots.utils as robot_utils

    original = robot_utils.make_robot_from_config
    if getattr(original, "_litearm_registered", False):
        return

    def patched(config):
        if getattr(config, "type", None) == "litearm":
            from .robot import LiteArmRobot

            return LiteArmRobot(config)
        return original(config)

    patched._litearm_registered = True  # type: ignore[attr-defined]
    robot_utils.make_robot_from_config = patched
