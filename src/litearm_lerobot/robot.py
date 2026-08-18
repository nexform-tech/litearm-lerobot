"""LeRobot-compatible driver for the LiteArm robotic arm.

Implements the abstract ``lerobot.robots.Robot`` interface on top of
litearm-python's remote ``Arm`` client. The LiteArm reports absolute joint
positions from the arm controller, so no homing or joint calibration is
needed — ``calibrate()`` is a no-op and ``is_calibrated`` is always True.
"""
from __future__ import annotations

import logging
import threading
from typing import Any, Dict, List, Optional

from lerobot.robots.robot import Robot

import litearm

from .config import LiteArmRobotConfig

log = logging.getLogger(__name__)


class _Commander:
    """Background thread that keeps moving the arm toward the latest target.

    ``Arm.movej`` is a *blocking* RPC: it returns only once the arm has
    physically settled. Running it in a thread lets ``send_action()`` return
    immediately while the arm continuously re-plans toward the newest goal —
    this is what makes a LeRobot teleop/record loop usable.
    """

    def __init__(
        self,
        arm: litearm.Arm,
        speed: float,
        settle_s: float,
        freq_hz: float,
    ) -> None:
        self._arm = arm
        self._speed = speed
        self._settle_s = settle_s
        self._period = 1.0 / max(float(freq_hz), 1.0)
        self._target: Optional[List[float]] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def set_target(self, q: List[float]) -> None:
        with self._lock:
            self._target = list(q)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="litearm-commander"
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None

    def _run(self) -> None:
        last_sent: Optional[tuple] = None
        while not self._stop.is_set():
            with self._lock:
                target = list(self._target) if self._target is not None else None
            key = tuple(target) if target is not None else None
            if key is not None and key != last_sent:
                try:
                    self._arm.movej(target, speed=self._speed, settle_s=self._settle_s)
                    last_sent = key
                except Exception:  # arm moved/stopped meanwhile: retry on next tick
                    last_sent = None
                    self._stop.wait(self._period)
            else:
                self._stop.wait(self._period)


class LiteArmRobot(Robot):
    """Drive a LiteArm through the LeRobot ``Robot`` interface.

    Observation / action vectors are the seven joint positions reported by the
    arm controller::

        observation = {"observation.state": [q0, q1, q2, q3, q4, q5, q6]}
        action      = {"action":                [q0, q1, q2, q3, q4, q5, q6]}

    By default a background commander thread turns each ``send_action`` into a
    non-blocking target update (set ``use_commander=False`` for blocking
    movej-per-step instead).
    """

    name = "litearm"
    config_class = LiteArmRobotConfig

    def __init__(self, config: LiteArmRobotConfig) -> None:
        super().__init__(config)
        self.config = config  # Robot.__init__ does not store the config
        self._arm: Optional[litearm.Arm] = None
        self._commander: Optional[_Commander] = None

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
        """Open a litearm-python connection and start the commander thread."""
        if self.is_connected:
            return
        arm = litearm.Arm(
            endpoint=self.config.endpoint,
            arm_id=self.config.arm_id,
            query_timeout=self.config.query_timeout,
        )
        # Warm the state broadcast cache so get_observation() can answer at once.
        arm.get_state()
        if calibrate:
            self.calibrate()
        if self.config.enable_on_connect:
            try:
                arm.enable()  # enable motors + hold current pose
            except Exception as exc:  # read-only servers are fine
                log.warning("enable() failed on connect: %s", exc)
        self._arm = arm
        if self.config.use_commander:
            self._commander = _Commander(
                arm,
                speed=self.config.movej_speed,
                settle_s=self.config.settle_s,
                freq_hz=self.config.control_frequency_hz,
            )
            self._commander.start()

    @property
    def is_calibrated(self) -> bool:
        # Absolute encoders are read directly from the arm controller: no homing.
        return True

    def calibrate(self) -> None:
        """No-op — the LiteArm reports absolute joint positions."""
        return None

    def configure(self) -> None:
        """No-op — tuning (gains, limits, ...) is handled by litearm-server."""
        return None

    # ── LeRobot observation / action ─────────────────────────────────────────

    def get_observation(self) -> Dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("LiteArmRobot is not connected")
        state = self._arm.get_state()
        if state is None:
            raise RuntimeError("No robot state received yet")
        return {"observation.state": [float(v) for v in state["q"]]}

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("LiteArmRobot is not connected")
        if "action" not in action:
            raise ValueError("action dict must contain an 'action' key")
        q = [float(v) for v in action["action"]]
        if len(q) != self.config.num_joints:
            raise ValueError(
                f"action must have {self.config.num_joints} joints, got {len(q)}"
            )
        if self._commander is not None:
            self._commander.set_target(q)  # non-blocking
        else:
            self._arm.movej(q, speed=self.config.movej_speed, settle_s=self.config.settle_s)
        return {"action": q}

    def disconnect(self) -> None:
        """Stop the commander and close the connection."""
        if self._commander is not None:
            self._commander.stop()
            self._commander = None
        if self._arm is not None:
            try:
                self._arm.hold()  # hold the current pose on exit
            except Exception:
                pass
            self._arm.close()
            self._arm = None

    def __str__(self) -> str:
        return f"{self.id} LiteArmRobot"
