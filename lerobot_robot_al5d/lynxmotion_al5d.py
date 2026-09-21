import logging
import math
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.robots import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

import al5d

from .config_lynxmotion_al5d import LynxmotionAL5DConfig

logger = logging.getLogger(__name__)

# Joint limits in radians, mirroring the asserts in al5d.AL5D. They are clipped in radians (with the
# driver's own expressions) so that a rounding error can never trip those asserts.
_RADIANS_LIMITS = {
    "base": (0.0, math.pi),
    "shoulder": (-math.pi / 4, math.pi / 4),
    "elbow": (0.0, math.pi * 7.0 / 8.0),
    "wrist": (-math.pi / 3, math.pi / 3),
    "wrist_rotate": (0.0, math.pi),
}
_GRIPPER_LIMITS = (0.0, 100.0)

JOINTS = ("base", "shoulder", "elbow", "wrist", "wrist_rotate", "gripper")

# Pose reached by al5d.AL5D.init(), in the units of the action space (degrees, gripper in percent).
HOME_POSE = {
    "base": 90.0,
    "shoulder": 0.0,
    "elbow": 0.0,
    "wrist": 0.0,
    "wrist_rotate": 0.0,
    "gripper": 50.0,
}


class LynxmotionAL5D(Robot):
    """Lynxmotion AL5D arm driven through an SSC-32(U) servo controller.

    Actions and observations are per-joint positions named ``<joint>.pos``. Joints are in degrees, using
    the joint conventions of :class:`al5d.AL5D` (e.g. base 90° faces away from the controller board, and
    ranges differ per joint, see ``al5d/al5d.py``), except ``gripper`` which is 0 (open) to 100 (closed).

    The SSC-32 drives hobby servos without feedback, so it cannot report where the arm actually is:
    observations are the last commanded (and clipped) target, not a measurement.
    """

    config_class = LynxmotionAL5DConfig
    name = "lynxmotion_al5d"

    def __init__(self, config: LynxmotionAL5DConfig):
        super().__init__(config)
        self.config = config
        # The driver opens the serial port when constructed, so it is only created in connect().
        self.arm: al5d.AL5D | None = None
        self.cameras = make_cameras_from_configs(config.cameras)
        self._last_target = dict(HOME_POSE)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in JOINTS}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return (
            self.arm is not None
            and self.arm.ssc32.serial.is_open
            and all(cam.is_connected for cam in self.cameras.values())
        )

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.arm = al5d.AL5D(self.config.port)
        if self.config.home_on_connect:
            logger.info(f"Homing {self}, this can take a while at low speeds.")
            self.arm.init()
            self.arm.wait_for_move()
            self._last_target = dict(HOME_POSE)

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        # Servo pulse widths are mapped to angles by fixed constants in the driver: nothing to calibrate.
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        obs: RobotObservation = {f"{joint}.pos": self._last_target[joint] for joint in JOINTS}

        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.async_read()

        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """Moves the joints named in ``action`` (all joints if it is complete), clipping to joint limits.

        Returns the action that was actually sent, i.e. after clipping.
        """
        targets = {key.removesuffix(".pos"): float(value) for key, value in action.items()}
        unknown = set(targets) - set(JOINTS)
        if unknown:
            raise ValueError(f"Unknown joints {sorted(unknown)}, expected a subset of {list(JOINTS)}")
        if not targets:
            return {}

        speed = self.config.speed
        sent: dict[str, float] = {}
        # A single grouped write: joints start together and it is one serial round trip at 9600 baud.
        with self.arm.ssc32.move_group():
            for joint, value in targets.items():
                if joint == "gripper":
                    sent[joint] = min(max(value, _GRIPPER_LIMITS[0]), _GRIPPER_LIMITS[1])
                    self.arm.gripper(sent[joint], speed=speed)
                else:
                    low, high = _RADIANS_LIMITS[joint]
                    radians = min(max(math.radians(value), low), high)
                    sent[joint] = math.degrees(radians)
                    getattr(self.arm, joint)(radians, speed=speed)

        self._last_target.update(sent)
        return {f"{joint}.pos": value for joint, value in sent.items()}

    @check_if_not_connected
    def disconnect(self) -> None:
        for cam in self.cameras.values():
            cam.disconnect()

        self.arm.ssc32.serial.close()
        self.arm = None
        logger.info(f"{self} disconnected.")
