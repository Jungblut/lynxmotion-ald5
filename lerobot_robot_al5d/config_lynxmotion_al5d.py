from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("lynxmotion_al5d")
@dataclass
class LynxmotionAL5DConfig(RobotConfig):
    # Serial device of the SSC-32(U) servo controller.
    port: str = "/dev/ttyUSB0"
    # Servo speed in µs of pulse width per second (SSC-32 "S" parameter), applied to every joint of
    # every action. 1000 µs/s is roughly 90°/s; lower is slower and safer. The default matches the
    # hardware integration tests.
    speed: int = 80
    # The SSC-32 cannot report servo positions, so observations are the last commanded targets.
    # Homing on connect makes that state accurate from the first observation.
    home_on_connect: bool = True
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
