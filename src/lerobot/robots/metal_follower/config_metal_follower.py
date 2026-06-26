from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig

DEFAULT_URDF = "/home/ethan/makermods/metal-python-ros/metal_sdk/example/urdf/metal_with_gripper.urdf"


@RobotConfig.register_subclass("metal_follower")
@dataclass
class MetalFollowerConfig(RobotConfig):
    can_id: str = "can0"
    urdf_path: str = DEFAULT_URDF
    arm_end_type: int = 1
    velocity_ratio: int = 5
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = None
    mock: bool = False
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    calibration: dict | None = None
