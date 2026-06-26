from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("metal_follower")
@dataclass
class MetalFollowerConfig(RobotConfig):
    can_id: str = "can0"
    # arm_end_type: 0=none, 1=gripper, 2=teaching pendant, 3=gripper+pendant.
    arm_end_type: int = 1
    # Empty -> auto-selected from arm_end_type (no_gripper URDF only for type 0).
    urdf_path: str = ""
    velocity_ratio: int = 5
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = None
    mock: bool = False
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    calibration: dict | None = None
