from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.motors.metal import validate_arm_end_type
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
    # Gripper actuator is slower than the joints; give it a faster ratio (1-10) so
    # it keeps up during teleoperation. Lower it if the gripper snaps too hard.
    gripper_velocity_ratio: int = 10
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = None
    mock: bool = False
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    calibration: dict | None = None

    def __post_init__(self):
        super().__post_init__()
        validate_arm_end_type(self.arm_end_type)
