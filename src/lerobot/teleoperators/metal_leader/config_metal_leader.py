from dataclasses import dataclass

from lerobot.motors.metal import validate_arm_end_type
from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("metal_leader")
@dataclass
class MetalLeaderConfig(TeleoperatorConfig):
    can_id: str = "can1"
    # arm_end_type: 0=none, 1=gripper, 2=teaching pendant, 3=gripper+pendant.
    # Leader and follower must agree on gripper presence (1/3) so their
    # action_features schemas match for lerobot record/replay.
    arm_end_type: int = 1
    # Empty -> auto-selected from arm_end_type (no_gripper URDF only for type 0).
    urdf_path: str = ""
    mock: bool = False
    calibration: dict | None = None

    def __post_init__(self):
        # TeleoperatorConfig has no __post_init__, so do not call super().
        validate_arm_end_type(self.arm_end_type)
