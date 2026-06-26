from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

DEFAULT_URDF = "/home/ethan/makermods/metal-python-ros/metal_sdk/example/urdf/metal_with_gripper.urdf"


@TeleoperatorConfig.register_subclass("metal_leader")
@dataclass
class MetalLeaderConfig(TeleoperatorConfig):
    can_id: str = "can1"
    urdf_path: str = DEFAULT_URDF
    arm_end_type: int = 1
    mock: bool = False
    calibration: dict | None = None
