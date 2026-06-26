import logging

from lerobot.motors.metal import DEFAULT_METAL_MOTORS, MetalMotorsBus
from lerobot.teleoperators.teleoperator import Teleoperator

from .config_metal_leader import MetalLeaderConfig

logger = logging.getLogger(__name__)


class MetalLeader(Teleoperator):
    """metal-arm leader (master) read by lerobot over CAN.

    can1, GRAVITY_COMPENSATION so the human can move it freely. get_action()
    returns the 7-dim joint command whose schema matches MetalFollower's
    action_features exactly.
    """

    config_class = MetalLeaderConfig
    name = "metal_leader"

    def __init__(self, config: MetalLeaderConfig):
        super().__init__(config)
        self.config = config
        self.bus = MetalMotorsBus(
            port=config.can_id,
            motors=dict(DEFAULT_METAL_MOTORS),
            calibration=config.calibration or {},
            urdf_path=config.urdf_path,
            arm_end_type=config.arm_end_type,
            mock=config.mock,
        )

    @property
    def action_features(self) -> dict:
        return {f"{m}.pos": float for m in self.bus.motors}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        self.bus.set_control_mode("gravity")
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict:
        action = self.bus.sync_read("Present_Position")
        return {f"{m}.pos": v for m, v in action.items()}

    def send_feedback(self, feedback: dict) -> None:
        pass

    def disconnect(self) -> None:
        self.bus.disconnect(disable_torque=False)
