import logging
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors.metal import DEFAULT_METAL_MOTORS, MetalMotorsBus
from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position

from .config_metal_follower import MetalFollowerConfig

logger = logging.getLogger(__name__)


class MetalFollower(Robot):
    """metal-arm follower (slave) driven by lerobot over CAN.

    can0, NRT_JOINT_POSITION control. 7-dim state (6 joints + gripper) plus the
    configured cameras; the same send_action() path serves teleop recording and
    policy inference.
    """

    config_class = MetalFollowerConfig
    name = "metal_follower"

    def __init__(self, config: MetalFollowerConfig):
        super().__init__(config)
        self.config = config
        self.bus = MetalMotorsBus(
            port=config.can_id,
            motors=dict(DEFAULT_METAL_MOTORS),
            calibration=config.calibration or {},
            urdf_path=config.urdf_path,
            arm_end_type=config.arm_end_type,
            velocity_ratio=config.velocity_ratio,
            mock=config.mock,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {c: (cfg.height, cfg.width, 3) for c, cfg in self.config.cameras.items()}

    @cached_property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(c.is_connected for c in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        self.bus.set_control_mode("nrt_joint")
        for cam in self.cameras.values():
            cam.connect()
        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_observation(self) -> dict:
        obs = self.bus.sync_read("Present_Position")
        obs = {f"{m}.pos": v for m, v in obs.items()}
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.async_read()
        return obs

    def send_action(self, action: dict) -> dict:
        goal = {k.removesuffix(".pos"): v for k, v in action.items() if k.endswith(".pos")}
        if self.config.max_relative_target is not None:
            present = self.bus.sync_read("Present_Position")
            goal_present = {k: (g, present[k]) for k, g in goal.items()}
            goal = ensure_safe_goal_position(goal_present, self.config.max_relative_target)
        self.bus.sync_write("Goal_Position", goal)
        return {f"{m}.pos": v for m, v in goal.items()}

    def disconnect(self) -> None:
        self.bus.disconnect(disable_torque=self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()
