"""Metal CAN arm adapter for lerobot's MotorsBusBase.

Wraps metal_sdk.MetalSDKInterface. The SDK .so links ROS2 C++ runtime libs,
so `import metal_sdk` is deferred into connect() / set_control_mode() (real
mode only); mock mode never touches the SDK, keeping this module importable
without ROS2/hardware (required: the lerobot CLI imports every registered
robot/teleop at startup).
"""

import logging
import math

from ..motors_bus import Motor, MotorCalibration, MotorNormMode, MotorsBusBase, Value

logger = logging.getLogger(__name__)

GRIPPER_MAX_MM = 80.0
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

DEFAULT_METAL_MOTORS: dict[str, Motor] = {
    "joint1": Motor(1, "metal-j", MotorNormMode.DEGREES),
    "joint2": Motor(2, "metal-j", MotorNormMode.DEGREES),
    "joint3": Motor(3, "metal-j", MotorNormMode.DEGREES),
    "joint4": Motor(4, "metal-j", MotorNormMode.DEGREES),
    "joint5": Motor(5, "metal-j", MotorNormMode.DEGREES),
    "joint6": Motor(6, "metal-j", MotorNormMode.DEGREES),
    "gripper": Motor(7, "metal-g", MotorNormMode.RANGE_0_100),
}


def gripper_mm_to_norm(mm: float) -> float:
    """metal gripper stroke (0-80 mm) -> lerobot RANGE_0_100."""
    return mm / GRIPPER_MAX_MM * 100.0


def gripper_norm_to_mm(norm: float) -> float:
    """lerobot RANGE_0_100 -> metal gripper stroke (0-80 mm)."""
    return norm / 100.0 * GRIPPER_MAX_MM


class _MockMetalSDK:
    """In-process stand-in for MetalSDKInterface. Speaks SDK-native units
    (radians, mm) so the bus's conversion logic is exercised under tests."""

    def __init__(self):
        self.joint_positions = [0.0] * 6  # radians
        self.gripper_mm = 0.0
        self.control_mode = None
        self.enabled = True
        self.commands: list = []
        self.get_position_calls = 0

    # Method names mirror the real MetalSDKInterface (pybind11) PascalCase API.
    def Init(self) -> bool:  # noqa: N802
        return True

    def GetJointNames(self):  # noqa: N802
        return JOINT_NAMES + ["gripper"]

    def GetJointPosition(self):  # noqa: N802
        self.get_position_calls += 1
        return list(self.joint_positions) + [self.gripper_mm]

    def SetArmJointPosition(self, positions, velocity_ratio=5):  # noqa: N802
        self.joint_positions = [float(p) for p in positions[:6]]
        self.commands.append(("joints", list(positions), velocity_ratio))

    def SetGripperStroke(self, stroke, velocity_ratio=5):  # noqa: N802
        self.gripper_mm = float(stroke)
        self.commands.append(("gripper", float(stroke), velocity_ratio))

    def SetArmControlMode(self, mode):  # noqa: N802
        self.control_mode = mode

    def SetEnableArm(self, flag):  # noqa: N802
        self.enabled = bool(flag)


class MetalMotorsBus(MotorsBusBase):
    """MotorsBusBase implementation over the metal CAN high-level SDK.

    Shared by MetalFollower (can0) and MetalLeader (can1). `port` carries the
    SocketCAN interface name ("can0"/"can1").
    """

    def __init__(
        self,
        port: str,
        motors: dict[str, Motor],
        calibration: dict[str, MotorCalibration] | None = None,
        *,
        urdf_path: str = "",
        arm_end_type: int = 1,
        velocity_ratio: int = 5,
        mock: bool = False,
    ):
        super().__init__(port, motors, calibration)
        self.urdf_path = urdf_path
        self.arm_end_type = arm_end_type
        self.velocity_ratio = velocity_ratio
        self.mock = mock
        self._sdk = None
        self._control_mode: str | None = None
        self._connected = False

    # --- lifecycle ---------------------------------------------------------
    def connect(self, handshake: bool = True) -> None:
        if self._connected:
            raise RuntimeError(f"{self.port}: already connected")
        if self.mock:
            self._sdk = _MockMetalSDK()
        else:
            # Deferred import: requires `source /opt/ros/humble/setup.bash`.
            from metal_sdk import MetalSDKInterface

            self._sdk = MetalSDKInterface(self.port, self.urdf_path, self.arm_end_type, True)
            if not self._sdk.Init():
                self._sdk = None
                raise ConnectionError(f"{self.port}: MetalSDKInterface.Init() failed")
        self._connected = True
        logger.info(f"MetalMotorsBus connected on {self.port} (mock={self.mock})")

    def disconnect(self, disable_torque: bool = True) -> None:
        if not self._connected:
            return
        if disable_torque and self._sdk is not None:
            self._sdk.SetEnableArm(False)
        self._sdk = None
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # --- control mode ------------------------------------------------------
    def set_control_mode(self, mode: str) -> None:
        if mode not in ("gravity", "nrt_joint"):
            raise ValueError(f"unknown control mode: {mode}")
        self._control_mode = mode
        if self.mock:
            return
        from metal_sdk import ControlMode

        self._sdk.SetArmControlMode(
            {
                "gravity": ControlMode.GRAVITY_COMPENSATION,
                "nrt_joint": ControlMode.NRT_JOINT_POSITION,
            }[mode]
        )

    # --- read/write --------------------------------------------------------
    def sync_read(
        self, data_name: str = "Present_Position", motors: str | list[str] | None = None
    ) -> dict[str, Value]:
        raw = self._sdk.GetJointPosition()  # [j1..j6 rad, gripper mm]
        full: dict[str, Value] = {name: math.degrees(raw[i]) for i, name in enumerate(JOINT_NAMES)}
        full["gripper"] = gripper_mm_to_norm(raw[6])
        if motors is None:
            return full
        if isinstance(motors, str):
            motors = [motors]
        return {m: full[m] for m in motors}

    def sync_write(self, data_name: str, values: dict[str, Value]) -> None:
        joint_vals = [values[j] for j in JOINT_NAMES if j in values]
        if len(joint_vals) == len(JOINT_NAMES):
            rads = [math.radians(v) for v in joint_vals]
            self._sdk.SetArmJointPosition(rads, self.velocity_ratio)
        if "gripper" in values:
            self._sdk.SetGripperStroke(gripper_norm_to_mm(values["gripper"]), self.velocity_ratio)

    def read(self, data_name: str, motor: str) -> Value:
        return self.sync_read(data_name, [motor])[motor]

    def write(self, data_name: str, motor: str, value: Value) -> None:
        # The metal SDK only accepts the full 6-joint vector, so a single-motor
        # write is read-modify-write: read the full pose, override one motor,
        # and re-issue the whole command.
        full = self.sync_read("Present_Position")
        full[motor] = value
        self.sync_write(data_name, full)

    # --- torque ------------------------------------------------------------
    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        self._sdk.SetEnableArm(True)

    def disable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        self._sdk.SetEnableArm(False)

    # --- calibration (no-op: 工业臂关节限位由固件保证) ----------------------
    def read_calibration(self) -> dict[str, MotorCalibration]:
        return self.calibration

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration], cache: bool = True) -> None:
        if cache:
            self.calibration = calibration_dict
