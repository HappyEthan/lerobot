"""Metal CAN arm adapter for lerobot's MotorsBusBase.

Wraps metal_sdk.MetalSDKInterface. The SDK .so links ROS2 C++ runtime libs,
so `import metal_sdk` is deferred into connect() / set_control_mode() (real
mode only); mock mode never touches the SDK, keeping this module importable
without ROS2/hardware (required: the lerobot CLI imports every registered
robot/teleop at startup).
"""

import logging
import math
import os
from pathlib import Path

from ..motors_bus import Motor, MotorCalibration, MotorNormMode, MotorsBusBase, Value

logger = logging.getLogger(__name__)

GRIPPER_MAX_MM = 80.0
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# arm_end_type semantics (sdk_manual.md): 0=none, 1=gripper, 2=teaching pendant,
# 3=gripper+pendant. GetJointPosition() returns 6 values for type 0 and 7 for
# types 1/2/3 (the with-gripper URDF), but a usable/actuated gripper exists only
# for types 1 and 3. Type 2's 7th value is a non-actuated slot we ignore.
GRIPPER_END_TYPES = frozenset({1, 3})
VALID_END_TYPES = (0, 1, 2, 3)
# Human-readable description of each end type, reused in error messages.
END_TYPE_HELP = "0=none(6 dof), 1=gripper(7 dof), 2=teaching pendant(7 dof), 3=gripper+pendant(7 dof)"


def validate_arm_end_type(arm_end_type: int) -> None:
    """Raise ValueError if arm_end_type is not a supported end-effector code."""
    if arm_end_type not in VALID_END_TYPES:
        raise ValueError(f"arm_end_type must be one of {END_TYPE_HELP}; got {arm_end_type!r}")


# URDFs are bundled next to this module so the integration is self-contained on a
# fresh clone. Override the directory with the METAL_URDF_DIR env var to point at
# your own URDFs (e.g. a copy with meshes). The bundled files are mesh-less, which
# is what the metal SDK uses for kinematics/gravity-compensation.
_BUNDLED_URDF_DIR = Path(__file__).resolve().parent / "urdf"
URDF_WITH_GRIPPER = str(_BUNDLED_URDF_DIR / "metal_with_gripper.urdf")
URDF_NO_GRIPPER = str(_BUNDLED_URDF_DIR / "metal_no_gripper.urdf")


def arm_has_gripper(arm_end_type: int) -> bool:
    """Whether a usable gripper DOF exists for this end type (types 1 and 3)."""
    return arm_end_type in GRIPPER_END_TYPES


def arm_position_dim(arm_end_type: int) -> int:
    """Length of GetJointPosition(): 6 for no end effector, 7 otherwise."""
    return 6 if arm_end_type == 0 else 7


def default_urdf(arm_end_type: int) -> str:
    """URDF paired with the end type (no_gripper only for type 0).

    Resolves to the bundled URDF, or to ``$METAL_URDF_DIR`` when that env var is
    set, so the path is portable across machines instead of hardcoded.
    """
    urdf_dir = Path(os.environ["METAL_URDF_DIR"]) if os.environ.get("METAL_URDF_DIR") else _BUNDLED_URDF_DIR
    name = "metal_no_gripper.urdf" if arm_end_type == 0 else "metal_with_gripper.urdf"
    return str(urdf_dir / name)


def metal_motors(arm_end_type: int = 1) -> dict[str, Motor]:
    """Motor schema for an end type: 6 joints, plus a gripper when usable."""
    motors: dict[str, Motor] = {
        name: Motor(i, "metal-j", MotorNormMode.DEGREES) for i, name in enumerate(JOINT_NAMES, start=1)
    }
    if arm_has_gripper(arm_end_type):
        motors["gripper"] = Motor(7, "metal-g", MotorNormMode.RANGE_0_100)
    return motors


# Convenience default for the gripper configuration (arm_end_type=1).
DEFAULT_METAL_MOTORS: dict[str, Motor] = metal_motors(1)


def gripper_mm_to_norm(mm: float) -> float:
    """metal gripper stroke (0-80 mm) -> lerobot RANGE_0_100."""
    return mm / GRIPPER_MAX_MM * 100.0


def gripper_norm_to_mm(norm: float) -> float:
    """lerobot RANGE_0_100 -> metal gripper stroke (0-80 mm)."""
    return norm / 100.0 * GRIPPER_MAX_MM


class _MockMetalSDK:
    """In-process stand-in for MetalSDKInterface. Speaks SDK-native units
    (radians, mm) so the bus's conversion logic is exercised under tests.
    `n_pos` mirrors GetJointPosition()'s length for the configured end type."""

    def __init__(self, n_pos: int = 7):
        self.n_pos = n_pos
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
        return JOINT_NAMES + (["gripper"] if self.n_pos == 7 else [])

    def GetJointPosition(self):  # noqa: N802
        self.get_position_calls += 1
        if self.n_pos == 7:
            return list(self.joint_positions) + [self.gripper_mm]
        return list(self.joint_positions)

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
        validate_arm_end_type(arm_end_type)
        self.urdf_path = urdf_path
        self.arm_end_type = arm_end_type
        self.velocity_ratio = velocity_ratio
        self.mock = mock
        self._has_gripper = arm_has_gripper(arm_end_type)
        self._pos_dim = arm_position_dim(arm_end_type)
        self._sdk = None
        self._control_mode: str | None = None
        self._connected = False

    # --- lifecycle ---------------------------------------------------------
    def connect(self, handshake: bool = True) -> None:
        if self._connected:
            raise RuntimeError(f"{self.port}: already connected")
        if self.mock:
            self._sdk = _MockMetalSDK(self._pos_dim)
        else:
            # Deferred import: requires `source /opt/ros/humble/setup.bash`.
            from metal_sdk import MetalSDKInterface

            self._sdk = MetalSDKInterface(self.port, self.urdf_path, self.arm_end_type, True)
            if not self._sdk.Init():
                self._sdk = None
                raise ConnectionError(f"{self.port}: MetalSDKInterface.Init() failed")
        self._verify_end_type()
        self._connected = True
        logger.info(f"MetalMotorsBus connected on {self.port} (mock={self.mock})")

    def _verify_end_type(self) -> None:
        """Fail fast when the configured arm_end_type does not match the hardware.

        The reported joint-position length is the ground truth coming off the CAN
        bus, so a wrong end-effector setting (e.g. arm_end_type=1 on an arm with no
        gripper) surfaces here as a clear error instead of a later IndexError.
        """
        actual = len(self._sdk.GetJointPosition())
        if actual != self._pos_dim:
            raise ValueError(
                f"{self.port}: arm_end_type={self.arm_end_type} expects GetJointPosition() to "
                f"return {self._pos_dim} values, but the arm reported {actual}. The configured end "
                f"effector does not match the hardware. Set arm_end_type to: {END_TYPE_HELP}."
            )

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
        raw = self._sdk.GetJointPosition()  # [j1..j6 rad] (+ gripper mm if present)
        if len(raw) < 6:
            raise ValueError(f"{self.port}: GetJointPosition() returned {len(raw)} values, expected >= 6")
        full: dict[str, Value] = {name: math.degrees(raw[i]) for i, name in enumerate(JOINT_NAMES)}
        if self._has_gripper:
            if len(raw) < 7:
                raise ValueError(
                    f"{self.port}: arm_end_type={self.arm_end_type} expects a gripper but "
                    f"GetJointPosition() returned only {len(raw)} values"
                )
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
        if self._has_gripper and "gripper" in values:
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

    # --- calibration (no-op: joint limits are enforced by the arm firmware) -
    def read_calibration(self) -> dict[str, MotorCalibration]:
        return self.calibration

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration], cache: bool = True) -> None:
        if cache:
            self.calibration = calibration_dict
