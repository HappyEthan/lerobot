import math

import pytest

from lerobot.motors import MotorNormMode
from lerobot.motors.metal import (
    DEFAULT_METAL_MOTORS,
    MetalMotorsBus,
    arm_has_gripper,
    arm_position_dim,
    default_urdf,
    metal_motors,
)


def make_bus(arm_end_type=1, **kw):
    return MetalMotorsBus(
        port="can0", motors=metal_motors(arm_end_type), arm_end_type=arm_end_type, mock=True, **kw
    )


def test_default_motors_schema():
    keys = list(DEFAULT_METAL_MOTORS)
    assert keys == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
    for j in keys[:6]:
        assert DEFAULT_METAL_MOTORS[j].norm_mode == MotorNormMode.DEGREES
    assert DEFAULT_METAL_MOTORS["gripper"].norm_mode == MotorNormMode.RANGE_0_100


def test_connect_disconnect_mock():
    bus = make_bus()
    assert bus.is_connected is False
    bus.connect()
    assert bus.is_connected is True
    bus.disconnect()
    assert bus.is_connected is False


def test_disconnect_disables_torque():
    bus = make_bus()
    bus.connect()
    bus.disconnect(disable_torque=True)
    assert bus._sdk is None  # released
    bus2 = make_bus()
    bus2.connect()
    sdk = bus2._sdk
    bus2.disconnect(disable_torque=False)
    assert sdk.enabled is True  # torque kept


def test_set_control_mode_records():
    bus = make_bus()
    bus.connect()
    bus.set_control_mode("nrt_joint")
    assert bus._control_mode == "nrt_joint"
    with pytest.raises(ValueError):
        bus.set_control_mode("bogus")


def test_is_connected_no_sdk_side_effect():
    bus = make_bus()
    bus.connect()
    bus._sdk.get_position_calls = 0
    _ = bus.is_connected
    assert bus._sdk.get_position_calls == 0


def test_sync_read_rad_to_deg_and_gripper():
    bus = make_bus()
    bus.connect()
    bus._sdk.joint_positions = [math.pi / 2, 0, 0, 0, 0, math.pi]
    bus._sdk.gripper_mm = 40.0
    obs = bus.sync_read("Present_Position")
    assert set(obs) == set(DEFAULT_METAL_MOTORS)
    assert obs["joint1"] == pytest.approx(90.0)
    assert obs["joint6"] == pytest.approx(180.0)
    assert obs["gripper"] == pytest.approx(50.0)  # 40mm / 80 * 100


def test_sync_write_deg_to_rad_and_writes_gripper():
    bus = make_bus()
    bus.connect()
    bus.sync_write(
        "Goal_Position",
        {
            "joint1": 90.0,
            "joint2": 0,
            "joint3": 0,
            "joint4": 0,
            "joint5": 0,
            "joint6": 180.0,
            "gripper": 50.0,
        },
    )
    assert bus._sdk.joint_positions[0] == pytest.approx(math.pi / 2)
    assert bus._sdk.joint_positions[5] == pytest.approx(math.pi)
    assert bus._sdk.gripper_mm == pytest.approx(40.0)
    kinds = [c[0] for c in bus._sdk.commands]
    assert "gripper" in kinds  # NRT path also writes gripper


def test_write_read_roundtrip():
    bus = make_bus()
    bus.connect()
    target = {
        "joint1": 10.0,
        "joint2": 20.0,
        "joint3": 30.0,
        "joint4": 40.0,
        "joint5": 50.0,
        "joint6": 60.0,
        "gripper": 25.0,
    }
    bus.sync_write("Goal_Position", target)
    back = bus.sync_read("Present_Position")
    for k, v in target.items():
        assert back[k] == pytest.approx(v)


def test_read_write_single():
    bus = make_bus()
    bus.connect()
    bus.write("Goal_Position", "joint1", 45.0)
    assert bus.read("Present_Position", "joint1") == pytest.approx(45.0)


def test_enable_disable_torque():
    bus = make_bus()
    bus.connect()
    bus.disable_torque()
    assert bus._sdk.enabled is False
    bus.enable_torque()
    assert bus._sdk.enabled is True


def test_read_calibration_returns_config():
    bus = MetalMotorsBus(port="can0", motors=DEFAULT_METAL_MOTORS, calibration={}, mock=True)
    assert bus.read_calibration() == {}
    bus.write_calibration({})  # no-op, must not raise


# --- arm_end_type variants -------------------------------------------------


def test_end_type_helpers():
    # gripper usable only for 1 and 3
    assert [arm_has_gripper(t) for t in (0, 1, 2, 3)] == [False, True, False, True]
    # GetJointPosition length: 6 for type 0, 7 otherwise
    assert [arm_position_dim(t) for t in (0, 1, 2, 3)] == [6, 7, 7, 7]
    assert default_urdf(0).endswith("metal_no_gripper.urdf")
    assert default_urdf(1).endswith("metal_with_gripper.urdf")
    assert default_urdf(2).endswith("metal_with_gripper.urdf")


def test_no_gripper_schema_and_read_write_type0():
    bus = make_bus(arm_end_type=0)  # no end effector -> 6 DOF
    assert set(metal_motors(0)) == {f"joint{i}" for i in range(1, 7)}
    bus.connect()
    bus._sdk.joint_positions = [math.pi / 2, 0, 0, 0, 0, 0]
    obs = bus.sync_read("Present_Position")
    assert set(obs) == {f"joint{i}" for i in range(1, 7)}  # no gripper key
    assert obs["joint1"] == pytest.approx(90.0)
    # writing must not attempt a gripper stroke
    bus.sync_write("Goal_Position", {f"joint{i}": 0.0 for i in range(1, 7)} | {"gripper": 50.0})
    assert all(c[0] != "gripper" for c in bus._sdk.commands)


def test_invalid_end_type_rejected():
    with pytest.raises(ValueError, match="arm_end_type"):
        MetalMotorsBus(port="can0", motors=metal_motors(1), arm_end_type=5, mock=True)


def test_connect_detects_wrong_end_type():
    # Configured as gripper (expects 7) but the arm only reports 6 -> wrong end effector.
    bus = make_bus(arm_end_type=1)
    bus.connect()
    bus._sdk.n_pos = 6  # simulate an arm that actually has no gripper
    with pytest.raises(ValueError, match="does not match the hardware"):
        bus._verify_end_type()


def test_pendant_type2_has_no_gripper_but_7dim():
    bus = make_bus(arm_end_type=2)  # teaching pendant: 7-dim vector, gripper unusable
    assert "gripper" not in metal_motors(2)
    bus.connect()
    assert len(bus._sdk.GetJointPosition()) == 7  # SDK still returns 7
    obs = bus.sync_read("Present_Position")
    assert "gripper" not in obs  # but bus does not expose it
    bus.sync_write("Goal_Position", {f"joint{i}": 0.0 for i in range(1, 7)} | {"gripper": 50.0})
    assert all(c[0] != "gripper" for c in bus._sdk.commands)
