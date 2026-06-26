import pytest

from lerobot.robots.metal_follower import MetalFollower, MetalFollowerConfig


@pytest.fixture
def follower():
    cfg = MetalFollowerConfig(can_id="can0", mock=True)
    robot = MetalFollower(cfg)
    yield robot
    if robot.is_connected:
        robot.disconnect()


def test_features(follower):
    assert set(follower.action_features) == {f"joint{i}.pos" for i in range(1, 7)} | {"gripper.pos"}
    # no cameras -> observation equals motor features
    assert set(follower.observation_features) == set(follower.action_features)


def test_connect_sets_nrt_mode(follower):
    follower.connect()
    assert follower.is_connected
    assert follower.bus._control_mode == "nrt_joint"


def test_get_observation_has_pos_suffix(follower):
    follower.connect()
    obs = follower.get_observation()
    assert set(obs) == {f"joint{i}.pos" for i in range(1, 7)} | {"gripper.pos"}


def test_send_action_roundtrip(follower):
    follower.connect()
    action = {f"joint{i}.pos": float(i * 5) for i in range(1, 7)} | {"gripper.pos": 30.0}
    sent = follower.send_action(action)
    assert sent == action


def test_send_action_respects_max_relative_target():
    cfg = MetalFollowerConfig(can_id="can0", mock=True, max_relative_target=1.0)
    robot = MetalFollower(cfg)
    robot.connect()
    # present pose is 0; ask for +10 deg on each joint, expect clamp to +1
    action = {f"joint{i}.pos": 10.0 for i in range(1, 7)} | {"gripper.pos": 10.0}
    sent = robot.send_action(action)
    assert sent["joint1.pos"] == pytest.approx(1.0)
    robot.disconnect()


def test_disconnect(follower):
    follower.connect()
    follower.disconnect()
    assert follower.is_connected is False


def test_factory_builds_metal_follower():
    from lerobot.robots.utils import make_robot_from_config

    robot = make_robot_from_config(MetalFollowerConfig(mock=True))
    assert robot.__class__.__name__ == "MetalFollower"
