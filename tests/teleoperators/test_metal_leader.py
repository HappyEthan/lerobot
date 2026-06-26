import pytest

from lerobot.robots.metal_follower import MetalFollower, MetalFollowerConfig
from lerobot.teleoperators.metal_leader import MetalLeader, MetalLeaderConfig


@pytest.fixture
def leader():
    teleop = MetalLeader(MetalLeaderConfig(can_id="can1", mock=True))
    yield teleop
    if teleop.is_connected:
        teleop.disconnect()


def test_action_features_match_follower(leader):
    follower = MetalFollower(MetalFollowerConfig(mock=True))
    assert set(leader.action_features) == set(follower.action_features)
    assert leader.action_features == follower.action_features


def test_feedback_features_empty(leader):
    assert leader.feedback_features == {}


def test_connect_sets_gravity_mode(leader):
    leader.connect()
    assert leader.is_connected
    assert leader.bus._control_mode == "gravity"


def test_get_action_has_pos_suffix(leader):
    leader.connect()
    action = leader.get_action()
    assert set(action) == {f"joint{i}.pos" for i in range(1, 7)} | {"gripper.pos"}


def test_send_feedback_noop(leader):
    leader.connect()
    leader.send_feedback({})  # must not raise


def test_disconnect_keeps_torque(leader):
    leader.connect()
    sdk = leader.bus._sdk
    leader.disconnect()
    assert leader.is_connected is False
    assert sdk.enabled is True  # leader torque not disabled on disconnect


def test_factory_builds_metal_leader():
    from lerobot.teleoperators.utils import make_teleoperator_from_config

    teleop = make_teleoperator_from_config(MetalLeaderConfig(mock=True))
    assert teleop.__class__.__name__ == "MetalLeader"
