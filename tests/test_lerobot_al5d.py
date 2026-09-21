"""Tests for the LeRobot bindings (real al5d driver, SSC-32 serial mocked)."""

from unittest.mock import MagicMock, patch

import pytest
from assertpy import assert_that

pytest.importorskip("lerobot")

from lerobot.robots import RobotConfig, make_robot_from_config
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.import_utils import register_third_party_plugins

import al5d
from lerobot_robot_al5d import LynxmotionAL5D, LynxmotionAL5DConfig
from lerobot_robot_al5d.lynxmotion_al5d import HOME_POSE, JOINTS

SERVOS = {
    "base": al5d.BASE,
    "shoulder": al5d.SHOULDER,
    "elbow": al5d.ELBOW,
    "wrist": al5d.WRIST,
    "wrist_rotate": al5d.WRIST_ROTATE,
    "gripper": al5d.GRIPPER,
}


@pytest.fixture
def ssc():
    mock = MagicMock()
    mock.move_group.return_value = mock
    mock.__enter__ = MagicMock(return_value=mock)
    mock.__exit__ = MagicMock(return_value=False)
    mock.move_done.return_value = True
    mock.serial.is_open = True
    mock.serial.close.side_effect = lambda: setattr(mock.serial, "is_open", False)
    return mock


@pytest.fixture
def config(tmp_path):
    return LynxmotionAL5DConfig(port="/dev/null", speed=250, calibration_dir=tmp_path)


@pytest.fixture
def robot(config, ssc):
    with patch("al5d.ssc32.SSC32", return_value=ssc):
        robot = LynxmotionAL5D(config)
        robot.connect()
        ssc.move.reset_mock()
        ssc.__enter__.reset_mock()
        yield robot
        if robot.is_connected:
            robot.disconnect()


def pulses(ssc):
    """servo -> pulse width of the moves recorded on the mocked controller."""
    return {c.args[0]: c.args[1] for c in ssc.move.call_args_list}


def test_features_available_before_connect(config):
    robot = LynxmotionAL5D(config)
    expected = {f"{joint}.pos": float for joint in JOINTS}
    assert_that(robot.action_features).is_equal_to(expected)
    assert_that(robot.observation_features).is_equal_to(expected)
    assert_that(robot.is_connected).is_false()
    assert_that(robot.is_calibrated).is_true()


def test_connect_homes_the_arm_to_home_pose(config, ssc):
    with patch("al5d.ssc32.SSC32", return_value=ssc):
        robot = LynxmotionAL5D(config)
        robot.connect()
        homed = pulses(ssc)
        ssc.move.reset_mock()
        robot.send_action({f"{joint}.pos": value for joint, value in HOME_POSE.items()})
        # HOME_POSE must stay in sync with al5d.AL5D.init()
        assert_that(pulses(ssc)).is_equal_to(homed)
        assert_that(robot.get_observation()).is_equal_to({f"{j}.pos": v for j, v in HOME_POSE.items()})


def test_connect_can_skip_homing(config, ssc):
    config.home_on_connect = False
    with patch("al5d.ssc32.SSC32", return_value=ssc):
        robot = LynxmotionAL5D(config)
        robot.connect()
    ssc.move.assert_not_called()
    assert_that(robot.is_connected).is_true()


def test_send_action_maps_degrees_to_pulses_with_speed_in_one_group(robot, ssc):
    sent = robot.send_action({"base.pos": 90.0, "gripper.pos": 100.0})

    assert_that(sent).is_equal_to({"base.pos": 90.0, "gripper.pos": 100.0})
    assert_that(pulses(ssc)).is_equal_to({al5d.BASE: 1400, al5d.GRIPPER: 2000})
    assert_that({c.args[2] for c in ssc.move.call_args_list}).is_equal_to({250})
    assert_that(ssc.__enter__.call_count).is_equal_to(1)


def test_send_action_clips_to_joint_limits_without_tripping_driver_asserts(robot, ssc):
    high = robot.send_action({f"{joint}.pos": 1000.0 for joint in JOINTS})
    low = robot.send_action({f"{joint}.pos": -1000.0 for joint in JOINTS})

    assert_that(high).is_equal_to(
        {
            "base.pos": pytest.approx(180.0),
            "shoulder.pos": pytest.approx(45.0),
            "elbow.pos": pytest.approx(157.5),
            "wrist.pos": pytest.approx(60.0),
            "wrist_rotate.pos": pytest.approx(180.0),
            "gripper.pos": 100.0,
        }
    )
    assert_that(low).is_equal_to(
        {
            "base.pos": 0.0,
            "shoulder.pos": pytest.approx(-45.0),
            "elbow.pos": 0.0,
            "wrist.pos": pytest.approx(-60.0),
            "wrist_rotate.pos": 0.0,
            "gripper.pos": 0.0,
        }
    )


def test_partial_action_only_moves_named_joints(robot, ssc):
    robot.send_action({"elbow.pos": 45.0})

    assert_that(pulses(ssc)).is_equal_to({al5d.ELBOW: 660 + int(1560 * 0.25)})
    obs = robot.get_observation()
    assert_that(obs["elbow.pos"]).is_close_to(45.0, 1e-9)
    assert_that(obs["base.pos"]).is_equal_to(HOME_POSE["base"])


def test_observation_is_last_clipped_target(robot):
    robot.send_action({"shoulder.pos": 90.0})
    assert_that(robot.get_observation()["shoulder.pos"]).is_close_to(45.0, 1e-9)


def test_send_action_rejects_unknown_joint(robot, ssc):
    with pytest.raises(ValueError, match="nose"):
        robot.send_action({"nose.pos": 1.0})
    ssc.move.assert_not_called()


def test_empty_action_sends_nothing(robot, ssc):
    assert_that(robot.send_action({})).is_empty()
    ssc.move.assert_not_called()


def test_io_requires_connection(config):
    robot = LynxmotionAL5D(config)
    with pytest.raises(DeviceNotConnectedError):
        robot.get_observation()
    with pytest.raises(DeviceNotConnectedError):
        robot.send_action({"base.pos": 90.0})
    with pytest.raises(DeviceNotConnectedError):
        robot.disconnect()


def test_connect_twice_raises(robot):
    with pytest.raises(DeviceAlreadyConnectedError):
        robot.connect()


def test_disconnect_closes_serial(robot, ssc):
    robot.disconnect()
    assert_that(robot.is_connected).is_false()
    ssc.serial.close.assert_called_once()


def test_registered_with_lerobot_factory(config, ssc):
    assert_that(RobotConfig.get_choice_class("lynxmotion_al5d")).is_same_as(LynxmotionAL5DConfig)
    assert_that(config.type).is_equal_to("lynxmotion_al5d")
    assert_that(make_robot_from_config(config)).is_instance_of(LynxmotionAL5D)


def test_plugin_is_discovered_by_distribution_name():
    """lerobot imports installed distributions named lerobot_robot_*, so the name must not change."""
    with patch("importlib.import_module") as import_module:
        register_third_party_plugins()
    imported = [c.args[0] for c in import_module.call_args_list]
    assert_that(imported).contains("lerobot_robot_al5d")
