"""Tests for AL5D joint mapping and inverse kinematics (hardware mocked)."""

import math
from unittest.mock import MagicMock, call, patch

import pytest
from assertpy import assert_that

import al5d


@pytest.fixture
def arm():
    mock_ssc32 = MagicMock()
    mock_ssc32.move_group.return_value = mock_ssc32
    mock_ssc32.__enter__ = MagicMock(return_value=mock_ssc32)
    mock_ssc32.__exit__ = MagicMock(return_value=False)
    with patch("ssc32.SSC32", return_value=mock_ssc32):
        robot = al5d.AL5D("/dev/null")
    return robot, mock_ssc32


def test_default_kinematics():
    assert al5d.Kinematics() == al5d.DEFAULT_KINEMATICS


def test_custom_kinematics_passed_to_al5d():
    custom = al5d.Kinematics(shoulder_height=0.05, elbow_wrist_length=0.2)
    mock_ssc32 = MagicMock()
    mock_ssc32.move_group.return_value = mock_ssc32
    with patch("ssc32.SSC32", return_value=mock_ssc32):
        robot = al5d.AL5D("/dev/null", kinematics=custom)
    assert robot.kinematics is custom


def test_base_maps_endpoints(arm):
    robot, ssc = arm
    robot.base(0)
    assert_that(ssc.move.call_args).is_equal_to(call(al5d.BASE, 2300, 100, None))
    ssc.move.reset_mock()
    robot.base(math.pi)
    assert_that(ssc.move.call_args).is_equal_to(call(al5d.BASE, 500, 100, None))


def test_shoulder_mid_angle(arm):
    robot, ssc = arm
    robot.shoulder(0)
    assert_that(ssc.move.call_args).is_equal_to(call(al5d.SHOULDER, 1500, 100, None))


def test_elbow_and_wrist_ranges(arm):
    robot, ssc = arm
    robot.elbow(0)
    assert_that(ssc.move.call_args).is_equal_to(call(al5d.ELBOW, 660, 100, None))
    ssc.move.reset_mock()
    robot.wrist(0)
    assert_that(ssc.move.call_args).is_equal_to(call(al5d.WRIST, 1550, 100, None))


def test_gripper_endpoints(arm):
    robot, ssc = arm
    robot.gripper(0)
    assert_that(ssc.move.call_args).is_equal_to(call(al5d.GRIPPER, 1100, 100, None))
    ssc.move.reset_mock()
    robot.gripper(100)
    assert_that(ssc.move.call_args).is_equal_to(call(al5d.GRIPPER, 2000, 100, None))


def test_gripper_percent_invalid(arm):
    robot, _ = arm
    assert_that(robot.gripper).raises(AssertionError).when_called_with(-1)
    assert_that(robot.gripper).raises(AssertionError).when_called_with(101)


def test_move_inverse_kinematics_calls_joint_moves(arm):
    robot, ssc = arm
    robot.move(0.15, 0.0, 0.22, 0.0)
    assert_that(ssc.move.call_count).is_greater_than_or_equal_to(3)
    servos = [c.args[0] for c in ssc.move.call_args_list]
    assert_that(servos).contains(al5d.ELBOW, al5d.SHOULDER, al5d.WRIST)


def test_init_sets_pose_via_move_group(arm):
    robot, ssc = arm
    robot.init()
    assert_that(ssc.move_group.call_count).is_greater_than(0)
    assert_that(ssc.move.call_count).is_greater_than_or_equal_to(6)
