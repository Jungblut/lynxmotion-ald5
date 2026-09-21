"""
Hardware integration tests for the LeRobot bindings (real SSC-32 + AL5D).

Same switches as ``test_hardware.py``: skipped unless ``RUN_INTEGRATION_TESTS=1``, and tests marked
``motion`` also need ``RUN_INTEGRATION_MOTION=1``.

Serial checks only (no servo motion)::

    RUN_INTEGRATION_TESTS=1 uv run pytest tests/integration/test_lerobot_hardware.py -v -m "integration and not motion"

Full stack including motion (clear the workspace first)::

    RUN_INTEGRATION_TESTS=1 RUN_INTEGRATION_MOTION=1 uv run pytest tests/integration/test_lerobot_hardware.py -v
"""

import pytest
from assertpy import assert_that

pytestmark = pytest.mark.integration


def test_lerobot_connect_without_homing_and_disconnect(integration_lerobot_al5d):
    """Opens and closes the port through the LeRobot API without sending any servo command."""
    robot = integration_lerobot_al5d(home_on_connect=False)

    robot.connect()
    assert_that(robot.is_connected).is_true()
    assert_that(robot.get_observation().keys()).is_equal_to(robot.observation_features.keys())

    robot.disconnect()
    assert_that(robot.is_connected).is_false()


@pytest.mark.motion
def test_lerobot_connect_homes_then_small_moves_and_returns(integration_lerobot_al5d):
    """``connect()`` homes the arm, then base and gripper move by a few degrees and back."""
    robot = integration_lerobot_al5d()
    robot.connect()
    home = robot.get_observation()

    sent = robot.send_action({"base.pos": 100.0, "gripper.pos": 55.0})
    robot.arm.wait_for_move()
    assert_that(sent).is_equal_to({"base.pos": pytest.approx(100.0), "gripper.pos": 55.0})
    obs = robot.get_observation()
    assert_that(obs["base.pos"]).is_close_to(100.0, 1e-6)
    assert_that(obs["gripper.pos"]).is_equal_to(55.0)

    robot.send_action(home)
    robot.arm.wait_for_move()
    assert_that(robot.arm.move_done()).is_true()
    assert_that(robot.get_observation()).is_equal_to(home)
