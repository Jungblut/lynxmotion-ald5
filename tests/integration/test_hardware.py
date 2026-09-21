"""
Hardware integration tests (real SSC-32 + AL5D).

These tests are skipped unless ``RUN_INTEGRATION_TESTS=1``.

Environment:

- ``AL5D_SERIAL`` — serial device (default ``/dev/ttyUSB0``)
- ``RUN_INTEGRATION_TESTS=1`` — enable integration tests
- ``RUN_INTEGRATION_MOTION=1`` — also run tests that move servos (default: motion tests skipped)

Serial-only tests (VER / ``move_done`` poll) do not move the arm. Tests marked ``@pytest.mark.motion``
call ``init()`` or move the gripper — use a clear workspace and stay clear of the arm.

Run::

    RUN_INTEGRATION_TESTS=1 uv run pytest tests/integration -v

Serial checks only (no servo motion)::

    RUN_INTEGRATION_TESTS=1 uv run pytest tests/integration -v -m "integration and not motion"

Full stack including motion::

    RUN_INTEGRATION_TESTS=1 RUN_INTEGRATION_MOTION=1 uv run pytest tests/integration -v
"""

import math

import pytest
from assertpy import assert_that

pytestmark = pytest.mark.integration


def test_ssc32_version_returns_non_empty_string(integration_ssc32):
    """Controller responds to VER with a firmware string."""
    ver = integration_ssc32.version()
    assert_that(ver).is_not_none()
    assert_that(ver.strip()).is_not_empty()


def test_ssc32_move_done_is_queryable(integration_ssc32):
    """Q command returns a byte; result is interpretable as done or not (API contract)."""
    done = integration_ssc32.move_done()
    assert_that(done).is_instance_of(bool)


@pytest.mark.motion
def test_al5d_init_reaches_idle(integration_arm):
    """Full pose reset: ``init()`` then wait until controller reports move complete."""
    integration_arm.init()
    integration_arm.wait_for_move()
    assert_that(integration_arm.move_done()).is_true()


@pytest.mark.motion
def test_al5d_gripper_small_change_and_wait(integration_arm):
    """Small gripper adjustment and wait (uses default ``init``-compatible mid closure)."""
    integration_arm.init()
    integration_arm.wait_for_move()
    integration_arm.gripper(48, speed=80)
    integration_arm.wait_for_move()
    integration_arm.gripper(50, speed=80)
    integration_arm.wait_for_move()
    assert_that(integration_arm.move_done()).is_true()


@pytest.mark.motion
def test_al5d_each_joint_sequential_ninety_deg_then_home(integration_arm):
    """Each joint moves by 90° (where API limits allow), returns to rest, then ``init()`` restores pose.

    Shoulder and wrist are limited to ±45° and ±60°; those steps use the largest safe positive angle.
    """
    arm = integration_arm
    half_pi = math.pi / 2
    speed = 80

    arm.init()
    arm.wait_for_move()

    # base: π/2 → π (+90°)
    arm.base(math.pi, speed=speed)
    arm.wait_for_move()
    arm.base(math.pi / 2, speed=speed)
    arm.wait_for_move()

    # shoulder: max +π/4 from 0 (not full 90°)
    arm.shoulder(math.pi / 4, speed=speed)
    arm.wait_for_move()
    arm.shoulder(0, speed=speed)
    arm.wait_for_move()

    # elbow: 0 → π/2 (+90°)
    arm.elbow(half_pi, speed=speed)
    arm.wait_for_move()
    arm.elbow(0, speed=speed)
    arm.wait_for_move()

    # wrist: max +π/3 from 0 (not full 90°)
    arm.wrist(math.pi / 3, speed=speed)
    arm.wait_for_move()
    arm.wrist(0, speed=speed)
    arm.wait_for_move()

    # wrist_rotate: 0 → π/2 (+90°)
    arm.wrist_rotate(half_pi, speed=speed)
    arm.wait_for_move()
    arm.wrist_rotate(0, speed=speed)
    arm.wait_for_move()

    arm.init()
    arm.wait_for_move()
    assert_that(arm.move_done()).is_true()
