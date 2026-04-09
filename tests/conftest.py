"""Shared pytest configuration and fixtures."""

import os
from pathlib import Path

import pytest


def pytest_collection_modifyitems(config, items):
    if os.environ.get("RUN_INTEGRATION_TESTS") != "1":
        skip_int = pytest.mark.skip(
            reason="set RUN_INTEGRATION_TESTS=1 to run hardware integration tests",
        )
        for item in items:
            if "integration" in item.keywords:
                item.add_marker(skip_int)
        return

    if os.environ.get("RUN_INTEGRATION_MOTION") != "1":
        skip_motion = pytest.mark.skip(
            reason="set RUN_INTEGRATION_MOTION=1 to run tests that move servos",
        )
        for item in items:
            if "motion" in item.keywords:
                item.add_marker(skip_motion)


@pytest.fixture(scope="session")
def integration_serial_port():
    """Device path from AL5D_SERIAL (default /dev/ttyUSB0). Skips if missing."""
    port = os.environ.get("AL5D_SERIAL", "/dev/ttyUSB0")
    if not Path(port).exists():
        pytest.skip(f"Serial device not found: {port}")
    return port


@pytest.fixture
def integration_ssc32(integration_serial_port):
    import ssc32

    ctrl = ssc32.SSC32(integration_serial_port)
    try:
        yield ctrl
    finally:
        ctrl.serial.close()


@pytest.fixture
def integration_arm(integration_serial_port):
    import al5d

    robot = al5d.AL5D(integration_serial_port)
    try:
        yield robot
    finally:
        robot.ssc32.serial.close()
