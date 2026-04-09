"""Unit tests for SSC-32 command formatting (serial I/O is mocked)."""

from unittest.mock import MagicMock

import pytest
from assertpy import assert_that

import ssc32


@pytest.fixture
def serial_port(monkeypatch):
    mock_serial = MagicMock()
    mock_serial.write = MagicMock()
    mock_serial.flush = MagicMock()
    mock_serial.read = MagicMock(return_value=b".")
    monkeypatch.setattr(ssc32.serial, "Serial", lambda *a, **k: mock_serial)
    return mock_serial


def test_write_appends_cr_and_utf8(serial_port):
    controller = ssc32.SSC32("/dev/null")
    controller.write("VER")
    assert_that(serial_port.write.call_count).is_equal_to(1)
    written = serial_port.write.call_args[0][0]
    assert_that(written).is_equal_to(b"VER\r")
    assert_that(serial_port.flush.call_count).is_equal_to(1)


def test_move_emits_pulse_and_speed(serial_port):
    controller = ssc32.SSC32("/dev/null")
    controller.move(3, 1500, speed=100)
    payload = serial_port.write.call_args[0][0]
    assert_that(payload).contains(b"#3 P1500 S100")


def test_move_emits_time_in_ms(serial_port):
    controller = ssc32.SSC32("/dev/null")
    controller.move(1, 1000, time=0.5)
    payload = serial_port.write.call_args[0][0]
    assert_that(payload).contains(b"#1 P1000 T500")


def test_move_group_batches_commands(serial_port):
    controller = ssc32.SSC32("/dev/null")
    with controller.move_group():
        controller.move(0, 1500, speed=100)
        controller.move(1, 1600, speed=100)
    payload = serial_port.write.call_args[0][0]
    assert_that(payload.strip()).is_equal_to(b"#0 P1500 S100 #1 P1600 S100")


def test_move_done_queries_and_interprets_dot(serial_port):
    serial_port.read.return_value = b"."
    controller = ssc32.SSC32("/dev/null")
    assert_that(controller.move_done()).is_true()

    serial_port.read.return_value = b"x"
    assert_that(controller.move_done()).is_false()


def test_readline_reads_until_cr(serial_port):
    serial_port.read.side_effect = [b"V", b"E", b"R", b"\r"]
    controller = ssc32.SSC32("/dev/null")
    assert_that(controller.readline()).is_equal_to("VER")


def test_move_rejects_invalid_pulse(serial_port):
    controller = ssc32.SSC32("/dev/null")
    assert_that(controller.move).raises(AssertionError).when_called_with(0, 499)
    assert_that(controller.move).raises(AssertionError).when_called_with(0, 2501)
