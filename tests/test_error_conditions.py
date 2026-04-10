"""Tests for error conditions: timeouts, sensor conflicts, burst exhaustion."""

import pytest
from conftest import (
    FORWARD,
    STOP,
    STATE_FEEDING,
    STATE_ERROR,
    STATE_STOPPED,
    set_sensors,
    simulate_e_move,
)


class TestForwardTimeout:
    def test_timeout_triggers_error(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t

        # Start feeding
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.state == STATE_FEEDING

        # Set forward start time and advance past timeout
        enabled_buf.forward_start_time = t
        t += enabled_buf.forward_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert enabled_buf.state == STATE_ERROR
        assert "timeout" in enabled_buf.error_msg.lower()

    def test_timeout_triggers_pause(self, enabled_buf, reactor, gcode):
        enabled_buf.pause_on_runout = True
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t

        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        enabled_buf.forward_start_time = t
        t += enabled_buf.forward_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert "PAUSE" in gcode.scripts_run

    def test_direction_change_resets_forward_time(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        t = 10.0
        reactor._monotonic = t
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)

        # Stop motor
        enabled_buf._stop_motor()
        assert enabled_buf.forward_elapsed == 0.0


class TestSensorConflict:
    def test_empty_and_full_error(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True, full=True)
        reactor._monotonic = 10.0
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.state == STATE_ERROR
        assert "conflict" in enabled_buf.error_msg.lower()

    def test_error_stops_motor(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True, full=True)
        reactor._monotonic = 10.0
        enabled_buf.motor_direction = FORWARD
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.motor_direction == STOP


class TestErrorBlocking:
    def test_error_blocks_evaluate(self, enabled_buf, reactor):
        enabled_buf.state = STATE_ERROR
        enabled_buf.error_msg = "test error"
        set_sensors(enabled_buf, middle=True)
        enabled_buf.extruder_velocity = 10.0
        enabled_buf._evaluate_and_drive(10.0)
        # Motor should not start in error state
        assert enabled_buf.motor_direction == STOP

    def test_error_blocks_timer(self, enabled_buf, reactor):
        enabled_buf.state = STATE_ERROR
        t = 10.0
        reactor._monotonic = t
        ret = enabled_buf._control_timer_cb(t)
        # Timer should still return next interval
        assert ret == pytest.approx(t + enabled_buf.control_interval)
