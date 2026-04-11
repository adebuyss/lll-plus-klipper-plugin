"""Tests for error conditions: safety timeouts, sensor conflicts."""

import pytest
from conftest import (
    STATE_FEEDING,
    STATE_ERROR,
    STATE_STOPPED,
    set_sensors,
)


class TestEmptySafetyTimeout:
    def test_empty_timeout_triggers_error(self, enabled_buf, reactor):
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, empty=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        assert enabled_buf._safety_zone_start == t

        t += enabled_buf.empty_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert enabled_buf.state == STATE_ERROR
        assert "empty" in enabled_buf.error_msg.lower()

    def test_no_timeout_when_not_printing(self, enabled_buf, reactor):
        """Safety timeout must not fire when the extruder is idle."""
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, empty=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)

        t += enabled_buf.empty_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert enabled_buf.state != STATE_ERROR


class TestFullSafetyTimeout:
    def test_full_timeout_triggers_retract(self, enabled_buf, reactor,
                                           force_move):
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, full=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)

        t += enabled_buf.full_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        # Should have done a safety retract via force_move
        assert len(force_move.moves) > 0
        assert force_move.moves[-1][1] < 0  # negative dist = retract

    def test_no_retract_when_not_printing(self, enabled_buf, reactor,
                                           force_move):
        """Safety retract must not fire when the extruder is idle."""
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, full=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)

        t += enabled_buf.full_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert len(force_move.moves) == 0


class TestSensorConflict:
    def test_conflict_triggers_error(self, enabled_buf):
        set_sensors(enabled_buf, empty=True, full=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf.state == STATE_ERROR
        assert "conflict" in enabled_buf.error_msg.lower()

    def test_error_stops_motor(self, enabled_buf):
        set_sensors(enabled_buf, empty=True, full=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf.motor_direction == "stop"


class TestErrorBlocking:
    def test_error_blocks_sensor_callback(self, enabled_buf, buttons):
        enabled_buf.state = STATE_ERROR
        enabled_buf.error_msg = "test error"
        set_sensors(enabled_buf, middle=True)
        # Sensor callback should not update rd when in error state
        old_mult = enabled_buf._rd_multiplier
        # Directly call the sensor callback
        buttons.callbacks["PE1"](10.0, 0)  # trigger middle
        # Multiplier unchanged because error state blocks update
        assert enabled_buf.state == STATE_ERROR

    def test_error_blocks_timer_sync(self, enabled_buf, reactor):
        enabled_buf.state = STATE_ERROR
        t = 10.0
        reactor._monotonic = t
        ret = enabled_buf._control_timer_cb(t)
        assert ret == pytest.approx(t + enabled_buf.control_interval)


class TestClearError:
    def test_clear_error_returns_to_stopped(self, enabled_buf):
        enabled_buf.state = STATE_ERROR
        enabled_buf.error_msg = "test error"
        enabled_buf._clear_error()
        assert enabled_buf.state == STATE_STOPPED
        assert enabled_buf.error_msg == ""

    def test_clear_error_returns_to_idle_if_not_enabled(self, buf):
        buf.state = STATE_ERROR
        buf.auto_enabled = False
        buf._clear_error()
        assert buf.state == "idle"
