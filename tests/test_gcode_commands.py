"""Tests for all BUFFER_* gcode command handlers."""

import pytest
from conftest import (
    FORWARD,
    BACK,
    STOP,
    STATE_DISABLED,
    STATE_IDLE,
    STATE_STOPPED,
    STATE_ERROR,
    STATE_MANUAL_FEED,
    STATE_MANUAL_RETRACT,
    ZONE_FULL,
    MockGcmd,
)


class TestBufferStatus:
    def test_reports_all_fields(self, buf, gcode):
        gcmd = MockGcmd("BUFFER_STATUS")
        buf.cmd_BUFFER_STATUS(gcmd)
        assert len(gcmd.responses) == 1
        resp = gcmd.responses[0]
        assert "State:" in resp
        assert "Motor:" in resp
        assert "Sensors:" in resp

    def test_reports_error(self, buf, gcode):
        buf.state = STATE_ERROR
        buf.error_msg = "test error message"
        gcmd = MockGcmd("BUFFER_STATUS")
        buf.cmd_BUFFER_STATUS(gcmd)
        assert "test error message" in gcmd.responses[0]


class TestBufferEnable:
    def test_enables_auto_control(self, buf):
        gcmd = MockGcmd("BUFFER_ENABLE")
        buf.cmd_BUFFER_ENABLE(gcmd)
        assert buf.auto_enabled is True
        assert buf.state == STATE_STOPPED
        assert buf.error_msg == ""

    def test_resets_state(self, buf):
        buf._current_zone = ZONE_FULL
        buf._full_zone_feed_time = 5.0
        buf._burst_count = 3
        buf._extruder_retracting = True
        gcmd = MockGcmd("BUFFER_ENABLE")
        buf.cmd_BUFFER_ENABLE(gcmd)
        assert buf._current_zone != ZONE_FULL
        assert buf._full_zone_feed_time == 0.0
        assert buf._burst_count == 0
        assert buf._extruder_retracting is False


class TestBufferDisable:
    def test_disables_and_stops(self, enabled_buf):
        gcmd = MockGcmd("BUFFER_DISABLE")
        enabled_buf.cmd_BUFFER_DISABLE(gcmd)
        assert enabled_buf.auto_enabled is False
        assert enabled_buf.state == STATE_DISABLED
        assert enabled_buf.motor_direction == STOP

    def test_resets_zone_state(self, enabled_buf):
        enabled_buf._current_zone = ZONE_FULL
        enabled_buf._burst_count = 3
        gcmd = MockGcmd("BUFFER_DISABLE")
        enabled_buf.cmd_BUFFER_DISABLE(gcmd)
        assert enabled_buf._current_zone != ZONE_FULL
        assert enabled_buf._burst_count == 0


class TestBufferFeed:
    def test_feeds_at_default_speed(self, buf):
        gcmd = MockGcmd("BUFFER_FEED")
        buf.cmd_BUFFER_FEED(gcmd)
        assert buf.motor_direction == FORWARD
        assert buf.state == STATE_MANUAL_FEED

    def test_feeds_at_custom_speed(self, buf):
        gcmd = MockGcmd("BUFFER_FEED", {"SPEED": 100.0})
        buf.cmd_BUFFER_FEED(gcmd)
        assert buf.motor_direction == FORWARD
        assert "100" in gcmd.responses[0]


class TestBufferRetract:
    def test_retracts(self, buf):
        gcmd = MockGcmd("BUFFER_RETRACT")
        buf.cmd_BUFFER_RETRACT(gcmd)
        assert buf.motor_direction == BACK
        assert buf.state == STATE_MANUAL_RETRACT

    def test_retracts_at_custom_speed(self, buf):
        gcmd = MockGcmd("BUFFER_RETRACT", {"SPEED": 150.0})
        buf.cmd_BUFFER_RETRACT(gcmd)
        assert "150" in gcmd.responses[0]


class TestBufferStop:
    def test_stops_with_auto_enabled(self, enabled_buf):
        enabled_buf.motor_direction = FORWARD
        gcmd = MockGcmd("BUFFER_STOP")
        enabled_buf.cmd_BUFFER_STOP(gcmd)
        assert enabled_buf.motor_direction == STOP
        assert enabled_buf.state == STATE_STOPPED

    def test_stops_without_auto(self, buf):
        buf.auto_enabled = False
        gcmd = MockGcmd("BUFFER_STOP")
        buf.cmd_BUFFER_STOP(gcmd)
        assert buf.state == STATE_IDLE


class TestBufferSetSpeed:
    def test_sets_speed(self, buf):
        gcmd = MockGcmd("BUFFER_SET_SPEED", {"SPEED": 300.0})
        buf.cmd_BUFFER_SET_SPEED(gcmd)
        assert buf.motor.speed_rpm == 300.0


class TestBufferClearError:
    def test_clears_error(self, enabled_buf):
        enabled_buf.state = STATE_ERROR
        enabled_buf.error_msg = "test"
        enabled_buf._current_zone = ZONE_FULL
        enabled_buf._burst_count = 5
        gcmd = MockGcmd("BUFFER_CLEAR_ERROR")
        enabled_buf.cmd_BUFFER_CLEAR_ERROR(gcmd)
        assert enabled_buf.state == STATE_STOPPED
        assert enabled_buf.error_msg == ""
        assert enabled_buf._current_zone != ZONE_FULL
        assert enabled_buf._burst_count == 0

    def test_clear_when_no_error(self, buf):
        buf.state = STATE_IDLE
        gcmd = MockGcmd("BUFFER_CLEAR_ERROR")
        buf.cmd_BUFFER_CLEAR_ERROR(gcmd)
        assert buf.state == STATE_IDLE
        assert "no error" in gcmd.responses[0].lower()

    def test_clear_error_without_auto(self, buf):
        buf.state = STATE_ERROR
        buf.auto_enabled = False
        gcmd = MockGcmd("BUFFER_CLEAR_ERROR")
        buf.cmd_BUFFER_CLEAR_ERROR(gcmd)
        assert buf.state == STATE_IDLE
