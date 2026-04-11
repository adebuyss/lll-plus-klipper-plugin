"""Tests for all BUFFER_* gcode command handlers."""

import pytest
from conftest import (
    FORWARD,
    BACK,
    STOP,
    STATE_DISABLED,
    STATE_IDLE,
    STATE_STOPPED,
    STATE_FEEDING,
    STATE_ERROR,
    STATE_MANUAL_FEED,
    STATE_MANUAL_RETRACT,
    MockGcmd,
)


class TestBufferStatus:
    def test_reports_all_fields(self, buf):
        gcmd = MockGcmd("BUFFER_STATUS")
        buf.cmd_BUFFER_STATUS(gcmd)
        assert len(gcmd.responses) == 1
        resp = gcmd.responses[0]
        assert "State:" in resp
        assert "Motor:" in resp
        assert "Sensors:" in resp
        assert "RD multiplier:" in resp
        assert "Synced to:" in resp

    def test_reports_error(self, buf):
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
        # Syncing to extruder transitions to FEEDING
        assert buf.state in (STATE_STOPPED, STATE_FEEDING)
        assert buf.error_msg == ""

    def test_syncs_to_extruder(self, buf):
        gcmd = MockGcmd("BUFFER_ENABLE")
        buf.cmd_BUFFER_ENABLE(gcmd)
        assert buf._synced_to is not None


class TestBufferDisable:
    def test_disables_and_stops(self, enabled_buf):
        gcmd = MockGcmd("BUFFER_DISABLE")
        enabled_buf.cmd_BUFFER_DISABLE(gcmd)
        assert enabled_buf.auto_enabled is False
        assert enabled_buf.state == STATE_DISABLED
        assert enabled_buf._synced_to is None

    def test_unsyncs_stepper(self, enabled_buf):
        assert enabled_buf._synced_to is not None
        gcmd = MockGcmd("BUFFER_DISABLE")
        enabled_buf.cmd_BUFFER_DISABLE(gcmd)
        assert enabled_buf._synced_to is None


class TestBufferFeed:
    def test_feeds_forward(self, buf, force_move):
        gcmd = MockGcmd("BUFFER_FEED")
        buf.cmd_BUFFER_FEED(gcmd)
        assert buf.motor_direction == FORWARD
        assert buf.state == STATE_MANUAL_FEED
        assert len(force_move.moves) > 0
        assert force_move.moves[-1][1] > 0  # positive dist

    def test_feeds_at_custom_speed(self, buf, force_move):
        gcmd = MockGcmd("BUFFER_FEED", {"SPEED": 20.0})
        buf.cmd_BUFFER_FEED(gcmd)
        assert force_move.moves[-1][2] == 20.0  # speed

    def test_custom_distance(self, buf, force_move):
        gcmd = MockGcmd("BUFFER_FEED", {"DIST": 100.0})
        buf.cmd_BUFFER_FEED(gcmd)
        assert force_move.moves[-1][1] == 100.0

    def test_blocked_in_error(self, buf):
        buf.state = STATE_ERROR
        gcmd = MockGcmd("BUFFER_FEED")
        buf.cmd_BUFFER_FEED(gcmd)
        assert buf.state == STATE_ERROR


class TestBufferRetract:
    def test_retracts(self, buf, force_move):
        gcmd = MockGcmd("BUFFER_RETRACT")
        buf.cmd_BUFFER_RETRACT(gcmd)
        assert buf.motor_direction == BACK
        assert buf.state == STATE_MANUAL_RETRACT
        assert force_move.moves[-1][1] < 0  # negative dist

    def test_blocked_in_error(self, buf):
        buf.state = STATE_ERROR
        gcmd = MockGcmd("BUFFER_RETRACT")
        buf.cmd_BUFFER_RETRACT(gcmd)
        assert buf.state == STATE_ERROR


class TestBufferStop:
    def test_stops_with_auto_enabled(self, enabled_buf):
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
        gcmd = MockGcmd("BUFFER_SET_SPEED", {"SPEED": 25.0})
        buf.cmd_BUFFER_SET_SPEED(gcmd)
        assert buf.manual_speed == 25.0


class TestBufferClearError:
    def test_clears_error(self, enabled_buf):
        enabled_buf.state = STATE_ERROR
        enabled_buf.error_msg = "test"
        gcmd = MockGcmd("BUFFER_CLEAR_ERROR")
        enabled_buf.cmd_BUFFER_CLEAR_ERROR(gcmd)
        assert enabled_buf.state == STATE_STOPPED
        assert enabled_buf.error_msg == ""

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
