"""Tests for state machine conflict guards and reset consolidation."""

import pytest
from conftest import (
    FORWARD,
    BACK,
    STOP,
    STATE_IDLE,
    STATE_STOPPED,
    STATE_DISABLED,
    STATE_ERROR,
    STATE_FEEDING,
    STATE_MANUAL_FEED,
    STATE_MANUAL_RETRACT,
    MockGcmd,
    set_sensors,
)


class TestErrorBlocksGcodeCommands:
    def test_buffer_feed_blocked_in_error(self, buf):
        buf.state = STATE_ERROR
        buf.cmd_BUFFER_FEED(MockGcmd("BUFFER_FEED"))
        assert buf.state == STATE_ERROR
        assert buf.motor_direction == STOP

    def test_buffer_retract_blocked_in_error(self, buf):
        buf.state = STATE_ERROR
        buf.cmd_BUFFER_RETRACT(MockGcmd("BUFFER_RETRACT"))
        assert buf.state == STATE_ERROR
        assert buf.motor_direction == STOP

    def test_buffer_feed_works_after_clear_error(self, buf):
        buf.auto_enabled = True
        buf.state = STATE_ERROR
        buf.cmd_BUFFER_CLEAR_ERROR(MockGcmd("BUFFER_CLEAR_ERROR"))
        buf.cmd_BUFFER_FEED(MockGcmd("BUFFER_FEED"))
        assert buf.state == STATE_MANUAL_FEED
        assert buf.motor_direction == FORWARD

    def test_buffer_retract_works_after_clear_error(self, buf):
        buf.auto_enabled = True
        buf.state = STATE_ERROR
        buf.cmd_BUFFER_CLEAR_ERROR(MockGcmd("BUFFER_CLEAR_ERROR"))
        buf.cmd_BUFFER_RETRACT(MockGcmd("BUFFER_RETRACT"))
        assert buf.state == STATE_MANUAL_RETRACT
        assert buf.motor_direction == BACK

    def test_error_during_feeding_blocks_subsequent_feed(self, enabled_buf):
        set_sensors(enabled_buf, empty=True, full=True)
        enabled_buf._evaluate_and_drive(1.0)
        assert enabled_buf.state == STATE_ERROR
        enabled_buf.cmd_BUFFER_FEED(MockGcmd("BUFFER_FEED"))
        assert enabled_buf.state == STATE_ERROR


class TestDisabledBlocksMaterialAutoEnable:
    def test_insert_after_disable_stays_disabled(self, buf, buttons):
        buf.cmd_BUFFER_DISABLE(MockGcmd("BUFFER_DISABLE"))
        buttons.callbacks["PE3"](1.0, 1)
        assert buf.auto_enabled is False
        assert buf.state == STATE_DISABLED

    def test_insert_after_idle_auto_enables(self, buf, buttons):
        assert buf.state == STATE_IDLE
        buttons.callbacks["PE3"](1.0, 1)
        assert buf.auto_enabled is True

    def test_enable_after_disable_then_insert_works(self, buf, buttons):
        buf.cmd_BUFFER_DISABLE(MockGcmd("BUFFER_DISABLE"))
        buttons.callbacks["PE3"](1.0, 1)
        assert buf.auto_enabled is False
        buf.cmd_BUFFER_ENABLE(MockGcmd("BUFFER_ENABLE"))
        assert buf.auto_enabled is True
        assert buf.state == STATE_STOPPED

    def test_disable_remove_insert_stays_disabled(self, buf, buttons):
        buf.cmd_BUFFER_DISABLE(MockGcmd("BUFFER_DISABLE"))
        buttons.callbacks["PE3"](1.0, 0)
        buttons.callbacks["PE3"](2.0, 1)
        assert buf.state == STATE_DISABLED
        assert buf.auto_enabled is False


class TestInitialState:
    def test_initial_state_is_idle(self, buf):
        assert buf.state == STATE_IDLE

    def test_initial_auto_enabled_is_false(self, buf):
        assert buf.auto_enabled is False

    def test_buffer_disable_sets_disabled(self, buf):
        buf.cmd_BUFFER_DISABLE(MockGcmd("BUFFER_DISABLE"))
        assert buf.state == STATE_DISABLED


class TestResetConsolidation:
    def _dirty(self, buf):
        buf._full_zone_feed_time = 5.0
        buf._last_full_feed_time = 4.0
        buf._full_zone_retract_start = 3.0
        buf._burst_delay_start = 2.0
        buf._burst_until = 1.0
        buf._burst_count = 7
        buf._velocity_window = [(0.0, 1.0)]
        buf._manual_feed_full_start = 1.0
        buf._initial_fill_until = 99.0

    def test_reset_control_state_clears_all(self, buf):
        self._dirty(buf)
        buf._reset_control_state()
        assert buf._full_zone_feed_time == 0.0
        assert buf._last_full_feed_time == 0.0
        assert buf._full_zone_retract_start == 0.0
        assert buf._burst_delay_start == 0.0
        assert buf._burst_until == 0.0
        assert buf._burst_count == 0
        assert buf._velocity_window == []
        assert buf._manual_feed_full_start == 0.0
        assert buf._initial_fill_until == 0.0

    def test_enable_uses_reset_control_state(self, buf):
        buf._full_zone_retract_start = 99.0
        buf.cmd_BUFFER_ENABLE(MockGcmd("BUFFER_ENABLE"))
        assert buf._full_zone_retract_start == 0.0

    def test_disable_uses_reset_control_state(self, buf):
        buf._full_zone_retract_start = 99.0
        buf.cmd_BUFFER_DISABLE(MockGcmd("BUFFER_DISABLE"))
        assert buf._full_zone_retract_start == 0.0

    def test_clear_error_uses_reset_control_state(self, buf):
        buf.state = STATE_ERROR
        buf._full_zone_retract_start = 99.0
        buf.cmd_BUFFER_CLEAR_ERROR(MockGcmd("BUFFER_CLEAR_ERROR"))
        assert buf._full_zone_retract_start == 0.0
