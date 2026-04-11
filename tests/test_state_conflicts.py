"""Tests for state machine conflict guards."""

import pytest
from conftest import (
    FORWARD,
    BACK,
    STOP,
    STATE_IDLE,
    STATE_STOPPED,
    STATE_FEEDING,
    STATE_DISABLED,
    STATE_ERROR,
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

    def test_buffer_retract_blocked_in_error(self, buf):
        buf.state = STATE_ERROR
        buf.cmd_BUFFER_RETRACT(MockGcmd("BUFFER_RETRACT"))
        assert buf.state == STATE_ERROR

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

    def test_error_via_sensor_blocks_feed(self, enabled_buf):
        set_sensors(enabled_buf, empty=True, full=True)
        enabled_buf._update_rotation_distance(1.0)
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
        # Sync transitions to FEEDING
        assert buf.state in (STATE_STOPPED, STATE_FEEDING)


class TestInitialState:
    def test_initial_state_is_idle(self, buf):
        assert buf.state == STATE_IDLE

    def test_initial_auto_enabled_is_false(self, buf):
        assert buf.auto_enabled is False

    def test_buffer_disable_sets_disabled(self, buf):
        buf.cmd_BUFFER_DISABLE(MockGcmd("BUFFER_DISABLE"))
        assert buf.state == STATE_DISABLED
