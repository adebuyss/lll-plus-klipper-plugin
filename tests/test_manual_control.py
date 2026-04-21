"""Tests for manual feed/retract button overrides."""

import pytest
from conftest import (
    FORWARD,
    BACK,
    STOP,
    STATE_IDLE,
    STATE_STOPPED,
    STATE_FEEDING,
    STATE_MANUAL_FEED,
    STATE_MANUAL_RETRACT,
    STATE_ERROR,
    set_sensors,
)


class TestFeedButton:
    def test_press_starts_feeding(self, buf, buttons, reactor, force_move):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        assert buf.state == STATE_MANUAL_FEED
        assert buf.motor_direction == FORWARD
        assert len(force_move.moves) > 0

    def test_release_stops(self, buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        assert buf.state == STATE_MANUAL_FEED

        buttons.callbacks["PE4"](10.5, 0)
        assert buf.state == STATE_IDLE
        assert buf.motor_direction == STOP

    def test_release_with_auto_enabled(self, enabled_buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        assert enabled_buf.state == STATE_MANUAL_FEED

        buttons.callbacks["PE4"](10.5, 0)
        assert enabled_buf.auto_enabled is True
        # Re-sync transitions to FEEDING
        assert enabled_buf.state in (STATE_STOPPED, STATE_FEEDING)

    def test_press_in_error_ignored(self, buf, buttons, reactor):
        buf.state = STATE_ERROR
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        assert buf.state == STATE_ERROR


class TestRetractButton:
    def test_press_starts_retracting(self, buf, buttons, reactor, force_move):
        reactor._monotonic = 10.0
        buttons.callbacks["PE5"](10.0, 1)
        assert buf.state == STATE_MANUAL_RETRACT
        assert buf.motor_direction == BACK
        assert len(force_move.moves) > 0
        assert force_move.moves[-1][1] < 0  # negative dist

    def test_release_stops(self, buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks["PE5"](10.0, 1)
        buttons.callbacks["PE5"](10.5, 0)
        assert buf.state == STATE_IDLE
        assert buf.motor_direction == STOP

    def test_press_in_error_ignored(self, buf, buttons, reactor):
        buf.state = STATE_ERROR
        buttons.callbacks["PE5"](10.0, 1)
        assert buf.state == STATE_ERROR


class TestButtonHoldFeedsContinuously:
    """Holding a manual button should keep chunks scheduling until release."""

    def test_feed_button_schedules_continuation(
            self, buf, buttons, reactor, force_move):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        # First chunk issued and a continuation is pending in the reactor.
        assert len(force_move.moves) == 1
        assert len(reactor._pending_callbacks) > 0

    def test_feed_button_held_produces_multiple_chunks(
            self, buf, buttons, reactor, force_move):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        assert buf.state == STATE_MANUAL_FEED
        # Fire three reactor ticks; each should issue another chunk.
        for _ in range(3):
            cb = reactor._pending_callbacks.pop(0)
            cb(reactor._monotonic)
        assert len(force_move.moves) == 4
        # All chunks feed forward.
        assert all(m[1] > 0 for m in force_move.moves)

    def test_retract_button_held_produces_multiple_chunks(
            self, buf, buttons, reactor, force_move):
        reactor._monotonic = 10.0
        buttons.callbacks["PE5"](10.0, 1)
        assert buf.state == STATE_MANUAL_RETRACT
        for _ in range(3):
            cb = reactor._pending_callbacks.pop(0)
            cb(reactor._monotonic)
        assert len(force_move.moves) == 4
        assert all(m[1] < 0 for m in force_move.moves)

    def test_feed_button_release_stops_loop(
            self, buf, buttons, reactor, force_move):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        buttons.callbacks["PE4"](10.5, 0)
        # After release, any pending chunk callback should see state != MANUAL
        # and not issue another move.
        chunks_before = len(force_move.moves)
        reactor.flush_callbacks()
        assert len(force_move.moves) == chunks_before


class TestButtonReleaseRestoresState:
    def test_retract_release_restores_auto(self, enabled_buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks["PE5"](10.0, 1)
        assert enabled_buf.state == STATE_MANUAL_RETRACT

        buttons.callbacks["PE5"](10.5, 0)
        assert enabled_buf.auto_enabled is True
        assert enabled_buf.state in (STATE_STOPPED, STATE_FEEDING)

    def test_release_without_auto_stays_idle(self, buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks["PE5"](10.0, 1)
        buttons.callbacks["PE5"](10.5, 0)
        assert buf.auto_enabled is False
        assert buf.state == STATE_IDLE


class TestBothButtonsToggle:
    def test_feed_then_retract_enables(self, buf, buttons, reactor):
        assert buf.auto_enabled is False
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        buttons.callbacks["PE5"](10.1, 1)
        assert buf.auto_enabled is True
        # Sync transitions to FEEDING
        assert buf.state in (STATE_STOPPED, STATE_FEEDING)

    def test_toggle_disables_when_enabled(self, enabled_buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        buttons.callbacks["PE5"](10.1, 1)
        assert enabled_buf.auto_enabled is False
        assert enabled_buf.state == STATE_IDLE

    def test_toggle_in_error_starts_hold(self, buf, buttons, reactor):
        buf.state = STATE_ERROR
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        buttons.callbacks["PE5"](10.1, 1)
        # Should start error clear hold, not toggle
        assert buf.state == STATE_ERROR
        assert buf._error_clear_hold_start == 10.1

    def test_toggle_sends_message(self, buf, buttons, reactor, gcode):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        buttons.callbacks["PE5"](10.1, 1)
        assert any("enabled via buttons" in r for r in gcode.responses)


class TestErrorClearViaButtons:
    def test_2s_hold_clears_error(self, buf, buttons, reactor):
        buf.state = STATE_ERROR
        buf.error_msg = "test"
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        buttons.callbacks["PE5"](10.1, 1)
        assert buf._error_clear_hold_start == 10.1

        # Advance past hold time
        reactor.advance_time(buf.error_clear_hold_time + 0.2)
        assert buf.state != STATE_ERROR
        assert buf.error_msg == ""

    def test_release_before_timeout_does_not_clear(self, buf, buttons,
                                                    reactor):
        buf.state = STATE_ERROR
        buf.error_msg = "test"
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        buttons.callbacks["PE5"](10.1, 1)

        # Release before 2s
        buttons.callbacks["PE4"](11.0, 0)
        assert buf._error_clear_hold_start == 0.0
        assert buf.state == STATE_ERROR


class TestManualFeedAutoStop:
    def test_stops_on_sustained_full(self, enabled_buf, reactor):
        enabled_buf.state = STATE_MANUAL_FEED
        enabled_buf.motor_direction = FORWARD
        set_sensors(enabled_buf, full=True)

        reactor._monotonic = 10.0
        enabled_buf._control_timer_cb(10.0)
        assert enabled_buf.state == STATE_MANUAL_FEED

        reactor._monotonic = 15.5
        enabled_buf._control_timer_cb(15.5)
        # After auto-stop, auto_enabled is preserved and the buffer
        # returns to STOPPED (then FEEDING once _sync() completes) so
        # normal control resumes instead of silently disabling.
        assert enabled_buf.state in (STATE_STOPPED, STATE_FEEDING)
        assert enabled_buf.motor_direction in (STOP, FORWARD)
        assert enabled_buf.auto_enabled is True

    def test_no_stop_on_brief_full(self, enabled_buf, reactor):
        enabled_buf.state = STATE_MANUAL_FEED
        enabled_buf.motor_direction = FORWARD
        set_sensors(enabled_buf, full=True)

        reactor._monotonic = 10.0
        enabled_buf._control_timer_cb(10.0)
        reactor._monotonic = 12.0
        enabled_buf._control_timer_cb(12.0)
        assert enabled_buf.state == STATE_MANUAL_FEED

        set_sensors(enabled_buf, full=False)
        reactor._monotonic = 12.5
        enabled_buf._control_timer_cb(12.5)
        assert enabled_buf.state == STATE_MANUAL_FEED


class TestButtonHeldOverridesAutoStop:
    def test_button_held_feed_ignores_full_sensor(
            self, enabled_buf, reactor):
        """Holding the feed button keeps MANUAL_FEED running even if the
        full sensor triggers — user's explicit intent."""
        enabled_buf.state = STATE_MANUAL_FEED
        enabled_buf.motor_direction = FORWARD
        enabled_buf._feed_button_pressed = True
        set_sensors(enabled_buf, full=True)

        reactor._monotonic = 10.0
        enabled_buf._control_timer_cb(10.0)
        reactor._monotonic = 15.5
        enabled_buf._control_timer_cb(15.5)
        # Still manual feeding; auto-stop skipped while button is held.
        assert enabled_buf.state == STATE_MANUAL_FEED
        assert enabled_buf.auto_enabled is True

    def test_button_held_retract_ignores_empty_sensor(
            self, enabled_buf, reactor):
        """Holding the retract button keeps MANUAL_RETRACT running even
        if the empty sensor triggers."""
        enabled_buf.state = STATE_MANUAL_RETRACT
        enabled_buf.motor_direction = "back"
        enabled_buf._retract_button_pressed = True
        set_sensors(enabled_buf, empty=True)

        reactor._monotonic = 10.0
        enabled_buf._control_timer_cb(10.0)
        # Still manual retracting; empty-sensor auto-stop skipped.
        assert enabled_buf.state == STATE_MANUAL_RETRACT
        assert enabled_buf.auto_enabled is True

    def test_non_button_retract_auto_stop_preserves_auto_enabled(
            self, enabled_buf, reactor):
        """BUFFER_RETRACT continuous hitting empty sensor should stop the
        motor but leave auto_enabled intact so the buffer returns to
        normal control after the motor stops."""
        enabled_buf.state = STATE_MANUAL_RETRACT
        enabled_buf.motor_direction = "back"
        enabled_buf._retract_button_pressed = False  # not button-held
        set_sensors(enabled_buf, empty=True)

        reactor._monotonic = 10.0
        enabled_buf._control_timer_cb(10.0)
        assert enabled_buf.auto_enabled is True
        # State should have returned to STOPPED/FEEDING via _sync().
        assert enabled_buf.state in (STATE_STOPPED, STATE_FEEDING)
