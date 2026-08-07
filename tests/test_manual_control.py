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
    def test_press_starts_feeding(self, buf, buttons, reactor, sidecar_moves):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        assert buf.state == STATE_MANUAL_FEED
        assert buf.motor_direction == FORWARD
        assert len(sidecar_moves) > 0

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
    def test_press_starts_retracting(self, buf, buttons, reactor, sidecar_moves):
        reactor._monotonic = 10.0
        buttons.callbacks["PE5"](10.0, 1)
        assert buf.state == STATE_MANUAL_RETRACT
        assert buf.motor_direction == BACK
        assert len(sidecar_moves) > 0
        assert sidecar_moves[-1][1] < 0  # negative dist

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
            self, buf, buttons, reactor, sidecar_moves):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        # First chunk issued and a continuation timer is scheduled at
        # chunk-completion eventtime.
        assert len(sidecar_moves) == 1
        assert buf._continuous_timer is not None

    def test_feed_button_held_produces_multiple_chunks(
            self, buf, buttons, reactor, sidecar_moves):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        assert buf.state == STATE_MANUAL_FEED
        # Advance past each chunk's completion so the timer fires the
        # next chunk and the sidecar throttle releases.
        for _ in range(3):
            reactor.advance_time(0.5)
        assert len(sidecar_moves) == 4
        # All chunks feed forward.
        assert all(m[1] > 0 for m in sidecar_moves)

    def test_retract_button_held_produces_multiple_chunks(
            self, buf, buttons, reactor, sidecar_moves):
        reactor._monotonic = 10.0
        buttons.callbacks["PE5"](10.0, 1)
        assert buf.state == STATE_MANUAL_RETRACT
        for _ in range(3):
            reactor.advance_time(0.5)
        assert len(sidecar_moves) == 4
        assert all(m[1] < 0 for m in sidecar_moves)

    def test_long_hold_does_not_stop_after_two_chunks(
            self, buf, buttons, reactor, sidecar_moves):
        """Regression: previously the chunked-callback used
        update_timer + return NEVER, which in production Klipper
        disarms the timer because update_timer is a no-op while a
        timer's callback runs and the return value unconditionally
        overwrites the timer's waketime.  Symptom on hardware: hold
        the feed button, get exactly two chunks (one inline + one
        timer-fire), then nothing.  The mock reactor previously
        masked this by honouring update_timer-from-within-callback;
        it now mirrors production semantics, so this loop only stays
        going if the production code returns the next waketime from
        the callback itself."""
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        # Hold for 10 chunk durations worth of time.
        for _ in range(10):
            reactor.advance_time(0.5)
        assert len(sidecar_moves) >= 8, (
            "expected continuous chunks, got only %d — chunk loop "
            "stopped early (regression of cfaba3f timer pattern)"
            % len(sidecar_moves))

    def test_brief_feed_tap_produces_exactly_one_chunk(
            self, buf, buttons, reactor, sidecar_moves):
        """Regression: pressing+releasing the feed button quickly must
        not queue multiple chunks ahead.  The earlier register_callback
        chain fired callbacks back-to-back within a single reactor pass,
        so a brief tap committed 3 chunks (30mm) to the MCU queue
        before the release callback flipped state."""
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        # Release immediately, before any chunk-completion timer can fire.
        buttons.callbacks["PE4"](10.001, 0)
        # Still advance time well past chunk duration — the cancelled
        # timer must not produce a second chunk.
        for _ in range(5):
            reactor.advance_time(0.5)
        assert len(sidecar_moves) == 1

    def test_brief_retract_tap_produces_exactly_one_chunk(
            self, buf, buttons, reactor, sidecar_moves):
        reactor._monotonic = 10.0
        buttons.callbacks["PE5"](10.0, 1)
        buttons.callbacks["PE5"](10.001, 0)
        for _ in range(5):
            reactor.advance_time(0.5)
        assert len(sidecar_moves) == 1

    def test_feed_button_release_stops_loop(
            self, buf, buttons, reactor, sidecar_moves):
        reactor._monotonic = 10.0
        buttons.callbacks["PE4"](10.0, 1)
        buttons.callbacks["PE4"](10.5, 0)
        # After release, any pending chunk callback should see state != MANUAL
        # and not issue another move.
        chunks_before = len(sidecar_moves)
        reactor.flush_callbacks()
        assert len(sidecar_moves) == chunks_before


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


class TestFixedMoveCompletion:
    """Fixed-distance BUFFER_FEED/RETRACT (DIST>0) previously stranded
    the state machine in MANUAL_* forever — no completion callback,
    and sensor callbacks exclude manual states.  The shared completion
    timer restores STOPPED/IDLE after the move's real duration."""

    def _feed(self, b, dist=20.0):
        from conftest import MockGcmd
        b.cmd_BUFFER_FEED(MockGcmd(params={"DIST": dist}))
        return b._chunk_move_duration(dist, b.manual_speed,
                                      b.manual_accel)

    def test_fixed_feed_returns_to_idle_after_duration(
            self, buf, reactor):
        reactor._monotonic = 10.0
        dur = self._feed(buf)
        assert buf.state == STATE_MANUAL_FEED
        reactor.advance_time(dur + 0.1)
        assert buf.state == STATE_IDLE
        assert buf.motor_direction == STOP

    def test_fixed_feed_returns_to_stopped_when_auto(
            self, enabled_buf, reactor):
        reactor._monotonic = 10.0
        dur = self._feed(enabled_buf)
        reactor.advance_time(dur + 0.1)
        # Unsynced at completion -> STOPPED; the next control tick
        # re-syncs and promotes to FEEDING.
        assert enabled_buf.state == STATE_STOPPED
        enabled_buf._control_timer_cb(reactor.monotonic())
        assert enabled_buf._synced_to is not None
        assert enabled_buf.state == STATE_FEEDING

    def test_fixed_retract_returns_after_duration(self, buf, reactor):
        from conftest import MockGcmd
        reactor._monotonic = 10.0
        buf.cmd_BUFFER_RETRACT(MockGcmd(params={"DIST": 15.0}))
        assert buf.state == STATE_MANUAL_RETRACT
        dur = buf._chunk_move_duration(15.0, buf.manual_speed,
                                       buf.manual_accel)
        reactor.advance_time(dur + 0.1)
        assert buf.state == STATE_IDLE

    def test_completion_noop_after_buffer_stop(
            self, enabled_buf, reactor):
        from conftest import MockGcmd
        reactor._monotonic = 10.0
        dur = self._feed(enabled_buf)
        enabled_buf.cmd_BUFFER_STOP(MockGcmd())
        state_after_stop = enabled_buf.state
        reactor.advance_time(dur + 0.1)
        # The stale completion must not fire (cancelled) nor change
        # the state BUFFER_STOP established.
        assert enabled_buf.state == state_after_stop

    def test_completion_cancelled_by_button_feed(
            self, enabled_buf, reactor):
        reactor._monotonic = 10.0
        dur = self._feed(enabled_buf)
        # Button-held feed takes over the MANUAL_FEED state; the
        # pending fixed-move completion must not kill it mid-hold.
        enabled_buf._feed_button_callback(10.1, 1)
        assert enabled_buf.state == STATE_MANUAL_FEED
        reactor.advance_time(dur + 5.0)
        assert enabled_buf.state == STATE_MANUAL_FEED

    def test_fixed_feed_failure_still_recovers(self, buf, reactor):
        def boom(*args, **kwargs):
            raise Exception("manual_move failed")
        buf.force_move.manual_move = boom
        reactor._monotonic = 10.0
        dur = self._feed(buf)
        assert buf.state == STATE_MANUAL_FEED
        reactor.advance_time(dur + 0.1)
        assert buf.state == STATE_IDLE
