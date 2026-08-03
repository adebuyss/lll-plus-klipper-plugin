"""Tests for continuous feed/retract mode (BUFFER_FEED/BUFFER_RETRACT with no DIST)."""

from conftest import (
    FORWARD,
    BACK,
    STOP,
    STATE_MANUAL_FEED,
    STATE_MANUAL_RETRACT,
    STATE_IDLE,
    STATE_STOPPED,
    STATE_FEEDING,
    MockGcmd,
    set_sensors,
)


class TestContinuousFeed:
    def test_feed_no_dist_starts_continuous(self, buf, sidecar_moves, reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_FEED")
        buf.cmd_BUFFER_FEED(gcmd)
        assert buf.state == STATE_MANUAL_FEED
        assert buf.motor_direction == FORWARD
        assert len(sidecar_moves) > 0
        assert sidecar_moves[-1][1] > 0  # positive distance
        # A continuation timer is scheduled at chunk-completion eventtime
        # so a brief button tap fires one chunk, not a back-to-back burst.
        assert buf._continuous_timer is not None

    def test_feed_continuous_stops_on_full(self, buf, sidecar_moves, reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_FEED")
        buf.cmd_BUFFER_FEED(gcmd)

        # Trigger full sensor
        set_sensors(buf, full=True)
        reactor._monotonic = 2.0
        buf._control_timer_cb(2.0)
        # First tick starts the full-sensor timer
        assert buf.state == STATE_MANUAL_FEED

        # Advance past manual_feed_full_timeout
        reactor._monotonic = 2.0 + buf.manual_feed_full_timeout + 0.1
        buf._control_timer_cb(reactor._monotonic)
        assert buf.state == STATE_IDLE
        assert buf.motor_direction == STOP

    def test_feed_continuous_cancelled_by_stop(self, buf, sidecar_moves, reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_FEED")
        buf.cmd_BUFFER_FEED(gcmd)
        assert buf.state == STATE_MANUAL_FEED

        # BUFFER_STOP changes state so pending chunk callback aborts
        buf.cmd_BUFFER_STOP(MockGcmd("BUFFER_STOP"))
        assert buf.motor_direction == STOP

        # Pending callback should see wrong state and not issue more chunks
        chunks_before = len(sidecar_moves)
        reactor.flush_callbacks()
        assert len(sidecar_moves) == chunks_before

    def test_feed_continuous_custom_speed(self, buf, sidecar_moves, reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_FEED", {"SPEED": 25.0})
        buf.cmd_BUFFER_FEED(gcmd)
        assert sidecar_moves[-1][2] == 25.0

    def test_autofeed_stop_at_full_does_not_start_drain_loop(
            self, enabled_buf, reactor, vactual_writes):
        # The reported field failure: no [virtual_sdcard] ->
        # print_stats missing -> _is_printing() always True.
        # BUFFER_FEED (no DIST) threads filament to the toolhead; the
        # sustained-full auto-stop then re-syncs the buffer at full
        # extend with the extruder idle.  FULL recovery must DEFER
        # here — an unconditional drain would pull the arm out of
        # full extend, unlatch the stop condition, and loop the load
        # flow forever.
        enabled_buf._print_stats = None
        reactor._monotonic = 1.0
        enabled_buf.cmd_BUFFER_FEED(MockGcmd("BUFFER_FEED"))
        assert enabled_buf.state == STATE_MANUAL_FEED

        # Filament reaches the toolhead: full sensor latches.
        set_sensors(enabled_buf, full=True)
        reactor._monotonic = 2.0
        enabled_buf._control_timer_cb(2.0)
        assert enabled_buf.state == STATE_MANUAL_FEED  # timer armed

        # Sustained full -> auto-stop -> re-sync at full extend.
        reactor._monotonic = 2.0 + enabled_buf.manual_feed_full_timeout + 0.1
        enabled_buf._control_timer_cb(reactor._monotonic)
        assert enabled_buf.state in (STATE_STOPPED, STATE_FEEDING)
        assert enabled_buf._synced_to is not None

        # Several more control ticks with the arm parked at full
        # extend and the extruder idle: recovery must keep deferring.
        # (Entering recovery always appends a VACTUAL write, so a
        # stable write count proves no drain started.)
        writes_after_stop = len(vactual_writes)
        for _ in range(6):
            reactor._monotonic += enabled_buf.control_interval
            enabled_buf._control_timer_cb(reactor._monotonic)
            assert enabled_buf._extreme_recovery_active is None
            assert len(vactual_writes) == writes_after_stop
            assert enabled_buf.state in (STATE_STOPPED, STATE_FEEDING)
        # The bounded hard escape (full_safety_timeout ->
        # _do_safety_retract) stays armed.
        assert enabled_buf._safety_zone_start > 0.0


class TestContinuousRetract:
    def test_retract_no_dist_starts_continuous(self, buf, sidecar_moves, reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_RETRACT")
        buf.cmd_BUFFER_RETRACT(gcmd)
        assert buf.state == STATE_MANUAL_RETRACT
        assert buf.motor_direction == BACK
        assert len(sidecar_moves) > 0
        assert sidecar_moves[-1][1] < 0  # negative distance

    def test_retract_continuous_stops_on_empty(self, buf, sidecar_moves, reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_RETRACT")
        buf.cmd_BUFFER_RETRACT(gcmd)
        assert buf.state == STATE_MANUAL_RETRACT

        # Trigger empty sensor — control timer auto-stops immediately
        set_sensors(buf, empty=True)
        reactor._monotonic = 2.0
        buf._control_timer_cb(2.0)
        assert buf.state == STATE_IDLE
        assert buf.motor_direction == STOP

    def test_retract_continuous_cancelled_by_stop(self, buf, sidecar_moves,
                                                   reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_RETRACT")
        buf.cmd_BUFFER_RETRACT(gcmd)

        buf.cmd_BUFFER_STOP(MockGcmd("BUFFER_STOP"))
        assert buf.motor_direction == STOP

        chunks_before = len(sidecar_moves)
        reactor.flush_callbacks()
        assert len(sidecar_moves) == chunks_before
