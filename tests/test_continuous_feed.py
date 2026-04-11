"""Tests for continuous feed/retract mode (BUFFER_FEED/BUFFER_RETRACT with no DIST)."""

from conftest import (
    FORWARD,
    BACK,
    STOP,
    STATE_MANUAL_FEED,
    STATE_MANUAL_RETRACT,
    STATE_IDLE,
    MockGcmd,
    set_sensors,
)


class TestContinuousFeed:
    def test_feed_no_dist_starts_continuous(self, buf, force_move, reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_FEED")
        buf.cmd_BUFFER_FEED(gcmd)
        assert buf.state == STATE_MANUAL_FEED
        assert buf.motor_direction == FORWARD
        assert len(force_move.moves) > 0
        assert force_move.moves[-1][1] > 0  # positive distance
        # A continuation callback should be pending
        assert len(reactor._pending_callbacks) > 0

    def test_feed_continuous_stops_on_full(self, buf, force_move, reactor):
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

    def test_feed_continuous_cancelled_by_stop(self, buf, force_move, reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_FEED")
        buf.cmd_BUFFER_FEED(gcmd)
        assert buf.state == STATE_MANUAL_FEED

        # BUFFER_STOP changes state so pending chunk callback aborts
        buf.cmd_BUFFER_STOP(MockGcmd("BUFFER_STOP"))
        assert buf.motor_direction == STOP

        # Pending callback should see wrong state and not issue more chunks
        chunks_before = len(force_move.moves)
        reactor.flush_callbacks()
        assert len(force_move.moves) == chunks_before

    def test_feed_continuous_custom_speed(self, buf, force_move, reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_FEED", {"SPEED": 25.0})
        buf.cmd_BUFFER_FEED(gcmd)
        assert force_move.moves[-1][2] == 25.0


class TestContinuousRetract:
    def test_retract_no_dist_starts_continuous(self, buf, force_move, reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_RETRACT")
        buf.cmd_BUFFER_RETRACT(gcmd)
        assert buf.state == STATE_MANUAL_RETRACT
        assert buf.motor_direction == BACK
        assert len(force_move.moves) > 0
        assert force_move.moves[-1][1] < 0  # negative distance

    def test_retract_continuous_stops_on_empty(self, buf, force_move, reactor):
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

    def test_retract_continuous_cancelled_by_stop(self, buf, force_move,
                                                   reactor):
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_RETRACT")
        buf.cmd_BUFFER_RETRACT(gcmd)

        buf.cmd_BUFFER_STOP(MockGcmd("BUFFER_STOP"))
        assert buf.motor_direction == STOP

        chunks_before = len(force_move.moves)
        reactor.flush_callbacks()
        assert len(force_move.moves) == chunks_before
