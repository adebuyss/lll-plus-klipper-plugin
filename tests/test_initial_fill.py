"""Tests for chunked initial fill on material insertion."""

from conftest import (
    FORWARD,
    STOP,
    STATE_FEEDING,
    STATE_IDLE,
    STATE_MANUAL_FEED,
    MockGcmd,
    set_sensors,
)


class TestInitialFillActivation:
    def test_material_insert_starts_fill(self, buf, buttons, reactor,
                                         sidecar_moves):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        # First chunk should have been issued
        assert buf.state == STATE_FEEDING
        assert buf.motor_direction == FORWARD
        assert len(sidecar_moves) > 0
        assert sidecar_moves[0][1] > 0  # positive distance

    def test_fill_queues_continuation(self, buf, buttons, reactor):
        """After the first chunk, a continuation timer is scheduled
        so the fill loop continues."""
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        assert buf._fill_timer is not None


class TestInitialFillAbortOnMiddle:
    def test_middle_sensor_stops_fill(self, buf, buttons, reactor,
                                      sidecar_moves):
        """Fill aborts when the middle sensor triggers between chunks."""
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        chunks_before = len(sidecar_moves)

        # Simulate middle sensor triggering between chunks
        set_sensors(buf, middle=True)
        t += 0.5
        reactor._monotonic = t
        # Fire the pending chunk callback
        reactor.flush_callbacks()

        # No new chunk should have been issued
        assert len(sidecar_moves) == chunks_before
        assert buf._initial_fill_until == 0.0
        # Should have synced to extruder
        assert buf._synced_to is not None

    def test_full_sensor_also_stops_fill(self, buf, buttons, reactor,
                                          sidecar_moves):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        chunks_before = len(sidecar_moves)

        set_sensors(buf, full=True)
        t += 0.5
        reactor._monotonic = t
        reactor.flush_callbacks()

        assert len(sidecar_moves) == chunks_before
        assert buf._initial_fill_until == 0.0


class TestInitialFillTimeout:
    def test_timeout_stops_fill(self, buf, buttons, reactor, sidecar_moves):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)

        # Advance past timeout
        t += buf.initial_fill_timeout + 1.0
        reactor._monotonic = t
        reactor.flush_callbacks()

        assert buf._initial_fill_until == 0.0
        assert buf.motor_direction == STOP


class TestInitialFillClear:
    def test_disable_during_fill(self, buf, buttons, reactor):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)

        buf.cmd_BUFFER_DISABLE(MockGcmd("BUFFER_DISABLE"))
        assert buf._synced_to is None

    def test_enable_does_not_set_fill(self, buf):
        buf.cmd_BUFFER_ENABLE(MockGcmd("BUFFER_ENABLE"))
        assert buf._initial_fill_until == 0.0

    def test_material_removal_cancels_fill(self, buf, buttons, reactor):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        assert buf._initial_fill_until > 0.0

        # Remove material — sets initial_fill_until via _unsync path
        t += 2.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 0)
        assert buf.material_present is False
        assert buf.state == STATE_IDLE

    def test_fill_cancelled_by_feed_button(self, buf, buttons, reactor,
                                            sidecar_moves):
        """Pressing feed button during fill cancels the fill loop."""
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)  # insert material, starts fill
        assert buf._initial_fill_until > 0.0

        # Press feed button — should cancel fill
        buttons.callbacks["PE4"](t + 0.5, 1)
        assert buf._initial_fill_until == 0.0
        assert buf.state == STATE_MANUAL_FEED

        # Fire the fill chunk path directly.  It should see
        # _initial_fill_until == 0.0 and bail without issuing another
        # move.  We don't drain other timers because the manual
        # continuous-feed loop keeps re-scheduling itself while the
        # button is "held" — that's covered by its own tests.
        chunks_before = len(sidecar_moves)
        buf._do_fill_chunk(reactor._monotonic)
        assert len(sidecar_moves) == chunks_before
        assert buf._initial_fill_until == 0.0

    def test_fill_cancelled_by_buffer_feed_command(self, buf, buttons,
                                                    reactor, sidecar_moves):
        """BUFFER_FEED command during fill cancels the fill loop."""
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)  # insert material, starts fill
        assert buf._initial_fill_until > 0.0

        gcmd = MockGcmd("BUFFER_FEED", {"DIST": 50.0})
        buf.cmd_BUFFER_FEED(gcmd)
        assert buf._initial_fill_until == 0.0
        assert buf.state == STATE_MANUAL_FEED
