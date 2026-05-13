"""Tests for the sidecar trapq helper.

The buffer-stepper chunk loop must NOT call toolhead.dwell() (which
would advance main toolhead print_time and inflate the gcode lookahead).
Every chunk site routes through Buffer._sidecar_move, which queues the
move on a private trapq and reports MCU queue activity via
note_mcu_movequeue_activity instead.
"""

from conftest import (
    BACK,
    FORWARD,
    STATE_MANUAL_FEED,
    STATE_MANUAL_RETRACT,
    STATE_RETRACTING,
    ZONE_EMPTY,
    ZONE_FULL,
    MockGcmd,
    set_sensors,
)


def _drive_into_empty_recovery(printing_buf, reactor):
    """Push the buffer into EMPTY-zone recovery so the recovery
    chunk loop fires.  Mirrors the setup other recovery tests use."""
    reactor._monotonic = 1.0
    set_sensors(printing_buf, empty=True)
    printing_buf._update_rotation_distance(1.0)


class TestSidecarNoToolheadDwell:
    """Acceptance #1: no chunk site may call toolhead.dwell()."""

    def test_recovery_chunk_no_dwell(self, printing_buf, reactor):
        _drive_into_empty_recovery(printing_buf, reactor)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
        assert printing_buf.toolhead.dwell_calls == []

    def test_initial_fill_no_dwell(self, buf, buttons, reactor):
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 1)  # material insert
        assert buf.toolhead.dwell_calls == []

    def test_safety_retract_no_dwell(self, enabled_buf, reactor):
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, full=True)
        reactor._monotonic = 10.0
        enabled_buf._update_rotation_distance(10.0)
        reactor._monotonic = 10.0 + enabled_buf.full_safety_timeout + 1.0
        enabled_buf._control_timer_cb(reactor._monotonic)
        assert enabled_buf.toolhead.dwell_calls == []
        assert enabled_buf.state == STATE_RETRACTING

    def test_continuous_feed_no_dwell(self, buf, reactor):
        reactor._monotonic = 1.0
        buf.cmd_BUFFER_FEED(MockGcmd("BUFFER_FEED"))
        assert buf.toolhead.dwell_calls == []
        assert buf.state == STATE_MANUAL_FEED

    def test_retract_until_clear_no_dwell(self, buf, reactor):
        buf.material_present = True
        reactor._monotonic = 1.0
        buf.cmd_BUFFER_RETRACT_UNTIL_CLEAR(MockGcmd("BUFFER_RETRACT_UNTIL_CLEAR"))
        assert buf.toolhead.dwell_calls == []
        assert buf.state == STATE_MANUAL_RETRACT

    def test_buffer_feed_dist_no_dwell(self, buf):
        buf.cmd_BUFFER_FEED(MockGcmd("BUFFER_FEED", {"DIST": 50.0}))
        assert buf.toolhead.dwell_calls == []

    def test_buffer_retract_dist_no_dwell(self, buf):
        buf.cmd_BUFFER_RETRACT(MockGcmd("BUFFER_RETRACT", {"DIST": 50.0}))
        assert buf.toolhead.dwell_calls == []


class TestSidecarNoLastMoveTimeAdvance:
    """Acceptance #2: chunked motion must not advance the main
    toolhead's last_move_time."""

    def test_recovery_does_not_advance_last_move_time(
            self, printing_buf, reactor):
        baseline = printing_buf.toolhead._last_move_time
        _drive_into_empty_recovery(printing_buf, reactor)
        # Fire several recovery chunks by advancing time past each.
        for _ in range(5):
            reactor.advance_time(0.3)
        assert printing_buf.toolhead._last_move_time == baseline

    def test_initial_fill_does_not_advance_last_move_time(
            self, buf, buttons, reactor):
        baseline = buf.toolhead._last_move_time
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 1)
        for _ in range(5):
            reactor.advance_time(0.3)
        assert buf.toolhead._last_move_time == baseline

    def test_continuous_feed_does_not_advance_last_move_time(
            self, buf, reactor):
        baseline = buf.toolhead._last_move_time
        reactor._monotonic = 1.0
        buf.cmd_BUFFER_FEED(MockGcmd("BUFFER_FEED"))
        for _ in range(5):
            reactor.advance_time(0.3)
        assert buf.toolhead._last_move_time == baseline


class TestSidecarReportsMovequeueActivity:
    """The helper must report MCU queue activity so backpressure
    tracking still works — even though it skips the dwell."""

    def test_chunk_reports_movequeue_activity(self, buf):
        # In test mode the trapq is None so the real path is skipped,
        # but the throttle bookkeeping still runs.  We assert the helper
        # at least produced a sidecar_moves entry — the
        # note_mcu_movequeue_activity call lives behind the chelper
        # path and isn't reachable in test mode without an FFI mock.
        before = len(buf.sidecar_moves)
        ok = buf._sidecar_move(5.0, 40.0, 1500.0)
        assert ok is True
        assert len(buf.sidecar_moves) == before + 1


class TestSidecarThrottle:
    """Acceptance #4: only one chunk in flight at a time."""

    def test_throttle_blocks_until_chunk_finishes(self, buf, reactor):
        reactor._monotonic = 0.0
        buf.toolhead._mcu.set_estimated_print_time(0.0)
        # First call queues; _sidecar_next_cmd_time advances past 0.
        assert buf._sidecar_move(5.0, 40.0, 1500.0) is True
        assert buf._sidecar_next_cmd_time > 0.0
        # Second call: previous chunk still in flight (mcu pinned at 0).
        assert buf._sidecar_move(5.0, 40.0, 1500.0) is False

    def test_throttle_releases_after_chunk_completes(self, buf, reactor):
        reactor._monotonic = 0.0
        buf.toolhead._mcu.set_estimated_print_time(0.0)
        assert buf._sidecar_move(5.0, 40.0, 1500.0) is True
        assert buf._sidecar_move(5.0, 40.0, 1500.0) is False
        # MCU catches up past the queued chunk's tail.
        buf.toolhead._mcu.set_estimated_print_time(
            buf._sidecar_next_cmd_time + 0.01)
        assert buf._sidecar_move(5.0, 40.0, 1500.0) is True


class TestSidecarBurst:
    """Risk-register: MCU step-queue depth.  A rapid burst of
    _sidecar_move calls without time advancement must throttle to
    one queued chunk, not 1000."""

    def test_burst_queues_only_one(self, buf, reactor):
        reactor._monotonic = 0.0
        buf.toolhead._mcu.set_estimated_print_time(0.0)
        results = [buf._sidecar_move(5.0, 40.0, 1500.0)
                   for _ in range(1000)]
        accepted = sum(1 for r in results if r)
        assert accepted == 1


class TestSidecarPacing:
    """_sidecar_next_chunk_eventtime returns the eventtime at which the
    previous chunk completes (plus a small margin), used by every site
    to schedule the next attempt."""

    def test_pacing_after_queue(self, buf, reactor):
        # Simulate the in-flight state directly: MCU is at print_time 5.0
        # and the tail of our queued motion is at 5.15 (~150ms ahead).
        # _sidecar_next_chunk_eventtime should compute now + (tail - est)
        # + margin.
        reactor._monotonic = 5.0
        buf.toolhead._mcu.set_estimated_print_time(5.0)
        buf._sidecar_next_cmd_time = 5.15
        wake = buf._sidecar_next_chunk_eventtime()
        assert wake > 5.05
        assert wake < 5.25

    def test_pacing_when_idle(self, buf, reactor):
        reactor._monotonic = 5.0
        buf.toolhead._mcu.set_estimated_print_time(5.0)
        wake = buf._sidecar_next_chunk_eventtime()
        # No chunk in flight: wake is now + a small margin only.
        assert wake >= 5.0
        assert wake < 5.01


class TestSidecarMockHygiene:
    """Coverage for the mock changes themselves."""

    def test_mock_mcu_default_returns_eventtime(self, buf, reactor):
        assert buf.toolhead._mcu.estimated_print_time(7.5) == 7.5

    def test_mock_dwell_records_and_advances(self, buf):
        buf.toolhead.dwell(0.1)
        assert buf.toolhead.dwell_calls == [0.1]
        assert buf.toolhead._last_move_time == 0.1

    def test_mock_movequeue_activity_does_not_advance(self, buf):
        buf.toolhead.note_mcu_movequeue_activity(5.0)
        assert buf.toolhead.note_mcu_movequeue_activity_calls == [5.0]
        assert buf.toolhead._last_move_time == 0.0


class TestForceMoveRegressionGuard:
    """If a future change accidentally re-introduces a
    force_move.manual_move call, this catches it."""

    def test_recovery_does_not_use_force_move(
            self, printing_buf, reactor, force_move):
        _drive_into_empty_recovery(printing_buf, reactor)
        assert force_move.moves == []

    def test_initial_fill_does_not_use_force_move(
            self, buf, buttons, reactor, force_move):
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 1)
        assert force_move.moves == []

    def test_continuous_feed_does_not_use_force_move(
            self, buf, reactor, force_move):
        buf.cmd_BUFFER_FEED(MockGcmd("BUFFER_FEED"))
        assert force_move.moves == []
