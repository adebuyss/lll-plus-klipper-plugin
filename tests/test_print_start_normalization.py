"""Tests for print-start FULL normalization.

Filament loading deliberately parks the buffer arm at FULL (continuous
feed auto-stops on sustained full), so a print beginning with a full
arm is the expected state.  Normalization drains it at
start_drain_speed via three stateless triggers:
  1. the Kalico print_stats:start_printing event (start AND resume),
  2. BUFFER_ENABLE while already printing (the recommended START_PRINT
     flow — for file prints print_stats is "printing" throughout the
     macro),
  3. auto-re-sync while printing.
Mid-print drift over-fill keeps the slow 1 mm/s drain.

Register magnitudes (reference config, 489.7 usteps/mm, /0.715):
  5 mm/s (start_drain_speed default) ~ 3424 -> assert 3100..3800
  1 mm/s (mid-print drain)           ~  685 -> assert  600..800
Positive sign = reverse drain (inverted polarity, see
_mm_per_s_to_vactual).
"""

import pytest
from conftest import (
    STATE_FEEDING,
    ZONE_FULL,
    set_sensors,
)


def _start_band(v):
    return 3100 < v < 3800


def _midprint_band(v):
    return 600 < v < 800


class TestStartEventTrigger:
    def test_event_handler_registered(self, buf, printer):
        assert "print_stats:start_printing" in printer.event_handlers

    def test_event_drains_full_at_start(
            self, printing_buf, printer, vactual_writes, reactor):
        set_sensors(printing_buf, full=True)
        printer.send_event("print_stats:start_printing")
        reactor.flush_callbacks()
        assert printing_buf._extreme_recovery_active == ZONE_FULL
        assert vactual_writes[-1] > 0
        assert _start_band(vactual_writes[-1])
        # Normalization does not change the state machine.
        assert printing_buf.state == STATE_FEEDING

    def test_event_is_deferred_not_inline(
            self, printing_buf, printer, vactual_writes):
        """Handlers run inside virtual_sdcard's work_handler dispatch
        and must stay cheap — the drain fires on the next reactor
        iteration, not inline."""
        set_sensors(printing_buf, full=True)
        printer.send_event("print_stats:start_printing")
        assert vactual_writes == []

    def test_event_noop_when_not_printing(
            self, enabled_buf, printer, vactual_writes, reactor):
        """Protects the TestRecoveryGatedOnPrinting invariant: no
        VACTUAL motion outside a print."""
        set_sensors(enabled_buf, full=True)
        printer.send_event("print_stats:start_printing")
        reactor.flush_callbacks()
        assert vactual_writes == []

    def test_event_noop_when_zone_not_full(
            self, printing_buf, printer, vactual_writes, reactor):
        set_sensors(printing_buf, middle=True)
        printer.send_event("print_stats:start_printing")
        reactor.flush_callbacks()
        assert vactual_writes == []
        assert printing_buf._extreme_recovery_active is None

    def test_event_noop_when_unsynced(
            self, printing_buf, printer, vactual_writes, reactor):
        printing_buf._unsync()
        set_sensors(printing_buf, full=True)
        printer.send_event("print_stats:start_printing")
        reactor.flush_callbacks()
        assert vactual_writes == []

    def test_event_noop_when_no_material(
            self, printing_buf, printer, vactual_writes, reactor):
        printing_buf.material_present = False
        set_sensors(printing_buf, full=True)
        printer.send_event("print_stats:start_printing")
        reactor.flush_callbacks()
        assert vactual_writes == []

    def test_event_noop_when_recovery_active(
            self, printing_buf, printer, vactual_writes, reactor):
        """A mid-print 1 mm/s drain already in flight must not be
        clobbered by a resume event."""
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_FULL
        before = len(vactual_writes)
        printer.send_event("print_stats:start_printing")
        reactor.flush_callbacks()
        assert vactual_writes[before:] == []


class TestEnableHookTrigger:
    def test_buffer_enable_while_printing_full_drains_fast(
            self, buf, printer, vactual_writes, reactor):
        """The real macro flow: BUFFER_DISABLE early in START_PRINT,
        BUFFER_ENABLE right before the prime line — with print_stats
        already "printing" and the arm parked at FULL by loading."""
        from conftest import MockGcmd
        printer.print_stats.state = "printing"
        buf.material_present = True
        set_sensors(buf, full=True)
        buf.cmd_BUFFER_ENABLE(MockGcmd())
        assert buf._extreme_recovery_active == ZONE_FULL
        assert _start_band(vactual_writes[-1])

    def test_buffer_enable_not_printing_no_drain(
            self, buf, printer, vactual_writes):
        from conftest import MockGcmd
        buf.material_present = True
        set_sensors(buf, full=True)
        buf.cmd_BUFFER_ENABLE(MockGcmd())
        assert buf._extreme_recovery_active is None
        assert vactual_writes == []


class TestAutoResyncTrigger:
    def test_auto_resync_full_while_printing_uses_start_speed(
            self, printing_buf, vactual_writes):
        printing_buf._unsync()
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(5.0)
        assert printing_buf._synced_to is not None
        assert printing_buf._extreme_recovery_active == ZONE_FULL
        assert _start_band(vactual_writes[-1])


class TestMidPrintDrainUnchanged:
    def test_midprint_full_edge_keeps_1mms(
            self, printing_buf, vactual_writes):
        """A synced buffer drifting into FULL mid-print takes the
        generic entry at 1 mm/s — normalization only owns the
        re-sync/enable/start moments."""
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_FULL
        assert _midprint_band(vactual_writes[-1])

    def test_enter_full_recovery_default_unchanged(
            self, printing_buf, vactual_writes):
        printing_buf._enter_full_recovery(1.0)
        assert _midprint_band(vactual_writes[-1])


class TestStartDrainSpeedConfig:
    def test_config_parsed_and_scales(self, printer):
        import buffer as buffer_module
        from conftest import MockConfig, DEFAULT_CONFIG
        values = dict(DEFAULT_CONFIG)
        values["start_drain_speed"] = 8.0
        b = buffer_module.Buffer(MockConfig(printer, values))
        assert b.start_drain_speed == 8.0

    def test_default_is_five(self, buf):
        assert buf.start_drain_speed == 5.0
        assert buf.get_status(0.0)["start_drain_speed"] == 5.0
