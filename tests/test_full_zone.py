"""Tests for full zone safety timeout and retraction logic."""

import pytest
from conftest import (
    ZONE_FULL,
    ZONE_FULL_MIDDLE,
    STATE_RETRACTING,
    STATE_STOPPED,
    set_sensors,
)


class TestFullZoneEntry:
    def test_entering_full_zone_starts_safety_timer(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        assert enabled_buf._current_zone == ZONE_FULL
        assert enabled_buf._safety_zone_start == t

    def test_leaving_full_resets_safety(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        assert enabled_buf._safety_zone_start == t

        # Leave to middle
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(11.0)
        assert enabled_buf._safety_zone_start == 0.0


class TestFullZoneTimeout:
    def test_timeout_triggers_safety_retract(self, enabled_buf, reactor,
                                              sidecar_moves):
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, full=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)

        t += enabled_buf.full_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert len(sidecar_moves) > 0
        assert sidecar_moves[-1][1] < 0  # retract = negative dist


class TestSafetyRetractBehavior:
    def test_safety_retract_leaves_unsynced(self, enabled_buf, reactor,
                                             sidecar_moves):
        """After safety retract, stepper must remain unsynced so the
        retract move completes before re-sync on next timer cycle."""
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, full=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)

        t += enabled_buf.full_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert len(sidecar_moves) > 0
        assert enabled_buf._synced_to is None


class TestFullMiddleOverlap:
    def test_full_middle_does_not_trigger_safety(self, enabled_buf, reactor):
        set_sensors(enabled_buf, full=True, middle=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        assert enabled_buf._current_zone == ZONE_FULL_MIDDLE
        # Safety timer should not be active for FULL_MIDDLE
        assert enabled_buf._safety_zone_start == 0.0

    def test_transition_full_to_full_middle_clears_safety(self, enabled_buf,
                                                           reactor):
        set_sensors(enabled_buf, full=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        assert enabled_buf._safety_zone_start == t

        # Transition to FULL_MIDDLE
        set_sensors(enabled_buf, full=True, middle=True)
        enabled_buf._update_rotation_distance(11.0)
        assert enabled_buf._current_zone == ZONE_FULL_MIDDLE
        assert enabled_buf._safety_zone_start == 0.0


class TestFullEnteredBeforePrinting:
    """Regression: the buffer can transition into FULL during the prime
    line / start macro (while _print_stats is still 'standby').  The
    zone-entry edge sets _safety_zone_start, the next not-printing tick
    of _control_timer_cb clears it to 0, and then no further sensor edge
    fires once printing actually starts because the sensor is steady at
    FULL.  The state-based fallback in _update_rotation_distance must
    re-arm the timer once _is_printing() flips to True."""

    def test_full_entered_before_printing_still_arms(self, enabled_buf,
                                                      reactor, sidecar_moves):
        # 1. Buffer hits FULL while not printing (e.g. prime line)
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, full=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        # Edge-based arming fired
        assert enabled_buf._safety_zone_start == t

        # 2. Control timer ticks while still not printing -> clears to 0
        t = 2.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert enabled_buf._safety_zone_start == 0.0

        # 3. Print actually starts.  No new sensor edge (steady FULL).
        enabled_buf._print_stats.state = "printing"

        # 4. Next control timer tick should re-arm via the state-based
        #    fallback in _update_rotation_distance and use the *current*
        #    eventtime (not the stale earlier edge time).
        t = 3.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert enabled_buf._safety_zone_start == t

        # 5. Advancing past full_safety_timeout fires the safety retract.
        t = 3.0 + enabled_buf.full_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert len(sidecar_moves) > 0
        assert sidecar_moves[-1][1] < 0  # negative dist = retract

    def test_state_arming_does_not_fire_when_not_printing(self,
                                                            enabled_buf,
                                                            reactor):
        """The state-based fallback must be gated on _is_printing() so
        the be70969 intent (no false fault from heat-up) is preserved."""
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, full=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        # Edge fires once
        assert enabled_buf._safety_zone_start == t
        # Not-printing tick clears
        t = 2.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert enabled_buf._safety_zone_start == 0.0
        # Subsequent _update_rotation_distance call (e.g. another
        # control tick) must NOT re-arm while still not printing
        t = 3.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        assert enabled_buf._safety_zone_start == 0.0


class TestSafetyRetractCompletion:
    """RETRACTING is no longer terminal: the shared completion timer
    restores STOPPED after the move duration, and the control timer's
    unconditional update call re-syncs (STOPPED -> FEEDING) within one
    control_interval."""

    def test_retracting_becomes_stopped_after_move_duration(
            self, printing_buf, reactor):
        """Completion step in isolation: fire the done callback
        directly (no control tick in between) — unsynced completion
        lands at STOPPED."""
        from conftest import STOP
        reactor._monotonic = 10.0
        printing_buf._do_safety_retract(10.0)
        assert printing_buf.state == STATE_RETRACTING
        assert printing_buf._synced_to is None
        printing_buf._timed_move_done_cb(10.4)  # > ~0.28 s duration
        assert printing_buf.state == STATE_STOPPED
        assert printing_buf.motor_direction == STOP

    def test_full_pipeline_resyncs_after_safety_retract(
            self, printing_buf, reactor):
        """End to end via the reactor: done timer AND control timer
        both fire during the advance — RETRACTING resolves all the
        way back to synced FEEDING with no manual intervention."""
        from conftest import STATE_FEEDING
        reactor._monotonic = 10.0
        printing_buf._do_safety_retract(10.0)
        reactor.advance_time(0.4)
        assert printing_buf._synced_to is not None
        assert printing_buf.state == STATE_FEEDING

    def test_retracting_promotes_to_feeding_if_resynced_mid_move(
            self, printing_buf, reactor):
        from conftest import STATE_FEEDING
        reactor._monotonic = 10.0
        printing_buf._do_safety_retract(10.0)
        # A control tick lands mid-move (control_interval < move
        # duration in non-default configs) and re-syncs first.
        printing_buf._update_rotation_distance(10.1)
        assert printing_buf._synced_to is not None
        assert printing_buf.state == STATE_RETRACTING
        reactor.advance_time(0.4)
        # _sync only promotes on the STOPPED edge, so the completion
        # callback must promote directly.
        assert printing_buf.state == STATE_FEEDING
