"""Tests for rotation_distance feedback loop (sensor -> multiplier -> rd)."""

import pytest
from conftest import (
    FORWARD,
    STOP,
    STATE_FEEDING,
    STATE_STOPPED,
    STATE_ERROR,
    ZONE_EMPTY,
    ZONE_EMPTY_MIDDLE,
    ZONE_MIDDLE,
    ZONE_FULL_MIDDLE,
    ZONE_FULL,
    set_sensors,
    trigger_sensor,
)


class TestZoneClassification:
    """Verify _compute_zone maps sensor states to zones correctly."""

    def test_middle_only(self, enabled_buf):
        set_sensors(enabled_buf, middle=True)
        assert enabled_buf._compute_zone() == ZONE_MIDDLE

    def test_empty(self, enabled_buf):
        set_sensors(enabled_buf, empty=True)
        assert enabled_buf._compute_zone() == ZONE_EMPTY

    def test_full(self, enabled_buf):
        set_sensors(enabled_buf, full=True)
        assert enabled_buf._compute_zone() == ZONE_FULL

    def test_full_middle(self, enabled_buf):
        set_sensors(enabled_buf, middle=True, full=True)
        assert enabled_buf._compute_zone() == ZONE_FULL_MIDDLE

    def test_empty_middle(self, enabled_buf):
        set_sensors(enabled_buf)  # all off
        assert enabled_buf._compute_zone() == ZONE_EMPTY_MIDDLE

    def test_sensor_conflict(self, enabled_buf):
        set_sensors(enabled_buf, empty=True, full=True)
        assert enabled_buf._compute_zone() is None


class TestMultiplierMapping:
    """Verify _zone_to_multiplier returns correct values."""

    def test_middle_is_exactly_one(self, enabled_buf):
        assert enabled_buf._zone_to_multiplier(ZONE_MIDDLE) == 1.0

    def test_empty_middle_adds_drift_gain(self, enabled_buf):
        m = enabled_buf._zone_to_multiplier(ZONE_EMPTY_MIDDLE)
        assert m == pytest.approx(1.0 + enabled_buf.drift_gain)

    def test_empty_uses_multiplier_high(self, enabled_buf):
        m = enabled_buf._zone_to_multiplier(ZONE_EMPTY)
        assert m == pytest.approx(enabled_buf.multiplier_high)

    def test_full_middle_subtracts_drift_gain(self, enabled_buf):
        m = enabled_buf._zone_to_multiplier(ZONE_FULL_MIDDLE)
        assert m == pytest.approx(1.0 - enabled_buf.drift_gain)

    def test_full_uses_multiplier_low(self, enabled_buf):
        m = enabled_buf._zone_to_multiplier(ZONE_FULL)
        assert m == pytest.approx(enabled_buf.multiplier_low)


class TestDeadBand:
    """The middle sensor should produce exactly multiplier=1.0
    (the dead-band invariant)."""

    def test_middle_sensor_no_correction(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._rd_multiplier == 1.0
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)


class TestRotationDistanceApplication:
    """Verify the multiplier is applied via rd_new = base_rd / multiplier."""

    def test_empty_increases_rd_multiplier(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(1.0)
        expected_mult = enabled_buf.multiplier_high
        expected_rd = base_rd / expected_mult
        assert enabled_buf._rd_multiplier == pytest.approx(expected_mult)
        assert stepper.get_rotation_distance()[0] == pytest.approx(expected_rd)

    def test_full_decreases_rd_multiplier(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf, full=True)
        enabled_buf._update_rotation_distance(1.0)
        expected_mult = enabled_buf.multiplier_low
        expected_rd = base_rd / expected_mult
        assert enabled_buf._rd_multiplier == pytest.approx(expected_mult)
        assert stepper.get_rotation_distance()[0] == pytest.approx(expected_rd)


class TestSensorCallbackUpdatesMultiplier:
    """Verify sensor callbacks trigger rotation_distance updates."""

    def test_sensor_callback_updates_rd(self, enabled_buf, buttons, stepper):
        base_rd = enabled_buf._base_rd
        # Trigger middle sensor
        trigger_sensor(buttons, "PE1", True, 1.0)
        assert enabled_buf._rd_multiplier == 1.0
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)

    def test_sensor_conflict_triggers_error(self, enabled_buf, buttons):
        # Trigger empty first, then full
        trigger_sensor(buttons, "PE0", True, 1.0)
        trigger_sensor(buttons, "PE2", True, 1.0)
        assert enabled_buf.state == STATE_ERROR
        assert "conflict" in enabled_buf.error_msg.lower()


class TestFaultEscalation:
    """After fault_escalation_time in a safety zone, the gain increases."""

    def test_empty_escalation(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(1.0)
        initial_mult = enabled_buf._rd_multiplier

        # Advance past fault_escalation_time
        reactor._monotonic = 1.0 + enabled_buf.fault_escalation_time + 0.1
        enabled_buf._update_rotation_distance(reactor._monotonic)

        # Multiplier should jump to the absolute fault_multiplier_high
        escalated_mult = enabled_buf.fault_multiplier_high
        assert enabled_buf._rd_multiplier == pytest.approx(escalated_mult)
        assert enabled_buf._safety_escalated is True

    def test_escalation_persists_on_subsequent_calls(self, enabled_buf, reactor):
        """Escalated multiplier must persist, not revert to base gain."""
        set_sensors(enabled_buf, empty=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)

        # Escalate
        t += enabled_buf.fault_escalation_time + 0.1
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        escalated_mult = enabled_buf._rd_multiplier
        assert enabled_buf._safety_escalated is True

        # Call again — escalated multiplier must persist
        t += 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        assert enabled_buf._rd_multiplier == pytest.approx(escalated_mult)
        assert enabled_buf._safety_escalated is True

    def test_returning_to_middle_resets_escalation(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(1.0)
        reactor._monotonic = 10.0
        enabled_buf._update_rotation_distance(10.0)

        # Return to middle.  Advance past apply_dwell so the transition
        # isn't deferred by short-cycle protection.
        reactor._monotonic = 10.0 + enabled_buf.apply_dwell
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(reactor._monotonic)
        assert enabled_buf._safety_escalated is False
        assert enabled_buf._rd_multiplier == 1.0


class TestFlushBeforeSetRotationDistance:
    """Regression for MCU 'Rescheduled timer in the past' shutdown:
    set_rotation_distance must be preceded by flush_step_generation,
    matching upstream Klipper's cmd_SET_E_ROTATION_DISTANCE."""

    def test_repeat_same_multiplier_skips_set(self, enabled_buf, stepper):
        baseline = len(stepper.rd_log)
        enabled_buf._apply_multiplier(1.0)
        enabled_buf._apply_multiplier(1.0)
        enabled_buf._apply_multiplier(1.0)
        assert len(stepper.rd_log) == baseline

    def test_each_change_flushes_before_set(
            self, enabled_buf, stepper, printer):
        # Validate the flush-before-set invariant directly by disabling
        # dwell coalescing (TestApplyDwellCoalescing covers throttling).
        enabled_buf.apply_dwell = 0.0
        toolhead = printer.toolhead
        flushes_before = toolhead.flush_count
        sets_before = len(stepper.rd_log)
        enabled_buf._apply_multiplier(enabled_buf.multiplier_high)
        enabled_buf._apply_multiplier(enabled_buf.fault_multiplier_high)
        assert len(stepper.rd_log) == sets_before + 2
        assert toolhead.flush_count == flushes_before + 2


class TestUnsyncRestoresBaseRotationDistance:
    """After _unsync, the stepper's rotation_distance must be restored
    to _base_rd so any subsequent force_move.manual_move computes the
    right number of steps.  Without this, BUFFER_FEED/BUFFER_RETRACT
    issued after leaving a non-MIDDLE zone would move the wrong amount
    of filament (last zone's multiplier would apply)."""

    def test_unsync_from_empty_restores_base(
            self, enabled_buf, stepper, printer):
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(1.0)
        assert stepper.get_rotation_distance()[0] == pytest.approx(
            base_rd / enabled_buf.multiplier_high)

        flushes_before = printer.toolhead.flush_count
        enabled_buf._unsync()
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)
        assert enabled_buf._rd_multiplier == 1.0
        # flush must have fired at least once before the restore set
        assert printer.toolhead.flush_count >= flushes_before + 1

    def test_unsync_from_full_restores_base(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf, full=True)
        enabled_buf._update_rotation_distance(1.0)
        assert stepper.get_rotation_distance()[0] == pytest.approx(
            base_rd / enabled_buf.multiplier_low)
        enabled_buf._unsync()
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)

    def test_unsync_from_fault_escalation_restores_base(
            self, enabled_buf, stepper, reactor):
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf, empty=True)
        reactor._monotonic = 1.0
        enabled_buf._update_rotation_distance(1.0)
        reactor._monotonic = 1.0 + enabled_buf.fault_escalation_time + 0.1
        enabled_buf._update_rotation_distance(reactor._monotonic)
        assert enabled_buf._rd_multiplier == pytest.approx(
            enabled_buf.fault_multiplier_high)

        enabled_buf._unsync()
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)

    def test_resync_after_unsync_in_empty_reapplies_multiplier(
            self, enabled_buf, stepper, reactor):
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(1.0)
        assert stepper.get_rotation_distance()[0] == pytest.approx(
            base_rd / enabled_buf.multiplier_high)

        enabled_buf._unsync()
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)

        # Re-sync still in EMPTY — next control tick must re-apply
        # the zone multiplier (dedupe must NOT suppress the write).
        enabled_buf._sync()
        reactor._monotonic = 2.0
        enabled_buf._update_rotation_distance(2.0)
        assert stepper.get_rotation_distance()[0] == pytest.approx(
            base_rd / enabled_buf.multiplier_high)


class TestApplyDwellCoalescing:
    """Short-cycle protection: zone oscillation within apply_dwell of the
    last applied change is coalesced into a single deferred apply.

    Each apply does flush_step_generation(), which drains the entire step
    pipeline (XYZ + extruder).  Without coalescing, MIDDLE <-> EMPTY_MIDDLE
    thrash at a sensor edge produces a flush per transition.  With dwell,
    the first transition applies immediately, subsequent changes are held
    as _pending_multiplier, and a reactor timer applies the latest value
    once the dwell window expires.  Fault escalation bypasses dwell."""

    def test_thrash_coalesces_to_one_flush_per_window(
            self, enabled_buf, reactor, printer):
        """5 zone toggles within the dwell window should produce exactly
        one immediate flush (the first transition); the deferred timer
        either applies the final value or short-circuits via dedup."""
        toolhead = printer.toolhead
        # Settle MIDDLE so _last_apply_time is established at a known time.
        reactor._monotonic = 1.0
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(1.0)
        # Advance past dwell so the first toggle below applies immediately.
        reactor._monotonic = 1.0 + enabled_buf.apply_dwell
        flushes_before = toolhead.flush_count

        # Burst: MIDDLE -> EMPTY_MIDDLE -> MIDDLE -> EMPTY_MIDDLE -> MIDDLE,
        # all within 100 ms (well under the 500 ms dwell).
        t = reactor._monotonic
        for i, sensors in enumerate([
                {},  # EMPTY_MIDDLE (all off)
                {"middle": True},  # MIDDLE
                {},  # EMPTY_MIDDLE
                {"middle": True},  # MIDDLE
        ]):
            t += 0.02
            reactor._monotonic = t
            set_sensors(enabled_buf, **sensors)
            enabled_buf._update_rotation_distance(t)

        # Exactly one flush during the burst (the first transition into
        # EMPTY_MIDDLE).  Subsequent changes are within dwell -> pending.
        assert toolhead.flush_count == flushes_before + 1
        # Final state is MIDDLE (multiplier 1.0); since 1.0 was the
        # _last_applied multiplier before the burst started, the dwell
        # timer will see pending == _rd_multiplier and short-circuit.
        # However, _rd_multiplier is currently 1.0 + drift_gain because
        # the first transition (into EMPTY_MIDDLE) applied it.
        assert enabled_buf._rd_multiplier == pytest.approx(
            1.0 + enabled_buf.drift_gain)
        # Pending value is the final intended (MIDDLE -> 1.0)
        assert enabled_buf._pending_multiplier == pytest.approx(1.0)

        # Advance past the dwell window — the deferred apply fires.
        reactor.advance_time(enabled_buf.apply_dwell + 0.01)
        # Now the latest pending (1.0) has been applied.
        assert enabled_buf._rd_multiplier == 1.0
        assert enabled_buf._pending_multiplier is None
        # Total: one immediate + one deferred = exactly two flushes.
        assert toolhead.flush_count == flushes_before + 2

    def test_within_dwell_change_is_deferred_not_dropped(
            self, enabled_buf, reactor, printer):
        """A second distinct multiplier within dwell must still arrive
        at the stepper — the latest pending value is applied by the
        timer when the window expires."""
        toolhead = printer.toolhead
        stepper = enabled_buf.extruder_stepper.stepper
        base_rd = enabled_buf._base_rd

        # Establish baseline at MIDDLE.
        reactor._monotonic = 1.0
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(1.0)
        reactor._monotonic = 1.0 + enabled_buf.apply_dwell

        # First transition: MIDDLE -> EMPTY (applies immediately).
        t = reactor._monotonic
        reactor._monotonic = t + 0.01
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(reactor._monotonic)
        assert enabled_buf._rd_multiplier == pytest.approx(
            enabled_buf.multiplier_high)
        flushes_after_first = toolhead.flush_count

        # Second transition within dwell: EMPTY -> EMPTY_MIDDLE (deferred).
        reactor._monotonic += 0.05
        set_sensors(enabled_buf)  # all off -> EMPTY_MIDDLE
        enabled_buf._update_rotation_distance(reactor._monotonic)
        # Not yet applied; held as pending.
        assert enabled_buf._rd_multiplier == pytest.approx(
            enabled_buf.multiplier_high)
        assert enabled_buf._pending_multiplier == pytest.approx(
            1.0 + enabled_buf.drift_gain)
        assert toolhead.flush_count == flushes_after_first

        # Fire the dwell timer.
        reactor.advance_time(enabled_buf.apply_dwell + 0.01)
        assert enabled_buf._rd_multiplier == pytest.approx(
            1.0 + enabled_buf.drift_gain)
        assert enabled_buf._pending_multiplier is None
        assert stepper.get_rotation_distance()[0] == pytest.approx(
            base_rd / (1.0 + enabled_buf.drift_gain))
        assert toolhead.flush_count == flushes_after_first + 1

    def test_fault_escalation_bypasses_dwell(
            self, enabled_buf, reactor, printer):
        """Safety escalation must apply immediately even within dwell."""
        toolhead = printer.toolhead
        # Enter EMPTY at t=1.0 (applies multiplier_high, _last_apply_time=1.0).
        set_sensors(enabled_buf, empty=True)
        reactor._monotonic = 1.0
        enabled_buf._update_rotation_distance(1.0)
        flushes_before = toolhead.flush_count

        # Advance past fault_escalation_time but stay within apply_dwell of
        # _last_apply_time (i.e. eventtime - _last_apply_time would be
        # > fault_escalation_time but < apply_dwell after the next apply).
        # Here we just verify force=True takes effect: the escalation tick
        # arrives at any time after fault_escalation_time elapsed.
        t = 1.0 + enabled_buf.fault_escalation_time + 0.1
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)

        # Escalated value applied despite dwell: force=True path.
        assert enabled_buf._safety_escalated is True
        assert enabled_buf._rd_multiplier == pytest.approx(
            enabled_buf.fault_multiplier_high)
        assert toolhead.flush_count == flushes_before + 1

    def test_unsync_cancels_pending_apply(
            self, enabled_buf, reactor, printer):
        """Pending multiplier and dwell timer must be cleaned up on
        _unsync — a stale pending would otherwise fire on the next
        timer tick after the stepper is no longer being driven."""
        toolhead = printer.toolhead

        # Establish baseline.
        reactor._monotonic = 1.0
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(1.0)
        reactor._monotonic = 1.0 + enabled_buf.apply_dwell

        # First transition applies; second within dwell defers.
        t = reactor._monotonic + 0.01
        reactor._monotonic = t
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(t)
        reactor._monotonic = t + 0.05
        set_sensors(enabled_buf)  # EMPTY_MIDDLE
        enabled_buf._update_rotation_distance(reactor._monotonic)
        assert enabled_buf._pending_multiplier is not None

        flushes_before_unsync = toolhead.flush_count
        enabled_buf._unsync()
        assert enabled_buf._pending_multiplier is None
        # Advancing past dwell must not trigger a deferred apply: the
        # stepper is unsynced, so the callback bails and rd is unchanged.
        # (The unsync itself flushes via sync_to_extruder(None) in the
        # mock, so flush_count increments by exactly one.)
        rd_before = enabled_buf.extruder_stepper.stepper.get_rotation_distance()[0]
        reactor.advance_time(enabled_buf.apply_dwell + 0.01)
        rd_after = enabled_buf.extruder_stepper.stepper.get_rotation_distance()[0]
        assert rd_before == pytest.approx(rd_after)
        assert toolhead.flush_count == flushes_before_unsync + 1

    def test_dwell_zero_disables_coalescing(
            self, enabled_buf, reactor, printer):
        """apply_dwell=0 -> every transition flushes immediately
        (legacy behavior, opt-out)."""
        enabled_buf.apply_dwell = 0.0
        toolhead = printer.toolhead

        reactor._monotonic = 1.0
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(1.0)
        flushes_before = toolhead.flush_count

        # Two distinct transitions back-to-back at the same eventtime.
        reactor._monotonic = 1.01
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(reactor._monotonic)
        reactor._monotonic = 1.02
        set_sensors(enabled_buf)  # EMPTY_MIDDLE
        enabled_buf._update_rotation_distance(reactor._monotonic)

        # Both applied immediately; pending should never have been set.
        assert toolhead.flush_count == flushes_before + 2
        assert enabled_buf._pending_multiplier is None
        assert enabled_buf._rd_multiplier == pytest.approx(
            1.0 + enabled_buf.drift_gain)
