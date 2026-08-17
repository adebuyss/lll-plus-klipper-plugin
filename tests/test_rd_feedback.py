"""Tests for rotation_distance feedback loop (sensor -> multiplier -> rd)
and VACTUAL-based extreme-zone recovery."""

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


def _apply_zone_with_hysteresis(buf, zone_setter, t0=1.0):
    """Helper: apply a zone twice with >200ms gap so the hysteresis
    gate in _update_rotation_distance commits the multiplier.  The
    plain-zone hysteresis is mandatory for non-extreme zones — the
    first call records the proposed zone, the second (after 200ms)
    commits it.  Extreme zones (EMPTY/FULL) are not hysteresis-gated;
    they enter recovery on the first call."""
    zone_setter()
    buf._update_rotation_distance(t0)
    buf._update_rotation_distance(t0 + 0.3)


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

    def test_full_middle_subtracts_drift_gain(self, enabled_buf):
        m = enabled_buf._zone_to_multiplier(ZONE_FULL_MIDDLE)
        assert m == pytest.approx(1.0 - enabled_buf.drift_gain)

    def test_extreme_zones_return_one(self, enabled_buf):
        """Extreme zones go through VACTUAL recovery, not multiplier.
        The mapping returns 1.0 so the apply path is a no-op even
        if the recovery branch is bypassed."""
        assert enabled_buf._zone_to_multiplier(ZONE_EMPTY) == 1.0
        assert enabled_buf._zone_to_multiplier(ZONE_FULL) == 1.0


class TestDeadBand:
    """The middle sensor should produce exactly multiplier=1.0
    (the dead-band invariant)."""

    def test_middle_sensor_no_correction(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        _apply_zone_with_hysteresis(
            enabled_buf, lambda: set_sensors(enabled_buf, middle=True))
        assert enabled_buf._rd_multiplier == 1.0
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)


class TestRotationDistanceApplication:
    """Verify the multiplier is applied via rd_new = base_rd / multiplier
    for the non-extreme drift_gain zones (after hysteresis settles)."""

    def test_empty_middle_applies_drift_gain(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        _apply_zone_with_hysteresis(
            enabled_buf, lambda: set_sensors(enabled_buf))  # EMPTY_MIDDLE
        expected_mult = 1.0 + enabled_buf.drift_gain
        expected_rd = base_rd / expected_mult
        assert enabled_buf._rd_multiplier == pytest.approx(expected_mult)
        assert stepper.get_rotation_distance()[0] == pytest.approx(expected_rd)

    def test_full_middle_applies_drift_gain(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        _apply_zone_with_hysteresis(
            enabled_buf,
            lambda: set_sensors(enabled_buf, full=True, middle=True))
        expected_mult = 1.0 - enabled_buf.drift_gain
        expected_rd = base_rd / expected_mult
        assert enabled_buf._rd_multiplier == pytest.approx(expected_mult)
        assert stepper.get_rotation_distance()[0] == pytest.approx(expected_rd)


class TestZoneHysteresis:
    """A non-extreme zone must be stable for 200ms before the multiplier
    commits.  Prevents sensor-bounce thrash at zone boundaries."""

    def test_first_call_does_not_apply(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf)  # EMPTY_MIDDLE
        enabled_buf._update_rotation_distance(1.0)
        # First call records the proposed zone but does not apply.
        assert enabled_buf._rd_multiplier == 1.0
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)

    def test_bounce_resets_proposed_zone(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        # Propose EMPTY_MIDDLE
        set_sensors(enabled_buf)
        enabled_buf._update_rotation_distance(1.0)
        # Bounce to MIDDLE before hysteresis settles
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(1.1)
        # Still no change — both attempts were under 200ms each.
        assert enabled_buf._rd_multiplier == 1.0
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)


class TestSensorCallbackUpdatesMultiplier:
    """Verify sensor callbacks trigger rotation_distance updates."""

    def test_sensor_callback_updates_rd(self, enabled_buf, buttons, stepper):
        base_rd = enabled_buf._base_rd
        # Trigger middle sensor twice across hysteresis window
        trigger_sensor(buttons, "PE1", True, 1.0)
        trigger_sensor(buttons, "PE1", True, 1.3)
        # MIDDLE -> multiplier 1.0 (same as baseline) -> no rd change
        assert enabled_buf._rd_multiplier == 1.0
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)

    def test_sensor_conflict_triggers_error(self, enabled_buf, buttons):
        # Trigger empty first, then full
        trigger_sensor(buttons, "PE0", True, 1.0)
        trigger_sensor(buttons, "PE2", True, 1.0)
        # Initial conflict is deferred (could be transient out-of-order
        # MCU report).  Persistent conflict promoted by control timer.
        assert enabled_buf.state != STATE_ERROR
        enabled_buf._control_timer_cb(1.0 + enabled_buf.control_interval)
        assert enabled_buf.state == STATE_ERROR
        assert "conflict" in enabled_buf.error_msg.lower()


class TestApplyMultiplierDedup:
    """_apply_multiplier should skip the stepper write when the requested
    multiplier matches the current value."""

    def test_repeat_same_multiplier_skips_set(self, enabled_buf, stepper):
        baseline = len(stepper.rd_log)
        enabled_buf._apply_multiplier(1.0)
        enabled_buf._apply_multiplier(1.0)
        enabled_buf._apply_multiplier(1.0)
        assert len(stepper.rd_log) == baseline


class TestApplyMultiplierRateLimit:
    """During printing, _apply_multiplier rate-limits to 2 Hz to prevent
    sensor-bounce-induced stacked no-flush rotation_distance writes.
    The deferred update lands on the next control timer tick."""

    def test_rate_limit_defers_during_print(
            self, printing_buf, stepper, reactor):
        baseline = len(stepper.rd_log)
        reactor._monotonic = 1.0
        printing_buf._apply_multiplier(1.05)
        assert len(stepper.rd_log) == baseline + 1
        # Second call within 500ms — should defer.
        reactor._monotonic = 1.1
        printing_buf._apply_multiplier(1.10)
        assert len(stepper.rd_log) == baseline + 1
        assert printing_buf._pending_multiplier == 1.10

    def test_control_timer_drains_pending(
            self, printing_buf, stepper, reactor):
        baseline = len(stepper.rd_log)
        reactor._monotonic = 1.0
        printing_buf._apply_multiplier(1.05)
        reactor._monotonic = 1.1
        printing_buf._apply_multiplier(1.10)
        assert printing_buf._pending_multiplier == 1.10
        # Advance past the 500ms rate-limit window and tick the
        # control timer.
        reactor._monotonic = 1.6
        printing_buf._control_timer_cb(1.6)
        assert printing_buf._pending_multiplier is None
        assert printing_buf._rd_multiplier == pytest.approx(1.10)


class TestUnsyncRestoresBaseRotationDistance:
    """After _unsync, the stepper's rotation_distance must be restored
    to _base_rd so any subsequent force_move.manual_move computes the
    right number of steps."""

    def test_unsync_from_drift_zone_restores_base(
            self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        _apply_zone_with_hysteresis(
            enabled_buf, lambda: set_sensors(enabled_buf))  # EMPTY_MIDDLE
        assert stepper.get_rotation_distance()[0] == pytest.approx(
            base_rd / (1.0 + enabled_buf.drift_gain))

        enabled_buf._unsync()
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)
        assert enabled_buf._rd_multiplier == 1.0


class TestVactualRecoveryEntry:
    """Entering an extreme zone while printing writes the VACTUAL
    register.  The stepper stays nominally synced — only the chip-side
    velocity mode is engaged."""

    def test_empty_entry_writes_forward_vactual(
            self, printing_buf, vactual_writes):
        # NB: VACTUAL polarity is inverted on the reference hardware
        # (see _mm_per_s_to_vactual).  Forward filament motion is
        # encoded as a NEGATIVE VACTUAL register value; the formula
        # negates internally so the magnitude reads from
        # +recovery_speed.
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
        # Stepper stays synced — VACTUAL bypasses STEP/DIR at the chip.
        assert printing_buf._synced_to is not None
        assert len(vactual_writes) >= 1
        # Forward feed -> negative VACTUAL (inverted polarity).
        assert vactual_writes[-1] < 0
        # Magnitude check: ~10 mm/s default × 489.4 / 0.715 ≈ 6,845.
        # Allow ±10% for rounding / step_dist variance.
        assert 6000 < abs(vactual_writes[-1]) < 7700

    def test_empty_entry_respects_recovery_speed_config(
            self, printing_buf, vactual_writes):
        # Override recovery_speed; recovery VACTUAL magnitude scales.
        # manual_speed stays at 40 mm/s (force_move.manual_move uses
        # it with a trapezoid ramp and that's fine).
        printing_buf.recovery_speed = 15.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        # ~15 mm/s × 489.4 / 0.715 ≈ 10,267.
        assert 9500 < abs(vactual_writes[-1]) < 11000

    def test_full_entry_writes_reverse_vactual(
            self, printing_buf, vactual_writes):
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_FULL
        # Slow reverse drain -> positive VACTUAL (inverted polarity).
        assert len(vactual_writes) >= 1
        assert vactual_writes[-1] > 0
        # Magnitude check: ~1 mm/s ≈ steps_per_mm/0.715 ≈ 685 LSB.
        # Allow ±10% slack for rounding.
        assert 600 < abs(vactual_writes[-1]) < 800


class TestVactualRecoveryExit:
    """On zone return, recovery writes VACTUAL=0 to stop the chip-side
    velocity mode.  Both sensor-callback path and polling-timer path
    should trigger this."""

    def test_zone_return_writes_vactual_zero_via_callback(
            self, printing_buf, vactual_writes, reactor):
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        # Forward feed active -> negative VACTUAL (inverted polarity).
        assert vactual_writes[-1] < 0

        # Zone progresses to MIDDLE — callback path stops VACTUAL.
        set_sensors(printing_buf, middle=True)
        reactor._monotonic = 2.0
        printing_buf._update_rotation_distance(2.0)
        assert printing_buf._extreme_recovery_active is None
        # _stop_recovery_vactual writes 0 then a positive latch-correction
        # nudge; the final 0 is deferred ~50 ms via register_callback.
        assert 0 in vactual_writes
        assert vactual_writes[-1] > 0
        reactor.flush_callbacks()
        assert vactual_writes[-1] == 0

    def test_zone_return_writes_vactual_zero_via_poller(
            self, printing_buf, vactual_writes, reactor):
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        # Zone returns without sensor-callback triggering update_rd.
        set_sensors(printing_buf, middle=True)
        # Drive the poller directly.
        printing_buf._check_recovery_done(1.1)
        assert printing_buf._extreme_recovery_active is None
        assert 0 in vactual_writes
        assert vactual_writes[-1] > 0
        reactor.flush_callbacks()
        assert vactual_writes[-1] == 0

    def test_stop_recovery_writes_latch_correction_nudge(
            self, printing_buf, vactual_writes, reactor):
        """The TMC chip latches the sign of the last nonzero VACTUAL
        and would override the DIR pin during subsequent STEP/DIR
        motion.  _stop_recovery_vactual must therefore write a
        positive-magnitude nudge to flip the latch back before
        returning to step/dir mode."""
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        # EMPTY recovery wrote a negative raw register value.
        assert vactual_writes[-1] < 0
        recovery_push_idx = len(vactual_writes) - 1

        # Exit recovery — must produce the three-step sequence:
        # [..., recovery_neg, 0, positive_nudge] then a deferred 0.
        set_sensors(printing_buf, middle=True)
        reactor._monotonic = 2.0
        printing_buf._update_rotation_distance(2.0)
        seq = vactual_writes[recovery_push_idx + 1:]
        assert seq[0] == 0           # immediate stop
        assert seq[1] > 0            # positive latch-correction nudge
        # Deferred final 0 hasn't fired until the reactor drains.
        assert vactual_writes[-1] == seq[1]
        reactor.flush_callbacks()
        assert vactual_writes[-1] == 0

    def test_full_recovery_exit_runs_nudge_sequence(
            self, printing_buf, vactual_writes, reactor):
        """FULL recovery pushes a positive VACTUAL (reverse drain).
        The chip latch is already in the "positive" state that matches
        normal forward STEP/DIR motion under this wiring, so the nudge
        is functionally redundant for FULL exit — but the sequence
        must still run cleanly without leaving VACTUAL stuck."""
        set_sensors(printing_buf, full=True)
        reactor._monotonic = 1.0
        printing_buf._update_rotation_distance(1.0)
        # FULL recovery wrote a positive raw register value (drain).
        assert printing_buf._extreme_recovery_active == ZONE_FULL
        assert vactual_writes[-1] > 0
        recovery_push_idx = len(vactual_writes) - 1

        # Drain succeeds — zone returns to MIDDLE.  (FULL_MIDDLE is
        # still in the FULL band per the poller's exit gate.)
        set_sensors(printing_buf, middle=True)
        printing_buf._check_recovery_done(1.5)
        assert printing_buf._extreme_recovery_active is None
        seq = vactual_writes[recovery_push_idx + 1:]
        assert seq[0] == 0           # immediate stop
        assert seq[1] > 0            # positive nudge (same sign as drain)
        reactor.flush_callbacks()
        assert vactual_writes[-1] == 0


class TestVactualRecoveryExitOnSkipMiddle:
    """A fast EMPTY -> FULL_MIDDLE transition (skipping MIDDLE between
    sensor callbacks) must still exit recovery — the exit gate is
    "left the EMPTY band", not "reached MIDDLE exactly"."""

    def test_empty_recovery_exits_at_full_middle(
            self, printing_buf, vactual_writes, reactor):
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY

        # Zone jumps straight to FULL_MIDDLE without a MIDDLE pass.
        set_sensors(printing_buf, middle=True, full=True)
        reactor._monotonic = 2.0
        printing_buf._update_rotation_distance(2.0)
        assert printing_buf._extreme_recovery_active is None
        assert 0 in vactual_writes
        assert vactual_writes[-1] > 0
        reactor.flush_callbacks()
        assert vactual_writes[-1] == 0


class TestVactualRecoveryTimeout:
    """Per-attempt cap: if recovery cannot pull the buffer out of the
    extreme zone within extreme_recovery_timeout seconds, the poller
    writes VACTUAL=0 and escalates to _handle_error."""

    def test_empty_recovery_timeout_triggers_error(
            self, printing_buf, vactual_writes, reactor):
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY

        # Advance past the timeout — poller faults.
        timeout_t = 1.0 + printing_buf.extreme_recovery_timeout + 0.5
        reactor._monotonic = timeout_t
        printing_buf._check_recovery_done(timeout_t)
        assert printing_buf.state == STATE_ERROR
        assert "recovery exceeded" in printing_buf.error_msg.lower()
        assert 0 in vactual_writes
        assert vactual_writes[-1] > 0
        reactor.flush_callbacks()
        assert vactual_writes[-1] == 0


class TestVactualRecoveryRunout:
    """If filament leaves the presence switch during recovery, the
    chunked fill aborts and VACTUAL=0 is written.  Runs through the
    cleanup invariant in _unsync."""

    def test_runout_aborts_recovery(
            self, printing_buf, vactual_writes, reactor):
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY

        # Material removed mid-recovery — poller stops VACTUAL.
        printing_buf.material_present = False
        printing_buf._check_recovery_done(1.5)
        assert printing_buf._extreme_recovery_active is None
        assert 0 in vactual_writes
        assert vactual_writes[-1] > 0
        reactor.flush_callbacks()
        assert vactual_writes[-1] == 0


class TestVactualLatencyRegressionGuard:
    """Critical regression guard: every VACTUAL register write must
    pass print_time=None to mcu_tmc.set_register.  Scheduling at the
    lookahead tail is what made the sidecar approach fail; we must
    apply VACTUAL immediately."""

    def test_all_vactual_writes_are_immediate(
            self, printing_buf, tmc_writes, reactor):
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        set_sensors(printing_buf, middle=True)
        reactor._monotonic = 2.0
        printing_buf._update_rotation_distance(2.0)
        vactual_writes_full = [w for w in tmc_writes if w[0] == "VACTUAL"]
        assert len(vactual_writes_full) >= 2
        for reg_name, value, print_time in vactual_writes_full:
            assert print_time is None, (
                "VACTUAL must be written immediately, not deferred to "
                "toolhead.get_last_move_time() — that's the bug we "
                "refactored away from")


class TestVactualFormula:
    """Sanity-check the formula and constants for the reference config.

    LLL Plus: 200 motor full steps/rev × 16 microsteps × 50/17 gear_ratio
    / 19.2357 mm rotation_distance ≈ 489.7 microsteps/mm.
    VACTUAL register = mm/s × steps_per_mm / 0.715.  Reference-hardware
    polarity is inverted so the helper negates the result (positive
    input mm/s -> negative register value).
    At 40 mm/s forward: -40 × 489.7 / 0.715 ≈ -27,400.
    At 1 mm/s forward:  -1 × 489.7 / 0.715 ≈ -685.
    """

    def test_40mm_per_s_magnitude_is_about_27400(self, enabled_buf):
        v = enabled_buf._mm_per_s_to_vactual(40.0)
        assert 27000 < abs(v) < 28000
        assert v < 0  # inverted polarity: forward feed -> negative

    def test_1mm_per_s_magnitude_is_about_685(self, enabled_buf):
        v = enabled_buf._mm_per_s_to_vactual(1.0)
        assert 600 < abs(v) < 800
        assert v < 0

    def test_negative_input_writes_positive_vactual(self, enabled_buf):
        # Reverse feed (-1 mm/s) under inverted polarity -> positive
        # register value.  FULL recovery uses this signed convention.
        v = enabled_buf._mm_per_s_to_vactual(-1.0)
        assert v > 0

    def test_zero_speed_is_zero(self, enabled_buf):
        assert enabled_buf._mm_per_s_to_vactual(0.0) == 0

    def test_steps_per_mm_derived_from_config(self, enabled_buf):
        # MockStepper default step_dist matches the reference config.
        assert enabled_buf._steps_per_mm == pytest.approx(489.7, rel=0.01)


class TestVactualCleanupInvariant:
    """Every code path that ends a sync session must clear VACTUAL,
    routed through _unsync.  BUFFER_DISABLE, runout, error, manual
    feed/retract entry all go through _unsync."""

    def test_unsync_clears_vactual_after_recovery(
            self, printing_buf, vactual_writes):
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        # Forward feed -> negative VACTUAL (inverted polarity).
        assert vactual_writes[-1] < 0

        printing_buf._unsync()
        # Cleanup invariant: _unsync writes VACTUAL=0.
        assert 0 in vactual_writes

    def test_disable_clears_vactual(
            self, printing_buf, vactual_writes):
        from conftest import MockGcmd
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        # Forward feed active -> negative VACTUAL (inverted polarity).
        assert vactual_writes[-1] < 0

        printing_buf.cmd_BUFFER_DISABLE(MockGcmd("BUFFER_DISABLE"))
        assert 0 in vactual_writes


class TestRecoveryGatedOnPrinting:
    """Recovery must not enter while the printer is not printing —
    surprise motion during manual loading/unloading was the bug."""

    def test_empty_does_not_enter_recovery_when_not_printing(
            self, enabled_buf, vactual_writes):
        # enabled_buf has _print_stats.state == "standby" by default
        # (the printing_buf fixture is what flips it to "printing").
        assert enabled_buf._print_stats.state != "printing"
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._extreme_recovery_active is None
        # No VACTUAL writes during non-printing zone entry.
        assert vactual_writes == []

    def test_full_does_not_enter_recovery_when_not_printing(
            self, enabled_buf, vactual_writes):
        set_sensors(enabled_buf, full=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._extreme_recovery_active is None
        assert vactual_writes == []


class TestExtremeMultipliersNoLongerApplied:
    """Sanity guard: the multiplier path must not drive rotation_distance
    away from base when in ZONE_FULL or ZONE_EMPTY.  Recovery owns
    those zones now via VACTUAL."""

    def test_extreme_zones_leave_rd_at_base(
            self, printing_buf, stepper, reactor):
        base_rd = printing_buf._base_rd
        # Sync, observe initial rd
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)

        # Enter EMPTY — recovery starts via VACTUAL, no rd change.
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)
        assert printing_buf._rd_multiplier == 1.0


class TestSafetyZoneStartInvariantDuringRecovery:
    """_safety_zone_start must not be re-armed by zone bouncing inside
    the same recovery cycle.  Cumulative cap measures from recovery
    entry, not the latest empty/full re-entry."""

    def test_empty_bounce_does_not_reset_safety_start(
            self, printing_buf, reactor):
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
        start_before = printing_buf._safety_zone_start
        assert start_before > 0.0

        # Bounce out to empty_middle and back to empty.
        set_sensors(printing_buf, empty=False)
        reactor._monotonic = 2.0
        printing_buf._update_rotation_distance(2.0)
        # _safety_zone_start should NOT have been cleared (recovery in
        # flight; cumulative cap keeps counting).
        assert printing_buf._safety_zone_start == start_before


class TestSensorConflictDebounce:
    """Out-of-order callbacks within one MCU report can produce a
    transient empty+full state.  That should defer, not error.  A
    persistent conflict (real wiring fault) escalates via the control
    timer."""

    def test_transient_conflict_then_resolved(self, enabled_buf):
        # Force the transient state, then resolve it before
        # control_interval elapses.
        set_sensors(enabled_buf, empty=True, full=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf.state != STATE_ERROR
        assert enabled_buf._conflict_since == 1.0
        set_sensors(enabled_buf, empty=False, middle=True, full=True)
        enabled_buf._update_rotation_distance(1.05)
        assert enabled_buf._conflict_since == 0.0
        enabled_buf._control_timer_cb(1.5)
        assert enabled_buf.state != STATE_ERROR

    def test_persistent_conflict_escalates(self, enabled_buf):
        set_sensors(enabled_buf, empty=True, full=True)
        enabled_buf._update_rotation_distance(1.0)
        # Conflict survives past control_interval.
        enabled_buf._control_timer_cb(1.0 + enabled_buf.control_interval)
        assert enabled_buf.state == STATE_ERROR


class TestDlogThrottle:
    """Zone-transition and rd_mult logs should collapse rapid same-key
    repeats so klippy.log isn't drowned by sensor chatter."""

    def test_throttle_suppresses_repeats(self, buf, caplog):
        import logging
        buf.debug = True
        caplog.set_level(logging.INFO)
        for i in range(5):
            buf._dlog_throttled(1.0 + i * 0.01, "key", 0.25, "hello %d", i)
        emitted = [r.getMessage() for r in caplog.records
                   if "hello" in r.getMessage()]
        assert len(emitted) == 1
        assert "hello 0" in emitted[0]

    def test_throttle_releases_after_window(self, buf, caplog):
        import logging
        buf.debug = True
        caplog.set_level(logging.INFO)
        buf._dlog_throttled(1.0, "key", 0.25, "first")
        for i in range(3):
            buf._dlog_throttled(1.05 + i * 0.01, "key", 0.25, "drop %d", i)
        buf._dlog_throttled(2.0, "key", 0.25, "after")
        msgs = [r.getMessage() for r in caplog.records]
        assert any("first" in m for m in msgs)
        assert any("after" in m for m in msgs)
        assert any("suppressed 3" in m for m in msgs)

    def test_different_keys_dont_interfere(self, buf, caplog):
        import logging
        buf.debug = True
        caplog.set_level(logging.INFO)
        buf._dlog_throttled(1.0, "key_a", 0.25, "A")
        buf._dlog_throttled(1.001, "key_b", 0.25, "B")
        msgs = [r.getMessage() for r in caplog.records]
        assert any(m.endswith("A") for m in msgs)
        assert any(m.endswith("B") for m in msgs)


def _enable_script(buf):
    return "SET_STEPPER_ENABLE STEPPER=%s ENABLE=0" % buf.stepper_name


class TestVactualRunningLatch:
    """_vactual_maybe_running tracks "the chip may be driving the
    motor".  It must NOT be inferred from _extreme_recovery_active:
    the two disagree exactly in the windows where a runaway is
    possible, so the latch is set before any nonzero write and
    released only by a CONFIRMED zero write."""

    def test_latch_starts_clear(self, enabled_buf):
        assert enabled_buf._vactual_maybe_running is False

    def test_successful_nonzero_write_sets_latch(self, enabled_buf):
        assert enabled_buf._write_vactual(-5000) is True
        assert enabled_buf._vactual_maybe_running is True

    def test_confirmed_zero_write_clears_latch(self, enabled_buf):
        enabled_buf._write_vactual(-5000)
        assert enabled_buf._write_vactual(0) is True
        assert enabled_buf._vactual_maybe_running is False

    def test_failed_nonzero_write_still_sets_latch(
            self, enabled_buf, mcu_tmc):
        """The critical case.  mcu_uart.reg_write is fire-and-forget, so
        a raising set_register means "IFCNT read-back failed", not "the
        chip never got it" — the motor may be running.  Inferring from
        _extreme_recovery_active would miss this entirely, because
        _enter_*_recovery bails out before setting that flag."""
        mcu_tmc.fail_registers.add("VACTUAL")
        assert enabled_buf._write_vactual(-5000) is False
        assert enabled_buf._vactual_maybe_running is True

    def test_failed_zero_write_does_not_clear_latch(
            self, enabled_buf, mcu_tmc):
        enabled_buf._write_vactual(-5000)
        mcu_tmc.fail_registers.add("VACTUAL")
        assert enabled_buf._write_vactual(0) is False
        assert enabled_buf._vactual_maybe_running is True

    def test_no_tmc_never_sets_latch(self, enabled_buf):
        """No TMC object means VACTUAL was never writable, so there is
        nothing that could be spinning."""
        enabled_buf._mcu_tmc = None
        assert enabled_buf._write_vactual(-5000) is False
        assert enabled_buf._vactual_maybe_running is False


class TestUnsyncVactualGate:
    """_unsync skips the VACTUAL cleanup only when the latch proves
    nothing can be running.  The cleanup invariant itself is
    unchanged."""

    def test_unsync_without_recovery_writes_nothing(
            self, enabled_buf, vactual_writes):
        """The optimisation: an _unsync on a buffer that never entered
        recovery costs zero UART transactions."""
        assert vactual_writes == []
        enabled_buf._unsync()
        assert vactual_writes == []

    def test_unsync_after_recovery_still_clears(
            self, printing_buf, vactual_writes):
        """The invariant: cleanup must still run when VACTUAL is live."""
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._vactual_maybe_running is True
        printing_buf._unsync()
        assert 0 in vactual_writes

    def test_failed_recovery_entry_defers_cleanup_to_reenable(
            self, printing_buf, mcu_tmc, vactual_writes):
        """Regression guard for the bug this design avoids: a failed
        recovery-entry write leaves _extreme_recovery_active None, so
        gating cleanup on that flag would lose it entirely.  Under the
        one-shot fallback, the episode's cleanup is owned by the
        driver kill: _unsync skips the bus (the driver is dead or
        dying), the latch survives, and the confirmed zero lands on
        the re-enable path."""
        mcu_tmc.fail_registers.add("VACTUAL")
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active is None
        assert printing_buf._vactual_maybe_running is True

        mcu_tmc.fail_registers.clear()
        before = len(vactual_writes)
        printing_buf._unsync()
        # No bus traffic during the episode — but nothing lost either:
        assert vactual_writes[before:] == []
        assert printing_buf._vactual_maybe_running is True
        from conftest import MockGcmd
        printing_buf.cmd_BUFFER_ENABLE(MockGcmd())
        assert 0 in vactual_writes[before:]

    def test_latch_survives_recovery_exit_nudge_window(
            self, printing_buf, reactor):
        """_stop_recovery_vactual writes a NONZERO nudge and clears
        _extreme_recovery_active immediately, so for ~50 ms the two
        disagree.  The latch must stay set until the deferred zero
        actually lands."""
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        set_sensors(printing_buf, middle=True)
        reactor._monotonic = 2.0
        printing_buf._update_rotation_distance(2.0)
        # Recovery is over, but the +nudge is still live on the chip.
        assert printing_buf._extreme_recovery_active is None
        assert printing_buf._vactual_maybe_running is True
        reactor.flush_callbacks()
        assert printing_buf._vactual_maybe_running is False


class TestEnablePinFallback:
    """When the TMC UART is the thing that failed, retrying it is the
    one action guaranteed not to help.  Drop the enable pin instead —
    a plain GPIO on the same MCU that halts the driver output stage
    regardless of what VACTUAL contains."""

    def test_recovery_entry_failure_disables_driver(
            self, printing_buf, mcu_tmc, gcode, reactor):
        mcu_tmc.fail_registers.add("VACTUAL")
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf.state == STATE_ERROR
        reactor.flush_callbacks()
        assert _enable_script(printing_buf) in gcode.scripts_run

    def test_full_recovery_entry_failure_disables_driver(
            self, printing_buf, mcu_tmc, gcode, reactor):
        mcu_tmc.fail_registers.add("VACTUAL")
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf.state == STATE_ERROR
        reactor.flush_callbacks()
        assert _enable_script(printing_buf) in gcode.scripts_run

    def test_stop_write_failure_disables_driver_and_skips_nudge(
            self, printing_buf, mcu_tmc, gcode, reactor, vactual_writes):
        """Once the stop write fails the chip is known to be driving
        and unreachable.  Issuing the latch nudge would only queue two
        more doomed transactions behind a driver we are about to cut."""
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        mcu_tmc.fail_registers.add("VACTUAL")
        before = len(vactual_writes)
        printing_buf._stop_recovery_vactual()
        # Exactly one attempt (the zero), then the enable-pin escape.
        assert vactual_writes[before:] == [0]
        reactor.flush_callbacks()
        assert _enable_script(printing_buf) in gcode.scripts_run

    def test_latch_nudge_clear_failure_disables_driver(
            self, printing_buf, mcu_tmc, gcode, reactor):
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        set_sensors(printing_buf, middle=True)
        reactor._monotonic = 2.0
        printing_buf._update_rotation_distance(2.0)
        # Nudge is live; break the bus before the deferred clear fires.
        mcu_tmc.fail_registers.add("VACTUAL")
        reactor.flush_callbacks()   # fires the nudge clear -> it fails
        reactor.flush_callbacks()   # fires the deferred enable drop
        assert _enable_script(printing_buf) in gcode.scripts_run
        assert printing_buf._vactual_maybe_running is True

    def test_healthy_recovery_never_touches_enable_pin(
            self, printing_buf, gcode, reactor):
        """The fallback must stay off the happy path."""
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        set_sensors(printing_buf, middle=True)
        reactor._monotonic = 2.0
        printing_buf._update_rotation_distance(2.0)
        reactor.flush_callbacks()
        reactor.flush_callbacks()
        assert _enable_script(printing_buf) not in gcode.scripts_run

    def test_fallback_from_gcode_handler_is_deferred_not_inline(
            self, printing_buf, mcu_tmc, gcode, reactor):
        """Deadlock guard.  BUFFER_DISABLE reaches the fallback through
        _unsync -> _stop_recovery_vactual while holding the gcode mutex,
        and Kalico's ReactorMutex is not reentrant — an inline
        run_script there parks the greenlet on pause(NEVER) forever.
        The script must therefore not be issued until the reactor
        drains, i.e. after the command handler has returned."""
        from conftest import MockGcmd
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        mcu_tmc.fail_registers.add("VACTUAL")

        printing_buf.cmd_BUFFER_DISABLE(MockGcmd())
        assert _enable_script(printing_buf) not in gcode.scripts_run
        reactor.flush_callbacks()
        assert _enable_script(printing_buf) in gcode.scripts_run


class TestShutdownVactualCleanup:
    """_handle_shutdown runs on klippy:disconnect as well as
    klippy:shutdown.  On disconnect the reactor is tearing down, so a
    deferred latch clear may never fire — both the register write and
    the enable-pin drop stay unconditional here."""

    def test_shutdown_writes_zero_and_drops_enable(
            self, printing_buf, gcode, vactual_writes):
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        printing_buf._handle_shutdown()
        assert 0 in vactual_writes
        assert _enable_script(printing_buf) in gcode.scripts_run

    def test_shutdown_drops_enable_even_when_never_recovered(
            self, enabled_buf, gcode):
        """Unconditional by design: shutdown is where belt-and-braces
        is worth its cost, so this must not be gated on the latch."""
        assert enabled_buf._vactual_maybe_running is False
        enabled_buf._handle_shutdown()
        assert _enable_script(enabled_buf) in gcode.scripts_run

    def test_shutdown_drops_enable_when_uart_is_dead(
            self, printing_buf, mcu_tmc, gcode):
        """The Aug-5 shape: MCU already gone, every VACTUAL write times
        out.  The enable-pin drop must still be reached."""
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        mcu_tmc.fail_registers.add("VACTUAL")
        printing_buf._handle_shutdown()
        assert _enable_script(printing_buf) in gcode.scripts_run


class TestBootVactualHygiene:
    """A fresh process cannot trust the chip: the TMC is powered from
    the motor rail, so VACTUAL survives klippy restarts and even
    FIRMWARE_RESTART's MCU reset — and no end-of-session path is
    guaranteed to clear it (MCU shutdown: all UART writes fail;
    disconnect: the MCU's serial closes before our handler runs).
    _handle_ready must therefore clear it before anything can
    re-enable the driver."""

    def _make_ready_buf(self, printer):
        import buffer as buffer_module
        from conftest import MockConfig, DEFAULT_CONFIG
        config = MockConfig(printer, dict(DEFAULT_CONFIG))
        b = buffer_module.Buffer(config)
        for handler in printer.event_handlers.get("klippy:ready", []):
            handler()
        return b

    def test_ready_runs_full_stop_sequence(self, printer):
        b = self._make_ready_buf(printer)
        w = printer.tmc2208.mcu_tmc.vactual_writes
        # Full sequence, not a bare zero: if the prior session died
        # with a nonzero VACTUAL, the chip's direction latch is stale
        # too and must be corrected before the first synced motion.
        assert w[0] == 0
        assert w[1] > 0
        printer.reactor.flush_callbacks()
        assert w[-1] == 0
        assert b._vactual_maybe_running is False

    def test_ready_hygiene_failure_latches_and_disables(self, printer):
        printer.tmc2208.mcu_tmc.fail_registers.add("VACTUAL")
        b = self._make_ready_buf(printer)
        # Unknown chip state + dead bus: stay pessimistic and kill
        # the driver rather than let the first motion re-enable it.
        assert b._vactual_maybe_running is True
        printer.reactor.flush_callbacks()
        assert _enable_script(b) in printer.gcode.scripts_run


class TestFallbackIdempotence:
    """One dead-bus episode must cost one doomed UART transaction and
    one fallback, not a stack of ~5 s retries as the error unwinds
    through _handle_error -> _unsync -> _stop_recovery_vactual."""

    def test_dead_bus_episode_stops_at_one_write(
            self, printing_buf, mcu_tmc, gcode, reactor, vactual_writes):
        mcu_tmc.fail_registers.add("VACTUAL")
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        # The entry attempt is the ONLY transaction; the unwind must
        # not retry the bus it just watched fail.
        expected = printing_buf._mm_per_s_to_vactual(
            printing_buf.recovery_speed)
        assert vactual_writes == [expected]
        reactor.flush_callbacks()
        reactor.flush_callbacks()
        assert gcode.scripts_run.count(_enable_script(printing_buf)) == 1


class TestReEnableConsumesLatch:
    """BUFFER_ENABLE and error-clear re-arm sync and the driver, so
    they must consume _vactual_maybe_running first: a stale nonzero
    VACTUAL resumes motion the moment the driver re-enables."""

    def _dead_bus_error(self, printing_buf, mcu_tmc):
        mcu_tmc.fail_registers.add("VACTUAL")
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf.state == STATE_ERROR
        assert printing_buf._vactual_maybe_running is True

    def test_enable_refused_while_uncleared(
            self, printing_buf, mcu_tmc):
        from conftest import MockGcmd
        self._dead_bus_error(printing_buf, mcu_tmc)
        printing_buf.cmd_BUFFER_ENABLE(MockGcmd())
        assert printing_buf.state == STATE_ERROR
        assert printing_buf._synced_to is None
        assert printing_buf._vactual_maybe_running is True

    def test_enable_clears_then_proceeds(
            self, printing_buf, mcu_tmc, vactual_writes, reactor):
        from conftest import MockGcmd
        self._dead_bus_error(printing_buf, mcu_tmc)
        mcu_tmc.fail_registers.clear()  # bus healed
        before = len(vactual_writes)
        printing_buf.cmd_BUFFER_ENABLE(MockGcmd())
        assert vactual_writes[before] == 0  # confirmed stop first
        # _sync promotes STOPPED -> FEEDING when it binds the trapq.
        assert printing_buf.state == STATE_FEEDING
        assert printing_buf._synced_to is not None
        reactor.flush_callbacks()
        assert printing_buf._vactual_maybe_running is False

    def test_error_clear_refused_while_uncleared(
            self, printing_buf, mcu_tmc):
        from conftest import MockGcmd
        self._dead_bus_error(printing_buf, mcu_tmc)
        printing_buf.cmd_BUFFER_CLEAR_ERROR(MockGcmd())
        assert printing_buf.state == STATE_ERROR
        assert "VACTUAL" in printing_buf.error_msg

    def test_error_clear_succeeds_after_heal(
            self, printing_buf, mcu_tmc, reactor):
        from conftest import MockGcmd
        self._dead_bus_error(printing_buf, mcu_tmc)
        mcu_tmc.fail_registers.clear()
        printing_buf.cmd_BUFFER_CLEAR_ERROR(MockGcmd())
        # auto_enabled was still set, so _clear_error re-syncs and
        # _sync promotes STOPPED -> FEEDING.
        assert printing_buf.state == STATE_FEEDING
        reactor.flush_callbacks()
        assert printing_buf._vactual_maybe_running is False

    def test_enable_during_recovery_is_a_noop(
            self, printing_buf, vactual_writes):
        """Regression guard: BUFFER_ENABLE mid-recovery must not
        consume the latch — the stop sequence would silently kill the
        recovery push while the poller stays armed, guaranteeing a
        spurious per-attempt timeout error."""
        from conftest import MockGcmd
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
        before = len(vactual_writes)
        printing_buf.cmd_BUFFER_ENABLE(MockGcmd())
        assert vactual_writes[before:] == []
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
        assert printing_buf.state == STATE_FEEDING


class TestResyncAndRecoverySameTick:
    """With the control timer's unconditional update call, the
    auto-sync inside _update_rotation_distance restores sync within
    one control_interval of any unsync — and extreme recovery can
    enter on the very tick that re-syncs (the synced gate passes
    because _sync ran earlier in the same call)."""

    def test_resync_and_empty_recovery_same_tick(
            self, printing_buf, vactual_writes):
        printing_buf._unsync()
        assert printing_buf._synced_to is None
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(5.0)
        assert printing_buf._synced_to is not None
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
