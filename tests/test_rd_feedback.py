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
            self, printing_buf, vactual_writes, reactor):
        # Forward-consuming extruder so the FULL gate passes.  The
        # reactor clock must track the eventtime — the mock extruder
        # position derives from reactor.monotonic().
        printing_buf.toolhead.get_extruder().set_rate(5.0, t0=0.0)
        reactor._monotonic = 1.0
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
        # Forward-consuming extruder so the FULL gate passes.
        printing_buf.toolhead.get_extruder().set_rate(5.0, t0=0.0)
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


class TestFullRecoveryGatedOnExtruderConsumption:
    """FULL recovery must only enter while the extruder is actively
    consuming forward.  An idle or retracting extruder DEFERs so the
    buffer stays parked at full extend (the load-to-toolhead flow)
    and full_safety_timeout -> _do_safety_retract remains the hard
    escape.  Regression guard for the drain/re-extend loop that
    defeated the manual_feed_full_timeout stop.

    NB: the mock extruder position derives from reactor.monotonic()
    (see conftest _MockExtrusionStepper), so every test keeps
    reactor._monotonic in step with the eventtime it passes in."""

    def test_full_defers_when_extruder_idle(
            self, printing_buf, vactual_writes, reactor):
        # No set_rate: commanded position is static -> rate 0.
        reactor._monotonic = 1.0
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active is None
        assert vactual_writes == []
        # Still synced — defer is passive, not an unsync.
        assert printing_buf._synced_to is not None
        # The hard escape stays armed.
        assert printing_buf._safety_zone_start == 1.0

    def test_full_enters_when_extruder_consuming(
            self, printing_buf, vactual_writes, reactor):
        printing_buf.toolhead.get_extruder().set_rate(5.0, t0=0.0)
        reactor._monotonic = 1.0
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_FULL
        # Reverse drain -> positive VACTUAL (inverted polarity).
        assert vactual_writes[-1] > 0

    def test_full_defers_when_extruder_retracting(
            self, printing_buf, vactual_writes, reactor):
        # Signed gate: a retracting extruder must defer just like an
        # idle one — draining the buffer while the extruder pulls
        # back would fight it.
        printing_buf.toolhead.get_extruder().set_rate(-5.0, t0=0.0)
        reactor._monotonic = 1.0
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active is None
        assert vactual_writes == []

    def test_deferred_full_still_escalates_to_safety_retract(
            self, printing_buf, reactor, force_move):
        # Defer-then-escalate is the complete fallback chain: the
        # deferred FULL keeps _safety_zone_start ticking, and
        # full_safety_timeout fires _do_safety_retract.
        reactor._monotonic = 1.0
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active is None
        assert printing_buf._safety_zone_start == 1.0

        reactor._monotonic = 1.0 + printing_buf.full_safety_timeout + 1.0
        printing_buf._control_timer_cb(reactor._monotonic)
        assert printing_buf._extreme_recovery_active is None
        # Safety retract issued a negative force_move distance.
        assert force_move.moves[-1][1] < 0

    def test_gate_uses_fresh_rate_after_stale_extruder_move(
            self, printing_buf, vactual_writes, reactor):
        # A manual extruder move minutes before FULL entry must not
        # leak a stale positive rate into the gate decision.  The
        # control timer refreshes the sample every tick so the entry
        # decision sees a <= control_interval window.  Without that
        # refresh, the rate here would average (50 mm - 0 mm) since
        # boot and enter recovery on a bogus 31 mm/s.
        stepper = (printing_buf.toolhead.get_extruder()
                   .extruder_stepper.stepper)
        stepper._position = 50.0  # extruder moved while zone was calm
        reactor._monotonic = 1.0
        printing_buf._control_timer_cb(1.0)   # samples the jump
        reactor._monotonic = 1.5
        printing_buf._control_timer_cb(1.5)   # fresh window: rate 0
        reactor._monotonic = 1.6
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.6)
        assert printing_buf._extreme_recovery_active is None
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
