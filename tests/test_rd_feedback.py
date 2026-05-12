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

    def test_full_middle_subtracts_drift_gain(self, enabled_buf):
        m = enabled_buf._zone_to_multiplier(ZONE_FULL_MIDDLE)
        assert m == pytest.approx(1.0 - enabled_buf.drift_gain)

    def test_extreme_zones_return_one(self, enabled_buf):
        """ZONE_EMPTY and ZONE_FULL no longer chase with rotation_distance —
        they go through the unsync-and-recover path instead.  The mapping
        function returns 1.0 for them so the apply path is a no-op even
        if the recovery branch is bypassed."""
        assert enabled_buf._zone_to_multiplier(ZONE_EMPTY) == 1.0
        assert enabled_buf._zone_to_multiplier(ZONE_FULL) == 1.0


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
    """Verify the multiplier is applied via rd_new = base_rd / multiplier
    for the non-extreme drift_gain zones."""

    def test_empty_middle_applies_drift_gain(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf)  # all off -> EMPTY_MIDDLE
        enabled_buf._update_rotation_distance(1.0)
        expected_mult = 1.0 + enabled_buf.drift_gain
        expected_rd = base_rd / expected_mult
        assert enabled_buf._rd_multiplier == pytest.approx(expected_mult)
        assert stepper.get_rotation_distance()[0] == pytest.approx(expected_rd)

    def test_full_middle_applies_drift_gain(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf, full=True, middle=True)
        enabled_buf._update_rotation_distance(1.0)
        expected_mult = 1.0 - enabled_buf.drift_gain
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


class TestApplyMultiplierDedup:
    """_apply_multiplier should skip the stepper write when the requested
    multiplier matches the current value (steady-state in a single zone =
    zero rotation_distance writes)."""

    def test_repeat_same_multiplier_skips_set(self, enabled_buf, stepper):
        baseline = len(stepper.rd_log)
        enabled_buf._apply_multiplier(1.0)
        enabled_buf._apply_multiplier(1.0)
        enabled_buf._apply_multiplier(1.0)
        assert len(stepper.rd_log) == baseline


class TestUnsyncRestoresBaseRotationDistance:
    """After _unsync, the stepper's rotation_distance must be restored
    to _base_rd so any subsequent force_move.manual_move computes the
    right number of steps.  Without this, BUFFER_FEED/BUFFER_RETRACT
    issued after leaving a non-MIDDLE zone would move the wrong amount
    of filament (last zone's multiplier would apply)."""

    def test_unsync_from_drift_zone_restores_base(self, enabled_buf, stepper):
        """Drift-zone multiplier was applied; explicit _unsync restores."""
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf)  # all off -> EMPTY_MIDDLE
        enabled_buf._update_rotation_distance(1.0)
        assert stepper.get_rotation_distance()[0] == pytest.approx(
            base_rd / (1.0 + enabled_buf.drift_gain))

        enabled_buf._unsync()
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)
        assert enabled_buf._rd_multiplier == 1.0

    def test_recovery_entry_unsyncs_and_restores_base(self, printing_buf,
                                                      stepper):
        """Entering EMPTY recovery should leave the stepper at base_rd
        (the recovery's internal _unsync restores it)."""
        base_rd = printing_buf._base_rd
        printing_buf.toolhead.get_extruder().set_rate(5.0, t0=0.0)
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
        assert printing_buf._synced_to is None
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)

    def test_recovery_exit_resyncs_at_one(
            self, printing_buf, stepper, reactor):
        """When zone returns to MIDDLE, recovery exits and resyncs at 1.0."""
        base_rd = printing_buf._base_rd
        printing_buf.toolhead.get_extruder().set_rate(5.0, t0=0.0)
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY

        # Zone progresses to MIDDLE — recovery exits, resyncs at 1.0.
        set_sensors(printing_buf, middle=True)
        reactor._monotonic = 2.0
        printing_buf._update_rotation_distance(2.0)
        assert printing_buf._extreme_recovery_active is None
        assert printing_buf._synced_to is not None
        assert printing_buf._rd_multiplier == 1.0
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)


def _seed_rate(buf, reactor, mm_per_s, t_start=0.0):
    """Set extruder rate and seed the rate sample so the next call to
    _estimated_extruder_rate sees a non-zero rate.  Returns the eventtime
    immediately after the seed (callers advance from here)."""
    buf.toolhead.get_extruder().set_rate(mm_per_s, t0=t_start)
    reactor._monotonic = t_start
    buf._estimated_extruder_rate(t_start)
    return t_start


class TestRecoveryDecisionFull:
    """FULL recovery requires forward extruder consumption — without it
    the buffer cannot drain passively, so we DEFER and let
    full_safety_timeout fire _do_safety_retract instead."""

    def test_full_forward_extruder_enters(self, printing_buf, reactor):
        _seed_rate(printing_buf, reactor, 5.0)
        reactor._monotonic = 1.0
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_FULL
        assert printing_buf._synced_to is None

    def test_full_idle_extruder_defers(self, printing_buf, reactor):
        # Rate stays 0 — no set_rate call, no extruder activity.
        reactor._monotonic = 1.0
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active is None
        # Buffer remains synced; safety_zone_start is armed for the
        # cumulative full_safety_timeout escape.
        assert printing_buf._synced_to is not None
        assert printing_buf._safety_zone_start == 1.0

    def test_full_retracting_extruder_defers(self, printing_buf, reactor):
        _seed_rate(printing_buf, reactor, -5.0)  # negative rate = retract
        reactor._monotonic = 1.0
        set_sensors(printing_buf, full=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active is None
        assert printing_buf._synced_to is not None

    def test_full_safety_retract_fallback_after_defer(
            self, enabled_buf, reactor, force_move):
        """When FULL recovery DEFERs (idle extruder) and the buffer
        stays in FULL past full_safety_timeout, the existing
        _do_safety_retract path must still fire."""
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, full=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        # DEFERed (idle extruder) — no recovery, no retract yet.
        assert enabled_buf._extreme_recovery_active is None
        assert len(force_move.moves) == 0

        # Advance past full_safety_timeout — _control_timer_cb should
        # call _do_safety_retract which issues a negative manual_move.
        t += enabled_buf.full_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert len(force_move.moves) > 0
        assert force_move.moves[-1][1] < 0


class TestRecoveryDecisionEmpty:
    """EMPTY recovery is symmetric across extruder direction; only the
    fill_speed varies based on whether the extruder rate exceeds the
    configured manual_speed."""

    def test_empty_idle_uses_manual_speed(self, printing_buf, reactor,
                                          force_move):
        # Idle extruder; first chunk should fire at manual_speed.
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
        # The first chunk fires inline from _enter_extreme_recovery.
        assert len(force_move.moves) >= 1
        last = force_move.moves[-1]
        assert last[1] > 0  # forward feed
        assert last[2] == pytest.approx(printing_buf.manual_speed)

    def test_empty_slow_extruder_uses_manual_speed(
            self, printing_buf, reactor, force_move):
        # Slow extruder (well below manual_speed / 1.2) — recovery
        # should still pick manual_speed as the chunk rate.
        slow_rate = printing_buf.manual_speed / 4.0
        _seed_rate(printing_buf, reactor, slow_rate)
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
        last = force_move.moves[-1]
        assert last[2] == pytest.approx(printing_buf.manual_speed)

    def test_empty_fast_extruder_bumps_speed(
            self, printing_buf, reactor, force_move):
        # Extruder faster than manual_speed / 1.2; chunk should fire at
        # rate * RECOVERY_OVERHEAD (1.2x) so we keep ahead of the drain.
        fast_rate = printing_buf.manual_speed * 2.0  # over the threshold
        _seed_rate(printing_buf, reactor, fast_rate)
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
        last = force_move.moves[-1]
        # 1.2 = RECOVERY_OVERHEAD constant in buffer.py.
        assert last[2] == pytest.approx(fast_rate * 1.2)

    def test_empty_fill_speed_capped(self, printing_buf, reactor,
                                     force_move):
        # Pathological extruder rate (e.g. measurement glitch). The
        # cap is manual_speed * 4 (RECOVERY_SPEED_CAP_FACTOR).
        crazy_rate = printing_buf.manual_speed * 100.0
        _seed_rate(printing_buf, reactor, crazy_rate)
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        last = force_move.moves[-1]
        assert last[2] == pytest.approx(printing_buf.manual_speed * 4.0)

    def test_empty_recovery_exits_when_zone_reaches_middle(
            self, printing_buf, reactor, force_move):
        # After fill chunks land filament at middle, recovery exits
        # and re-syncs at multiplier 1.0.
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY

        # Simulate the next chunk callback finding zone == MIDDLE.
        set_sensors(printing_buf, middle=True)
        reactor._monotonic = 1.5
        printing_buf._do_recovery_fill_chunk(1.5)
        assert printing_buf._extreme_recovery_active is None
        assert printing_buf._synced_to is not None
        assert printing_buf._rd_multiplier == 1.0


class TestRecoveryFillSpeedReevaluated:
    """During EMPTY recovery, each chunk re-samples the extruder rate
    and adjusts the next chunk's fill speed.  A mid-recovery print-speed
    change should bump the fill rate accordingly."""

    def test_chunk_recomputes_fill_speed(self, printing_buf, reactor,
                                          force_move):
        # Start at slow extruder rate -> recovery enters at manual_speed.
        slow = printing_buf.manual_speed / 4.0
        _seed_rate(printing_buf, reactor, slow)
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert force_move.moves[-1][2] == pytest.approx(
            printing_buf.manual_speed)

        # Print speed jumps mid-recovery; re-seed sample so next call
        # observes the new rate.
        fast = printing_buf.manual_speed * 3.0
        printing_buf.toolhead.get_extruder().set_rate(fast, t0=1.0)
        # Force a fresh sample reference at t=1.0
        printing_buf._last_extruder_position_sample = (1.0, 0.0)
        reactor._monotonic = 2.0
        # Next chunk should observe the bumped rate and feed faster.
        printing_buf._do_recovery_fill_chunk(2.0)
        assert force_move.moves[-1][2] == pytest.approx(fast * 1.2)


class TestRecoveryTimeout:
    """Per-attempt EMPTY recovery cap: extreme_recovery_timeout."""

    def test_empty_recovery_timeout_triggers_error(
            self, printing_buf, reactor):
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY

        # Advance past the per-attempt timeout and fire a chunk callback
        # while still in EMPTY — timeout branch should call _handle_error.
        reactor._monotonic = (
            1.0 + printing_buf.extreme_recovery_timeout + 0.5)
        printing_buf._do_recovery_fill_chunk(reactor._monotonic)
        assert printing_buf.state == STATE_ERROR
        assert "EMPTY recovery" in printing_buf.error_msg


class TestMaterialRunoutAbortsRecovery:
    """If filament leaves the presence switch mid-recovery, the chunked
    fill aborts immediately — pushing more filament against an empty
    feed path serves no purpose."""

    def test_runout_aborts_empty_recovery(self, printing_buf, reactor):
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY

        # Material removed mid-recovery.
        printing_buf.material_present = False
        reactor._monotonic = 1.5
        printing_buf._do_recovery_fill_chunk(1.5)
        assert printing_buf._extreme_recovery_active is None


class TestExtremeMultipliersNoLongerApplied:
    """Sanity guard against regression: the multiplier path must not
    drive rotation_distance away from base when in ZONE_FULL or
    ZONE_EMPTY.  Recovery owns those zones now."""

    def test_extreme_zones_leave_rd_at_base(
            self, printing_buf, stepper, reactor):
        base_rd = printing_buf._base_rd
        # Sync, observe initial rd
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)

        # Enter EMPTY (recovery enters and unsyncs, restoring base_rd)
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert stepper.get_rotation_distance()[0] == pytest.approx(base_rd)
        assert printing_buf._rd_multiplier == 1.0


class TestRecoveryGatedOnPrinting:
    """Recovery must not enter while the printer is not printing —
    surprise motion during manual loading/unloading was the bug."""

    def test_empty_does_not_enter_recovery_when_not_printing(
            self, enabled_buf, reactor, force_move):
        # enabled_buf has _print_stats.state == "standby" by default
        # (the printing_buf fixture is what flips it to "printing").
        assert enabled_buf._print_stats.state != "printing"
        reactor._monotonic = 1.0
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._extreme_recovery_active is None
        # No chunked feed should have fired.
        assert force_move.moves == []

    def test_full_does_not_enter_recovery_when_not_printing(
            self, enabled_buf, reactor):
        # Forward extruder rate would normally cause FULL recovery to
        # ENTER and unsync; the printing gate must block it.
        _seed_rate(enabled_buf, reactor, 5.0)
        reactor._monotonic = 1.0
        set_sensors(enabled_buf, full=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._extreme_recovery_active is None
        assert enabled_buf._synced_to is not None


class TestRateSamplePrimedAtReady:
    """The first call to _estimated_extruder_rate after startup must
    return a real slope, not 0 — _handle_ready primes the sample."""

    def test_sample_is_primed_after_handle_ready(self, enabled_buf):
        # The buf fixture fires klippy:ready, which should have called
        # _handle_ready and primed _last_extruder_position_sample.
        assert enabled_buf._last_extruder_position_sample is not None

    def test_first_recovery_decision_sees_nonzero_rate(
            self, printing_buf, reactor, force_move):
        # Configure the extruder so a fresh rate read returns > the
        # bump threshold (manual_speed / 1.2).  With the primed sample
        # from _handle_ready, the FIRST recovery decision must compute
        # a real slope and bump the fill_speed accordingly.
        fast = printing_buf.manual_speed * 2.0
        # Reset sample baseline to a known (t=0, pos=0) so the rate
        # computed at t=1.0 reflects the configured rate.
        printing_buf._last_extruder_position_sample = (0.0, 0.0)
        printing_buf.toolhead.get_extruder().set_rate(fast, t0=0.0)
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
        last = force_move.moves[-1]
        # Did NOT collapse to manual_speed — the rate was observed.
        assert last[2] == pytest.approx(fast * 1.2)


class TestToolChangeResetsRateCache:
    """The active-extruder identity is the only thing that invalidates
    the cached (eventtime, position) sample.  _handle_extruder_change
    must clear the cache and re-prime against the new extruder so the
    next _estimated_extruder_rate call doesn't compute a discontinuous
    delta against the old extruder's position."""

    def test_handle_extruder_change_reprimes_sample(
            self, printing_buf, reactor):
        from conftest import MockExtruder
        _seed_rate(printing_buf, reactor, 5.0)
        sample_before = printing_buf._last_extruder_position_sample
        assert sample_before is not None

        # Swap the active extruder identity to a fresh one.  The new
        # extruder's position starts at 0; the old one's was advancing.
        new_ext = MockExtruder(name="extruder1")
        printing_buf.toolhead.set_extruder(new_ext)
        reactor._monotonic = 2.0
        printing_buf._handle_extruder_change("extruder1")

        # Sample must have been re-primed (non-None, but distinct from
        # the pre-swap sample tuple).
        assert printing_buf._last_extruder_position_sample is not None
        assert (printing_buf._last_extruder_position_sample
                != sample_before)
        assert printing_buf._last_computed_extruder_rate == 0.0


class TestRecoveryExitOnSkipMiddle:
    """A fast EMPTY -> FULL_MIDDLE transition (skipping MIDDLE between
    sensor callbacks) must still exit recovery — the sensor-driven
    exit gate is "left the EMPTY band", not "reached MIDDLE exactly"."""

    def test_empty_recovery_exits_at_full_middle(
            self, printing_buf, reactor):
        reactor._monotonic = 1.0
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY

        # Zone jumps straight to FULL_MIDDLE without a MIDDLE pass.
        set_sensors(printing_buf, middle=True, full=True)
        reactor._monotonic = 2.0
        printing_buf._update_rotation_distance(2.0)
        assert printing_buf._extreme_recovery_active is None
        assert printing_buf._synced_to is not None


