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

        # Return to middle
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(10.0)
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
