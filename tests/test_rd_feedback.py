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

    def test_empty_adds_safety_gain(self, enabled_buf):
        m = enabled_buf._zone_to_multiplier(ZONE_EMPTY)
        assert m == pytest.approx(1.0 + enabled_buf.safety_gain)

    def test_full_middle_subtracts_drift_gain(self, enabled_buf):
        m = enabled_buf._zone_to_multiplier(ZONE_FULL_MIDDLE)
        assert m == pytest.approx(1.0 - enabled_buf.drift_gain)

    def test_full_subtracts_safety_gain(self, enabled_buf):
        m = enabled_buf._zone_to_multiplier(ZONE_FULL)
        assert m == pytest.approx(1.0 - enabled_buf.safety_gain)


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
        expected_mult = 1.0 + enabled_buf.safety_gain
        expected_rd = base_rd / expected_mult
        assert enabled_buf._rd_multiplier == pytest.approx(expected_mult)
        assert stepper.get_rotation_distance()[0] == pytest.approx(expected_rd)

    def test_full_decreases_rd_multiplier(self, enabled_buf, stepper):
        base_rd = enabled_buf._base_rd
        set_sensors(enabled_buf, full=True)
        enabled_buf._update_rotation_distance(1.0)
        expected_mult = 1.0 - enabled_buf.safety_gain
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

        # Multiplier should be escalated (safety_gain * 1.5)
        escalated_mult = 1.0 + enabled_buf.safety_gain * 1.5
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
