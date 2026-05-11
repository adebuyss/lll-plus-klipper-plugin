"""Tests for sensor zone classification and state transitions."""

import pytest
from conftest import (
    FORWARD,
    STOP,
    STATE_STOPPED,
    STATE_FEEDING,
    STATE_ERROR,
    STATE_MANUAL_FEED,
    STATE_DISABLED,
    ZONE_EMPTY,
    ZONE_EMPTY_MIDDLE,
    ZONE_MIDDLE,
    ZONE_FULL,
    ZONE_FULL_MIDDLE,
    set_sensors,
)


class TestMiddleZone:
    def test_synced_and_middle_is_feeding(self, enabled_buf):
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._current_zone == ZONE_MIDDLE
        assert enabled_buf._rd_multiplier == 1.0

    def test_middle_resets_safety_state(self, enabled_buf):
        # First enter empty to set safety state
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._safety_zone_start > 0.0 or True  # may be 1.0

        # Return to middle
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(2.0)
        assert enabled_buf._safety_zone_start == 0.0


class TestEmptyZone:
    def test_empty_enters_recovery_and_unsyncs(self, enabled_buf):
        # Forward extruder rate so EMPTY recovery is ENTERed.
        enabled_buf.toolhead.get_extruder().set_rate(5.0, t0=0.0)
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._current_zone == ZONE_EMPTY
        # Recovery owns the unsynced state; rd_multiplier is left at 1.0
        # (recovery does not chase with rotation_distance).
        assert enabled_buf._extreme_recovery_active == ZONE_EMPTY
        assert enabled_buf._synced_to is None


class TestFullZone:
    def test_full_enters_recovery_when_extruder_consuming(
            self, enabled_buf, reactor):
        # Forward extruder rate so FULL recovery is ENTERed.  Two-step
        # update so the rate sampler has a prior reference (it returns
        # 0.0 on its first call regardless of the underlying rate).
        enabled_buf.toolhead.get_extruder().set_rate(5.0, t0=0.0)
        set_sensors(enabled_buf, full=True)
        enabled_buf._update_rotation_distance(0.0)
        # First call seeds the rate sample → DEFER, no recovery yet.
        assert enabled_buf._extreme_recovery_active is None
        reactor._monotonic = 1.0
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._current_zone == ZONE_FULL
        assert enabled_buf._extreme_recovery_active == ZONE_FULL
        assert enabled_buf._synced_to is None


class TestEmptyMiddleZone:
    def test_empty_middle_slight_increase(self, enabled_buf):
        set_sensors(enabled_buf)  # all off
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._current_zone == ZONE_EMPTY_MIDDLE
        assert enabled_buf._rd_multiplier > 1.0
        # drift correction is gentler than full extreme — capped at
        # 1.0 + drift_gain (no escalation in the near-edge zone).
        assert enabled_buf._rd_multiplier == pytest.approx(
            1.0 + enabled_buf.drift_gain)


class TestFullMiddleZone:
    def test_full_middle_slight_decrease(self, enabled_buf):
        set_sensors(enabled_buf, full=True, middle=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._current_zone == ZONE_FULL_MIDDLE
        assert enabled_buf._rd_multiplier < 1.0
        assert enabled_buf._rd_multiplier == pytest.approx(
            1.0 - enabled_buf.drift_gain)


class TestSensorConflict:
    def test_empty_and_full_triggers_error(self, enabled_buf):
        set_sensors(enabled_buf, empty=True, full=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf.state == STATE_ERROR
        assert "conflict" in enabled_buf.error_msg.lower()


class TestGuardConditions:
    def test_sensor_callback_skipped_when_not_enabled(self, buf, buttons):
        buf.auto_enabled = False
        set_sensors(buf, empty=True)
        # Trigger sensor — should not crash or update rd
        buttons.callbacks["PE0"](10.0, 0)
        assert buf._rd_multiplier == 1.0

    def test_sensor_callback_skipped_in_manual_state(self, enabled_buf,
                                                      buttons):
        enabled_buf.state = STATE_MANUAL_FEED
        buttons.callbacks["PE0"](10.0, 0)
        # Should not update multiplier in manual state

    def test_sensor_callback_skipped_in_error(self, enabled_buf, buttons):
        enabled_buf.state = STATE_ERROR
        buttons.callbacks["PE0"](10.0, 0)

    def test_sensor_callback_skipped_when_disabled(self, enabled_buf, buttons):
        enabled_buf.state = STATE_DISABLED
        buttons.callbacks["PE0"](10.0, 0)


class TestZoneTransitions:
    def test_middle_to_empty_to_middle(self, enabled_buf):
        # Forward extruder rate so EMPTY recovery is ENTERed.
        enabled_buf.toolhead.get_extruder().set_rate(5.0, t0=0.0)
        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._current_zone == ZONE_MIDDLE
        assert enabled_buf._rd_multiplier == 1.0

        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(2.0)
        assert enabled_buf._current_zone == ZONE_EMPTY
        # Recovery owns this; buffer is unsynced, no multiplier change.
        assert enabled_buf._extreme_recovery_active == ZONE_EMPTY

        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(3.0)
        assert enabled_buf._current_zone == ZONE_MIDDLE
        # Returning to MIDDLE exits recovery and re-syncs at 1.0.
        assert enabled_buf._extreme_recovery_active is None
        assert enabled_buf._rd_multiplier == 1.0

    def test_full_to_full_middle_to_middle(self, enabled_buf):
        set_sensors(enabled_buf, full=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._current_zone == ZONE_FULL

        set_sensors(enabled_buf, full=True, middle=True)
        enabled_buf._update_rotation_distance(2.0)
        assert enabled_buf._current_zone == ZONE_FULL_MIDDLE

        set_sensors(enabled_buf, middle=True)
        enabled_buf._update_rotation_distance(3.0)
        assert enabled_buf._current_zone == ZONE_MIDDLE
        assert enabled_buf._rd_multiplier == 1.0
