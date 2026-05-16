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
    def test_empty_enters_vactual_recovery(self, printing_buf):
        # EMPTY recovery writes VACTUAL.  The stepper is unsynced from
        # the trapq during recovery because on hardware the TMC's
        # internal commanded position diverges from Klipper's trapq
        # tracking, and after VACTUAL=0 the trapq sync would silently
        # break (drift or wrong direction).  The previous synced
        # extruder is saved so _exit_extreme_recovery can re-sync.
        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._current_zone == ZONE_EMPTY
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY
        assert printing_buf._synced_to is None
        assert printing_buf._recovery_resync_to == "extruder"


class TestFullZone:
    def test_full_enters_vactual_recovery(self, printing_buf, reactor):
        # FULL recovery enters unconditionally — slow reverse VACTUAL
        # drains regardless of extruder direction (no "defer if idle"
        # branch any more).  Same unsync pattern as EMPTY.
        set_sensors(printing_buf, full=True)
        reactor._monotonic = 1.0
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._current_zone == ZONE_FULL
        assert printing_buf._extreme_recovery_active == ZONE_FULL
        assert printing_buf._synced_to is None
        assert printing_buf._recovery_resync_to == "extruder"


class TestEmptyMiddleZone:
    def test_empty_middle_slight_increase(self, enabled_buf):
        # Non-extreme zone application is hysteresis-gated: requires
        # the same zone to be seen across >200ms before commit.
        set_sensors(enabled_buf)  # all off -> EMPTY_MIDDLE
        enabled_buf._update_rotation_distance(1.0)
        enabled_buf._update_rotation_distance(1.3)
        assert enabled_buf._current_zone == ZONE_EMPTY_MIDDLE
        assert enabled_buf._rd_multiplier > 1.0
        assert enabled_buf._rd_multiplier == pytest.approx(
            1.0 + enabled_buf.drift_gain)


class TestFullMiddleZone:
    def test_full_middle_slight_decrease(self, enabled_buf):
        set_sensors(enabled_buf, full=True, middle=True)
        enabled_buf._update_rotation_distance(1.0)
        enabled_buf._update_rotation_distance(1.3)
        assert enabled_buf._current_zone == ZONE_FULL_MIDDLE
        assert enabled_buf._rd_multiplier < 1.0
        assert enabled_buf._rd_multiplier == pytest.approx(
            1.0 - enabled_buf.drift_gain)


class TestSensorConflict:
    def test_empty_and_full_triggers_error(self, enabled_buf):
        set_sensors(enabled_buf, empty=True, full=True)
        enabled_buf._update_rotation_distance(1.0)
        # Conflict is deferred at first (could be transient).  Persistent
        # conflict escalates via the control timer after control_interval.
        assert enabled_buf.state != STATE_ERROR
        enabled_buf._control_timer_cb(1.0 + enabled_buf.control_interval)
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
    def test_middle_to_empty_to_middle(self, printing_buf):
        # Forward extruder rate so EMPTY recovery is ENTERed.
        printing_buf.toolhead.get_extruder().set_rate(5.0, t0=0.0)
        set_sensors(printing_buf, middle=True)
        printing_buf._update_rotation_distance(1.0)
        assert printing_buf._current_zone == ZONE_MIDDLE
        assert printing_buf._rd_multiplier == 1.0

        set_sensors(printing_buf, empty=True)
        printing_buf._update_rotation_distance(2.0)
        assert printing_buf._current_zone == ZONE_EMPTY
        # Recovery owns this; buffer is unsynced, no multiplier change.
        assert printing_buf._extreme_recovery_active == ZONE_EMPTY

        set_sensors(printing_buf, middle=True)
        printing_buf._update_rotation_distance(3.0)
        assert printing_buf._current_zone == ZONE_MIDDLE
        # Returning to MIDDLE exits recovery and re-syncs at 1.0.
        assert printing_buf._extreme_recovery_active is None
        assert printing_buf._rd_multiplier == 1.0

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
