"""Tests for full zone safety timeout and retraction logic."""

import pytest
from conftest import (
    ZONE_FULL,
    ZONE_FULL_MIDDLE,
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
                                              force_move):
        set_sensors(enabled_buf, full=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)

        t += enabled_buf.full_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert len(force_move.moves) > 0
        assert force_move.moves[-1][1] < 0  # retract = negative dist


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
