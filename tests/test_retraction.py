"""Tests for retraction following when not printing."""

import pytest
from conftest import (
    FORWARD,
    BACK,
    STOP,
    STATE_STOPPED,
    STATE_RETRACTING,
    set_sensors,
    simulate_e_move,
)


class TestRetractionFollowing:
    def test_non_printing_retraction_follows_back(self, enabled_buf, reactor):
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=-5.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is True
        assert enabled_buf.motor_direction == BACK

    def test_retraction_below_min_velocity_stops(self, enabled_buf, reactor):
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # Set velocity below min
        enabled_buf._extruder_retracting = True
        enabled_buf.extruder_velocity = 0.01  # below min_velocity (0.05)
        enabled_buf._evaluate_and_drive(10.0)
        assert enabled_buf.motor_direction == STOP

    def test_printing_retraction_zeroes_velocity(self, enabled_buf, reactor):
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        enabled_buf.motor_direction = FORWARD
        enabled_buf.state = STATE_STOPPED
        # First extrude to populate the velocity window
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        prev_velocity = enabled_buf.extruder_velocity
        assert prev_velocity > 0
        # Now retract — velocity should be zeroed
        reactor._monotonic = 10.05
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is True
        # Velocity zeroed during retraction to prevent stale forward drive
        assert enabled_buf.extruder_velocity == 0.0

    def test_follow_retract_disabled(self, enabled_buf, reactor):
        enabled_buf.follow_retract = False
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=-5.0, xyz_dist=0.0, speed=30.0)
        # With follow_retract=False and not printing, retraction should still
        # set _extruder_retracting but velocity should be 0
        assert enabled_buf._extruder_retracting is True
        assert enabled_buf.extruder_velocity == 0.0

    def test_forward_extrusion_clears_retraction(self, enabled_buf, reactor):
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=-5.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is True

        # Forward extrusion (with XY movement = real extrusion)
        simulate_e_move(enabled_buf, e_delta=5.0, xyz_dist=10.0, speed=30.0)
        assert enabled_buf._extruder_retracting is False
        assert enabled_buf.extruder_velocity > 0


class TestRetractionZhop:
    """Tests for retraction + z-hop behaviour during printing."""

    def test_travel_retraction_stops_motor(self, enabled_buf, reactor):
        """Retract during print, wait 0.25s (timer fires) → motor stops."""
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # Extrude to set up forward motion
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.motor_direction == FORWARD
        # Retract (travel retraction)
        reactor._monotonic = 10.05
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        # Advance time so control timer fires (0.25s interval)
        reactor.advance_time(0.25)
        assert enabled_buf.motor_direction == STOP

    def test_infill_retraction_preserves_motor(self, enabled_buf, reactor):
        """Extrude → retract → extrude within 0.05s → motor stays FORWARD."""
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # Extrude
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.motor_direction == FORWARD
        # Brief retract
        reactor._monotonic = 10.02
        simulate_e_move(enabled_buf, e_delta=-0.8, xyz_dist=0.0, speed=30.0)
        # Quick resume extrusion (infill gap < 0.05s)
        reactor._monotonic = 10.05
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        # Fire deferred callbacks
        reactor.flush_callbacks()
        assert enabled_buf.motor_direction == FORWARD

    def test_smoothed_velocity_returns_zero_during_retraction(self, enabled_buf, reactor):
        """_smoothed_velocity returns 0 when _extruder_retracting is True."""
        reactor._monotonic = 10.0
        enabled_buf._velocity_window = [(10.0, 5.0)]
        enabled_buf._extruder_retracting = True
        assert enabled_buf._smoothed_velocity(10.0) == 0.0

    def test_smoothed_velocity_resumes_after_deretract(self, enabled_buf, reactor):
        """After retract + forward extrusion, _smoothed_velocity returns
        positive value from velocity window."""
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # Extrude to populate window
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        # Retract
        reactor._monotonic = 10.05
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._smoothed_velocity(10.05) == 0.0
        # De-retract / resume extrusion
        reactor._monotonic = 10.10
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf._extruder_retracting is False
        assert enabled_buf._smoothed_velocity(10.10) > 0.0

    def test_retract_zhop_travel_deretract_sequence(self, enabled_buf, reactor):
        """Full retract → z-hop → travel → de-retract sequence: motor
        stops during travel gap, resumes after de-retract."""
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # T=0: Extrude (printing)
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.motor_direction == FORWARD
        # T=0.02: Retract (G1 E-2)
        reactor._monotonic = 10.02
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is True
        # T=0.05: Z-hop (no E movement, just Z) — no _on_e_movement call
        # T=0.10: XY travel (no E movement) — no _on_e_movement call
        # Advance time so timer fires during travel gap
        reactor.advance_time(0.25)
        assert enabled_buf.motor_direction == STOP
        # De-retract after sufficient time for rate limit to expire
        # (need > _min_drive_interval since last drive)
        reactor._monotonic = 10.50
        simulate_e_move(enabled_buf, e_delta=2.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is False
        # Fire deferred callbacks and timers
        reactor.flush_callbacks()
        reactor._fire_timers()
        # Resume extrusion after rate limit interval
        reactor._monotonic = 10.65
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        # Fire all pending callbacks and timers
        reactor.flush_callbacks()
        reactor._fire_timers()
        assert enabled_buf.motor_direction == FORWARD

    def test_velocity_window_preserved_during_retraction(self, enabled_buf, reactor):
        """Velocity window is NOT cleared during print retraction,
        preserving speed data for matching when extrusion resumes."""
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # Extrude to populate velocity window
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        assert len(enabled_buf._velocity_window) > 0
        # Retract
        reactor._monotonic = 10.05
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        # Window should still have entries
        assert len(enabled_buf._velocity_window) > 0


class TestDeretractFiltering:
    """Tests that de-retract (pure E forward) doesn't pollute velocity."""

    def test_deretract_does_not_pollute_velocity_window(self, enabled_buf, reactor):
        """De-retract (pure E+) should NOT add entry to window."""
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # Extrude to populate window
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        window_len = len(enabled_buf._velocity_window)
        # Retract
        reactor._monotonic = 10.05
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        # De-retract (pure E at retract speed — should NOT add to window)
        reactor._monotonic = 10.30
        simulate_e_move(enabled_buf, e_delta=2.0, xyz_dist=0.0, speed=40.0)
        assert len(enabled_buf._velocity_window) == window_len

    def test_deretract_clears_retracting_flag(self, enabled_buf, reactor):
        """De-retract should clear _extruder_retracting even without
        updating velocity."""
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        # Retract
        reactor._monotonic = 10.05
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is True
        # De-retract (pure E forward)
        reactor._monotonic = 10.30
        simulate_e_move(enabled_buf, e_delta=2.0, xyz_dist=0.0, speed=40.0)
        assert enabled_buf._extruder_retracting is False

    def test_smoothed_velocity_after_deretract_uses_window(self, enabled_buf, reactor):
        """After de-retract, _smoothed_velocity should return old window
        values, not retract speed."""
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # Extrude at 5 mm/s
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        extrude_vel = enabled_buf.extruder_velocity
        # Retract
        reactor._monotonic = 10.05
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        # De-retract at 40 mm/s (should not pollute window)
        reactor._monotonic = 10.15
        simulate_e_move(enabled_buf, e_delta=2.0, xyz_dist=0.0, speed=40.0)
        smoothed = enabled_buf._smoothed_velocity(10.15)
        # Should return old extrusion velocity (~5), not retract speed (40)
        assert smoothed < 10.0
        assert smoothed > 0.0


class TestPassiveFollow:
    """Passive follow in critical zones when auto_enabled is off."""

    def test_retract_in_full_zone_when_disabled(self, buf, reactor):
        buf.material_present = True
        buf._print_stats.state = "standby"
        set_sensors(buf, full=True)
        reactor._monotonic = 10.0
        simulate_e_move(buf, e_delta=-5.0, xyz_dist=0.0, speed=30.0)
        assert buf.motor_direction == BACK

    def test_feed_in_empty_zone_when_disabled(self, buf, reactor):
        buf.material_present = True
        buf._print_stats.state = "standby"
        set_sensors(buf, empty=True)
        reactor._monotonic = 10.0
        simulate_e_move(buf, e_delta=5.0, xyz_dist=0.0, speed=30.0)
        assert buf.motor_direction == FORWARD

    def test_no_follow_in_middle_zone(self, buf, reactor):
        buf.material_present = True
        buf._print_stats.state = "standby"
        set_sensors(buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(buf, e_delta=-5.0, xyz_dist=0.0, speed=30.0)
        assert buf.motor_direction == STOP

    def test_stops_when_extruder_stops(self, buf, reactor):
        buf.material_present = True
        buf._print_stats.state = "standby"
        set_sensors(buf, full=True)
        reactor._monotonic = 10.0
        simulate_e_move(buf, e_delta=-5.0, xyz_dist=0.0, speed=30.0)
        assert buf.motor_direction == BACK

        # Extruder velocity drops to 0
        buf.extruder_velocity = 0.0
        buf._evaluate_and_drive(10.5)
        assert buf.motor_direction == STOP

    def test_no_passive_follow_when_printing(self, buf, reactor):
        buf.material_present = True
        buf._print_stats.state = "printing"
        set_sensors(buf, full=True)
        reactor._monotonic = 10.0
        simulate_e_move(buf, e_delta=-5.0, xyz_dist=0.0, speed=30.0)
        assert buf.motor_direction == STOP

    def test_no_passive_follow_without_material(self, buf, reactor):
        buf.material_present = False
        buf._print_stats.state = "standby"
        set_sensors(buf, full=True)
        reactor._monotonic = 10.0
        # Can't use simulate_e_move since follow_retract gate in hook
        # requires material detection path — set state directly
        buf._extruder_retracting = True
        buf.extruder_velocity = 30.0
        buf._evaluate_and_drive(10.0)
        assert buf.motor_direction == STOP
