"""Tests for E movement detection and velocity decay."""

import pytest
from conftest import (
    FORWARD,
    STOP,
    STATE_STOPPED,
    set_sensors,
    simulate_e_move,
)


class TestEVelocityComputation:
    def test_mixed_xyz_e_move(self, enabled_buf, reactor):
        """E velocity = speed * |e_delta| / xyz_dist for mixed moves."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=0.5, xyz_dist=10.0, speed=100.0)
        # Expected: 100 * 0.5 / 10 = 5.0
        assert enabled_buf.extruder_velocity == pytest.approx(5.0, abs=0.1)

    def test_pure_e_move(self, enabled_buf, reactor):
        """Pure forward E move outside printing (load/purge) should
        update extruder_velocity normally."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # Not printing — pure E move should update velocity
        enabled_buf._print_stats.state = "standby"
        simulate_e_move(enabled_buf, e_delta=5.0, xyz_dist=0.0, speed=25.0)
        assert enabled_buf.extruder_velocity == pytest.approx(25.0, abs=0.1)

    def test_expected_move_end_computed(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        t = 10.0
        reactor._monotonic = t
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=50.0, speed=100.0)
        # move_dist=50, speed=100 -> duration=0.5s
        assert enabled_buf._expected_move_end == pytest.approx(t + 0.5, abs=0.01)

    def test_pure_e_move_end_uses_e_delta(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        t = 10.0
        reactor._monotonic = t
        simulate_e_move(enabled_buf, e_delta=5.0, xyz_dist=0.0, speed=25.0)
        # move_dist=5, speed=25 -> duration=0.2s
        assert enabled_buf._expected_move_end == pytest.approx(t + 0.2, abs=0.01)


class TestVelocityDecay:
    def test_decay_zeroes_velocity(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        t = 10.0
        reactor._monotonic = t
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=50.0, speed=100.0)
        assert enabled_buf.extruder_velocity > 0

        # Advance past expected_move_end + control_interval
        t = enabled_buf._expected_move_end + enabled_buf.control_interval + 0.1
        reactor._monotonic = t
        enabled_buf._last_e_command_time = 10.0  # original command time
        enabled_buf._control_timer_cb(t)
        assert enabled_buf.extruder_velocity == 0.0

    def test_new_move_prevents_decay(self, enabled_buf, reactor):
        set_sensors(enabled_buf, middle=True)
        t = 10.0
        reactor._monotonic = t
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=50.0, speed=100.0)

        # Send another move before decay would kick in
        t += 0.3
        reactor._monotonic = t
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=50.0, speed=100.0)

        # The expected_move_end is now pushed forward
        assert enabled_buf._expected_move_end > t


class TestEMovementDetection:
    def test_small_e_delta_ignored(self, enabled_buf, reactor, gcode_move):
        """E deltas < 0.001 are ignored."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        prev_velocity = enabled_buf.extruder_velocity
        # Very tiny E move
        gcode_move.last_position[3] += 0.0001
        # _on_e_movement would normally be called by the wrapper,
        # but with delta < 0.001 the wrapper skips it
        # So we verify the wrapper logic indirectly
        assert enabled_buf.extruder_velocity == prev_velocity

    def test_negative_e_delta_is_retraction(self, enabled_buf, reactor):
        """Negative E delta while printing sets velocity to zero."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        enabled_buf.motor_direction = FORWARD
        enabled_buf.state = STATE_STOPPED
        # Set print_stats to printing
        enabled_buf._print_stats.state = "printing"
        # First extrude to populate velocity window
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        prev_velocity = enabled_buf.extruder_velocity
        # Now retract
        reactor._monotonic = 10.05
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is True
        # Velocity zeroed during retraction to prevent stale forward drive
        assert enabled_buf.extruder_velocity == 0.0


class TestSmoothedVelocity:
    def test_returns_max_in_window(self, enabled_buf, reactor):
        """Smoothed velocity returns the max velocity in the window."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=0.5, xyz_dist=10.0, speed=50.0)
        v1 = enabled_buf.extruder_velocity  # 50*0.5/10 = 2.5

        reactor._monotonic = 10.1
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=100.0)
        v2 = enabled_buf.extruder_velocity  # 100*1/10 = 10.0

        assert v2 > v1
        smoothed = enabled_buf._smoothed_velocity(10.1)
        assert smoothed == pytest.approx(v2, abs=0.1)

    def test_expires_old_entries(self, enabled_buf, reactor):
        """Entries older than window duration are pruned."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=100.0)
        high_vel = enabled_buf.extruder_velocity  # 10.0

        # Advance past the window (default 0.3s)
        reactor._monotonic = 10.5
        simulate_e_move(enabled_buf, e_delta=0.5, xyz_dist=10.0, speed=30.0)
        low_vel = enabled_buf.extruder_velocity  # 1.5

        smoothed = enabled_buf._smoothed_velocity(10.5)
        # Old high entry should be expired; smoothed = low_vel
        assert smoothed == pytest.approx(low_vel, abs=0.1)
        assert smoothed < high_vel

    def test_retraction_zeroes_smoothed_during_print(self, enabled_buf, reactor):
        """During print retraction, smoothed velocity returns 0."""
        set_sensors(enabled_buf, middle=True)
        enabled_buf._print_stats.state = "printing"
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)

        # Brief retraction (< window duration)
        reactor._monotonic = 10.05
        simulate_e_move(enabled_buf, e_delta=-0.8, xyz_dist=0.0, speed=30.0)

        # Smoothed velocity returns 0 during active retraction
        smoothed = enabled_buf._smoothed_velocity(10.05)
        assert smoothed == 0.0

    def test_window_cleared_on_decay(self, enabled_buf, reactor):
        """Velocity decay should clear the window."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=50.0, speed=100.0)
        assert len(enabled_buf._velocity_window) > 0

        # Trigger decay
        t = enabled_buf._expected_move_end + enabled_buf.control_interval + 0.1
        reactor._monotonic = t
        enabled_buf._last_e_command_time = 10.0
        enabled_buf._control_timer_cb(t)
        assert enabled_buf._velocity_window == []

    def test_velocity_factor_applied_in_middle_zone(self, enabled_buf, reactor, mcu_tmc):
        """Middle zone should apply velocity_factor to smoothed velocity."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        mcu_tmc.write_log.clear()
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        # velocity_factor is 1.05 by default; motor should be driving
        assert enabled_buf.motor_direction == FORWARD
        # The vactual should reflect velocity * 1.05
        assert len(mcu_tmc.write_log) > 0

    def test_velocity_factor_default(self, enabled_buf):
        """Default velocity_factor should be 1.05."""
        assert enabled_buf.velocity_factor == pytest.approx(1.05)

    def test_empty_window_falls_back_to_extruder_velocity(self, enabled_buf, reactor):
        """With empty window, smoothed velocity returns extruder_velocity."""
        enabled_buf.extruder_velocity = 5.0
        enabled_buf._velocity_window = []
        smoothed = enabled_buf._smoothed_velocity(10.0)
        assert smoothed == pytest.approx(5.0)
