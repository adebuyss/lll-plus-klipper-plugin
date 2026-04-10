"""Tests for deferred motor drive: rate-limiting, coalescing, and timer catch-up."""

import pytest
from conftest import (
    FORWARD,
    BACK,
    STOP,
    STATE_STOPPED,
    STATE_FEEDING,
    set_sensors,
    simulate_e_move,
)


class TestDeferredDriveScheduling:
    def test_first_move_schedules_callback(self, enabled_buf, reactor):
        """First E move should queue a deferred callback."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # _on_e_movement is called but callback not yet fired
        gm = enabled_buf._gcode_move
        prev_pos = list(gm.last_position)
        gm.speed = 50.0
        gm.last_position[3] += 1.0
        gm.last_position[0] += 10.0
        enabled_buf._on_e_movement(1.0, prev_pos)
        # Callback should be pending
        assert enabled_buf._drive_pending is True
        assert len(reactor._pending_callbacks) == 1

    def test_callback_clears_pending_flag(self, enabled_buf, reactor):
        """After flush, _drive_pending should be False."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf._drive_pending is False

    def test_callback_sets_last_drive_time(self, enabled_buf, reactor):
        """Deferred drive should update _last_drive_time."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf._last_drive_time == 10.0


class TestRateLimiting:
    def test_rapid_moves_coalesce(self, enabled_buf, reactor, mcu_tmc):
        """Multiple rapid E moves within the rate limit should produce
        only one UART write (from the first deferred callback)."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        mcu_tmc.write_log.clear()

        # First move — schedules callback
        simulate_e_move(enabled_buf, e_delta=0.5, xyz_dist=10.0, speed=50.0)
        writes_after_first = len(mcu_tmc.write_log)

        # Second move at same time — rate limit blocks new callback
        reactor._monotonic = 10.01  # 10ms later, within 100ms interval
        gm = enabled_buf._gcode_move
        prev_pos = list(gm.last_position)
        gm.speed = 80.0
        gm.last_position[3] += 0.5
        gm.last_position[0] += 10.0
        enabled_buf._on_e_movement(0.5, prev_pos)
        # Rate-limited: schedules a timer instead of immediate callback
        assert enabled_buf._drive_pending is True
        # No new immediate callback — timer used instead
        assert len(reactor._pending_callbacks) == 0
        # Drive timer was activated with correct wake time
        _, wake = reactor._timers[enabled_buf._drive_timer]
        assert wake != reactor.NEVER
        assert wake == pytest.approx(enabled_buf._last_drive_time + enabled_buf._min_drive_interval)

    def test_rate_limited_moves_do_not_accumulate_timers(self, enabled_buf, reactor):
        """Repeated rate-limited moves must reuse a single timer,
        not create new ones each time."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        # First move — immediate callback path
        simulate_e_move(enabled_buf, e_delta=0.5, xyz_dist=10.0, speed=50.0)
        timer_count_baseline = len(reactor._timers)
        # Fire 20 rate-limited cycles
        for i in range(20):
            # Advance past rate limit so timer fires and clears pending
            reactor.advance_time(0.15)
            # Next move within rate limit
            reactor._monotonic += 0.02
            gm = enabled_buf._gcode_move
            prev_pos = list(gm.last_position)
            gm.speed = 50.0
            gm.last_position[3] += 0.1
            gm.last_position[0] += 1.0
            enabled_buf._on_e_movement(0.1, prev_pos)
        # No new timers should have been created
        assert len(reactor._timers) == timer_count_baseline

    def test_after_interval_new_callback_scheduled(self, enabled_buf, reactor):
        """After the rate limit interval passes, a new move should
        schedule a callback."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)

        # Advance past the rate limit
        reactor._monotonic = 10.15  # 150ms later, past 100ms interval
        gm = enabled_buf._gcode_move
        prev_pos = list(gm.last_position)
        gm.speed = 50.0
        gm.last_position[3] += 1.0
        gm.last_position[0] += 10.0
        enabled_buf._on_e_movement(1.0, prev_pos)
        assert enabled_buf._drive_pending is True
        reactor.flush_callbacks()
        assert enabled_buf._last_drive_time == 10.15

    def test_duplicate_callbacks_prevented(self, enabled_buf, reactor):
        """If a callback is already pending, no duplicate should be queued."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        enabled_buf._last_drive_time = 0.0  # ensure rate limit passes

        gm = enabled_buf._gcode_move
        prev_pos = list(gm.last_position)
        gm.speed = 50.0
        gm.last_position[3] += 1.0
        gm.last_position[0] += 10.0
        enabled_buf._on_e_movement(1.0, prev_pos)
        assert len(reactor._pending_callbacks) == 1

        # Another move while callback still pending
        prev_pos = list(gm.last_position)
        gm.last_position[3] += 1.0
        gm.last_position[0] += 10.0
        enabled_buf._on_e_movement(1.0, prev_pos)
        # Still only one pending callback
        assert len(reactor._pending_callbacks) == 1


class TestControlTimerCatchUp:
    def test_timer_catches_rate_limited_updates(self, enabled_buf, reactor):
        """Control timer should perform drive update when rate limit has
        elapsed and motor is active."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.motor_direction == FORWARD

        # Simulate rate-limited state: velocity changed but drive skipped
        enabled_buf.extruder_velocity = 8.0
        old_drive_time = enabled_buf._last_drive_time

        # Advance past rate limit and fire timer
        reactor._monotonic = 10.2
        enabled_buf._control_timer_cb(10.2)
        # Timer should have updated last_drive_time
        assert enabled_buf._last_drive_time == 10.2


class TestDeferredRetraction:
    def test_retraction_during_print_zeroes_velocity(self, enabled_buf, reactor):
        """Retraction during printing should zero velocity so motor
        stops during travel retractions."""
        set_sensors(enabled_buf, middle=True)
        reactor._monotonic = 10.0
        enabled_buf._print_stats.state = "printing"
        simulate_e_move(enabled_buf, e_delta=1.0, xyz_dist=10.0, speed=50.0)
        assert enabled_buf.motor_direction == FORWARD
        prev_velocity = enabled_buf.extruder_velocity

        # Now retract — velocity should be zeroed
        reactor._monotonic = 10.15
        simulate_e_move(enabled_buf, e_delta=-2.0, xyz_dist=0.0, speed=30.0)
        assert enabled_buf._extruder_retracting is True
        # Velocity zeroed during retraction to prevent stale forward drive
        assert enabled_buf.extruder_velocity == 0.0


class TestSensorCallbacksUnchanged:
    def test_sensor_callback_drives_directly(self, enabled_buf, reactor, printer):
        """Sensor callbacks should still call _evaluate_and_drive directly
        (not deferred)."""
        enabled_buf.extruder_velocity = 5.0
        reactor._monotonic = 10.0
        buttons = printer.buttons
        # Trigger middle sensor
        cb = buttons.callbacks["PE1"]
        cb(10.0, 0)  # state=0 -> triggered (inverted)
        # Motor should be driving immediately (no pending callback needed)
        assert enabled_buf.motor_direction == FORWARD


class TestDefaultConfigChange:
    def test_default_control_interval(self, enabled_buf):
        """Default control_interval should be 0.25s (tightened from 0.5s)."""
        assert enabled_buf.control_interval == 0.25

    def test_default_drive_interval(self, enabled_buf):
        """Default drive_interval should be 0.1s."""
        assert enabled_buf._min_drive_interval == 0.1
