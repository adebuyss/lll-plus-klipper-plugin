"""Tests for burst feed cycle logic in the empty zone."""

import pytest
from conftest import (
    FORWARD,
    STOP,
    STATE_STOPPED,
    STATE_FEEDING,
    STATE_ERROR,
    set_sensors,
)


class TestBurstDelay:
    def test_first_empty_starts_delay(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf._burst_delay_start == t
        assert enabled_buf.motor_direction == STOP

    def test_during_delay_stays_stopped(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(t)

        # Advance less than burst_delay (0.5s)
        t += 0.3
        reactor._monotonic = t
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf.motor_direction == STOP
        assert enabled_buf._burst_count == 0


class TestBurstFeed:
    def test_after_delay_starts_burst(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(t)

        # Advance past burst_delay
        t += 0.6
        reactor._monotonic = t
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf._burst_count == 1
        assert enabled_buf.motor_direction == FORWARD
        assert enabled_buf._burst_until > t

    def test_burst_expires_returns_to_stop(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 0.0

        # Start delay
        enabled_buf._evaluate_and_drive(t)

        # Trigger burst
        t += 0.6
        reactor._monotonic = t
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf.motor_direction == FORWARD

        # Advance past burst_feed_time (0.5s)
        t += 0.6
        reactor._monotonic = t
        enabled_buf._evaluate_and_drive(t)
        # Burst expired, starts new delay
        assert enabled_buf.motor_direction == STOP
        assert enabled_buf._burst_delay_start == t

    def test_multiple_burst_cycles(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 0.0

        for cycle in range(3):
            # Delay
            enabled_buf._evaluate_and_drive(t)
            t += 0.6
            reactor._monotonic = t
            # Burst
            enabled_buf._evaluate_and_drive(t)
            assert enabled_buf._burst_count == cycle + 1
            assert enabled_buf.motor_direction == FORWARD
            # Expire
            t += 0.6
            reactor._monotonic = t
            enabled_buf._evaluate_and_drive(t)

        assert enabled_buf._burst_count == 3


class TestBurstExhaustion:
    def test_max_bursts_triggers_error(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 0.0

        for cycle in range(5):
            # Delay
            enabled_buf._evaluate_and_drive(t)
            t += 0.6
            reactor._monotonic = t
            # Burst
            enabled_buf._evaluate_and_drive(t)
            t += 0.6
            reactor._monotonic = t
            # Expire
            enabled_buf._evaluate_and_drive(t)

        # 6th attempt should error
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf.state == STATE_ERROR
        assert "burst" in enabled_buf.error_msg.lower()


class TestBurstReset:
    def test_extrusion_resets_burst(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 0.0

        # Start burst cycle
        enabled_buf._evaluate_and_drive(t)
        t += 0.6
        reactor._monotonic = t
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf._burst_count == 1

        # Extrusion starts
        enabled_buf.extruder_velocity = 5.0
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf._burst_delay_start == 0.0
        assert enabled_buf._burst_until == 0.0
        assert enabled_buf._burst_count == 0

    def test_leaving_empty_zone_resets_burst(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 0.0
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf._burst_delay_start == t

        # Move to middle zone
        set_sensors(enabled_buf, middle=True)
        enabled_buf._evaluate_and_drive(t + 0.1)
        assert enabled_buf._burst_delay_start == 0.0
        assert enabled_buf._burst_count == 0

    def test_active_burst_continues_in_no_sensor_zone(self, enabled_buf, reactor):
        set_sensors(enabled_buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        enabled_buf.extruder_velocity = 0.0

        # Start delay then burst
        enabled_buf._evaluate_and_drive(t)
        t += 0.6
        reactor._monotonic = t
        enabled_buf._evaluate_and_drive(t)
        burst_until = enabled_buf._burst_until
        assert enabled_buf.motor_direction == FORWARD

        # Leave empty zone but burst still active
        set_sensors(enabled_buf)
        t += 0.1  # still within burst window
        reactor._monotonic = t
        enabled_buf._evaluate_and_drive(t)
        assert enabled_buf.motor_direction == FORWARD
