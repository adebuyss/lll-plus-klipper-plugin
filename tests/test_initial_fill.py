"""Tests for initial fill mode on material insertion."""

from conftest import (
    FORWARD,
    STOP,
    STATE_STOPPED,
    STATE_FEEDING,
    ZONE_MIDDLE,
    MockGcmd,
    set_sensors,
)


class TestInitialFillActivation:
    def test_material_insert_activates_fill(self, buf, buttons, reactor):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        assert buf._initial_fill_until == t + buf.initial_fill_timeout

    def test_fill_feeds_forward(self, buf, buttons, reactor):
        set_sensors(buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        assert buf.motor_direction == FORWARD

    def test_fill_bypasses_burst(self, buf, buttons, reactor):
        set_sensors(buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)

        # Advance 4 seconds (well past burst exhaustion threshold)
        for _ in range(16):
            t += 0.25
            reactor._monotonic = t
            buf._evaluate_and_drive(t)
        assert buf._burst_count == 0
        assert buf.motor_direction == FORWARD


class TestInitialFillTimeout:
    def test_timeout_sets_stopped(self, buf, buttons, reactor):
        set_sensors(buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)

        # Advance past initial_fill_timeout (10s)
        t += 11.0
        reactor._monotonic = t
        buf._evaluate_and_drive(t)
        assert buf.state == STATE_STOPPED
        assert buf.motor_direction == STOP
        assert buf.auto_enabled is True
        assert buf._initial_fill_until == 0.0

    def test_normal_zone_logic_after_timeout(self, buf, buttons, reactor):
        set_sensors(buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)

        # Timeout
        t += 11.0
        reactor._monotonic = t
        buf._evaluate_and_drive(t)
        assert buf.state == STATE_STOPPED

        # Now set extruder velocity and re-evaluate — normal zone logic
        buf.extruder_velocity = 50.0
        buf._velocity_window = [(t, 50.0)]
        buf._evaluate_and_drive(t)
        assert buf.motor_direction == FORWARD
        assert buf.state == STATE_FEEDING


class TestInitialFillClear:
    def test_leaving_empty_clears_fill(self, buf, buttons, reactor):
        set_sensors(buf, empty=True)
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        assert buf._initial_fill_until > 0.0

        # Filament reaches middle sensor
        set_sensors(buf, middle=True)
        t += 1.0
        reactor._monotonic = t
        buf._evaluate_and_drive(t)
        assert buf._initial_fill_until == 0.0
        assert buf._current_zone == ZONE_MIDDLE

    def test_disable_clears_fill(self, buf, buttons, reactor):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        assert buf._initial_fill_until > 0.0

        buf.cmd_BUFFER_DISABLE(MockGcmd("BUFFER_DISABLE"))
        assert buf._initial_fill_until == 0.0

    def test_enable_does_not_set_fill(self, buf):
        buf.cmd_BUFFER_ENABLE(MockGcmd("BUFFER_ENABLE"))
        assert buf._initial_fill_until == 0.0

    def test_material_removal_during_fill(self, buf, buttons, reactor):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        assert buf._initial_fill_until > 0.0

        # Remove material
        t += 2.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 0)
        assert buf.motor_direction == STOP
        assert buf.material_present is False
