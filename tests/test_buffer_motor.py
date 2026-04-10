"""Tests for BufferMotor: VACTUAL writes, caching, direction changes."""

import pytest
from conftest import FORWARD, BACK, STOP


class TestHandleReady:
    def test_reads_microsteps_from_chopconf(self, buf, mcu_tmc):
        # CHOPCONF 0x04000000 -> mres=4 -> microsteps=256>>4=16
        # But our default is mres in bits 27-24 = 0x04 -> mres=4 -> 16
        assert buf.motor.microsteps == 16

    def test_enables_stepper_at_startup(self, buf, printer):
        assert printer.manual_stepper.enabled is True

    def test_writes_vactual_zero_at_startup(self, buf, mcu_tmc):
        # Should have written VACTUAL=0 during handle_ready
        vactual_writes = [(n, v) for n, v in mcu_tmc.write_log if n == "VACTUAL"]
        assert vactual_writes[0] == ("VACTUAL", 0)

    def test_computes_vactual_value(self, buf):
        # Formula: RPM * microsteps * 200 / 60 / 0.715
        expected = int(260 * 16 * 200 / 60.0 / 0.715)
        assert buf.motor.vactual_value == expected


class TestSetVelocity:
    def test_forward_sets_shaft_and_vactual(self, buf, mcu_tmc):
        mcu_tmc.write_log.clear()
        buf.motor.set_velocity(FORWARD)
        # shaft=1 for forward
        gconf_writes = [v for n, v in mcu_tmc.write_log if n == "GCONF"]
        assert len(gconf_writes) >= 1
        assert gconf_writes[0] & (1 << 3)  # shaft bit set
        vactual_writes = [v for n, v in mcu_tmc.write_log if n == "VACTUAL"]
        assert vactual_writes[-1] == buf.motor.vactual_value

    def test_back_sets_shaft_zero(self, buf, mcu_tmc):
        mcu_tmc.write_log.clear()
        buf.motor.set_velocity(BACK)
        gconf_writes = [v for n, v in mcu_tmc.write_log if n == "GCONF"]
        assert len(gconf_writes) >= 1
        assert not (gconf_writes[0] & (1 << 3))  # shaft bit clear

    def test_stop_writes_vactual_zero(self, buf, mcu_tmc):
        buf.motor.set_velocity(FORWARD)
        mcu_tmc.write_log.clear()
        buf.motor.set_velocity(STOP)
        vactual_writes = [v for n, v in mcu_tmc.write_log if n == "VACTUAL"]
        assert vactual_writes[-1] == 0

    def test_direction_change_stops_first(self, buf, mcu_tmc):
        buf.motor.set_velocity(FORWARD)
        mcu_tmc.write_log.clear()
        buf.motor.set_velocity(BACK)
        # First write should be VACTUAL=0 (stop)
        assert mcu_tmc.write_log[0] == ("VACTUAL", 0)

    def test_stop_when_already_stopped_is_noop(self, buf, mcu_tmc):
        buf.motor.set_velocity(STOP)
        mcu_tmc.write_log.clear()
        buf.motor.set_velocity(STOP)
        assert len(mcu_tmc.write_log) == 0


class TestCaching:
    def test_same_direction_same_speed_skips_writes(self, buf, mcu_tmc):
        buf.motor.set_velocity(FORWARD)
        mcu_tmc.write_log.clear()
        buf.motor.set_velocity(FORWARD)
        # No new writes — shaft and vactual unchanged
        assert len(mcu_tmc.write_log) == 0

    def test_same_direction_different_speed_writes_vactual(self, buf, mcu_tmc):
        buf.motor.set_velocity(FORWARD)
        mcu_tmc.write_log.clear()
        new_vactual = buf.motor.vactual_value + 100
        buf.motor.set_velocity(FORWARD, new_vactual)
        vactual_writes = [v for n, v in mcu_tmc.write_log if n == "VACTUAL"]
        assert vactual_writes[-1] == new_vactual


class TestRetry:
    def test_write_vactual_retries_on_failure(self, buf, mcu_tmc):
        # Make first 2 writes fail, third succeeds
        call_count = [0]
        original = mcu_tmc.set_register

        def failing_set_register(name, value):
            call_count[0] += 1
            if name == "VACTUAL" and call_count[0] <= 2:
                raise Exception("simulated failure")
            original(name, value)

        mcu_tmc.set_register = failing_set_register
        buf.motor._last_vactual = -1  # force write
        buf.motor._write_vactual(1234)
        assert call_count[0] == 3
        assert buf.motor._last_vactual == 1234


class TestEmergencyStop:
    def test_resets_all_state(self, buf):
        buf.motor.set_velocity(FORWARD)
        buf.motor.emergency_stop()
        assert buf.motor.current_direction == STOP
        assert buf.motor._enabled is False
        assert buf.motor._last_shaft == -1
        assert buf.motor._last_vactual == -1
