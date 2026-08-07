"""Tests for error conditions: safety timeouts, sensor conflicts."""

import pytest
from conftest import (
    STATE_FEEDING,
    STATE_ERROR,
    STATE_STOPPED,
    set_sensors,
)


class TestEmptySafetyTimeout:
    def test_empty_timeout_triggers_error(self, enabled_buf, reactor):
        # No extruder rate set, so _recovery_decision returns ENTER for
        # EMPTY at manual_speed.  Recovery preserves _safety_zone_start
        # so the cumulative empty_safety_timeout still escalates.
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, empty=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)
        assert enabled_buf._safety_zone_start == t

        t += enabled_buf.empty_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert enabled_buf.state == STATE_ERROR
        assert "empty" in enabled_buf.error_msg.lower()

    def test_no_timeout_when_not_printing(self, enabled_buf, reactor):
        """Safety timeout must not fire when the extruder is idle."""
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, empty=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)

        t += enabled_buf.empty_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert enabled_buf.state != STATE_ERROR


class TestFullSafetyTimeout:
    def test_full_timeout_triggers_retract(self, enabled_buf, reactor,
                                           sidecar_moves):
        enabled_buf._print_stats.state = "printing"
        set_sensors(enabled_buf, full=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)

        t += enabled_buf.full_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        # Should have done a safety retract via force_move
        assert len(sidecar_moves) > 0
        assert sidecar_moves[-1][1] < 0  # negative dist = retract

    def test_no_retract_when_not_printing(self, enabled_buf, reactor,
                                           sidecar_moves):
        """Safety retract must not fire when the extruder is idle."""
        enabled_buf._print_stats.state = "standby"
        set_sensors(enabled_buf, full=True)
        t = 1.0
        reactor._monotonic = t
        enabled_buf._update_rotation_distance(t)

        t += enabled_buf.full_safety_timeout + 1.0
        reactor._monotonic = t
        enabled_buf._control_timer_cb(t)
        assert len(sidecar_moves) == 0


class TestSensorConflict:
    def test_conflict_triggers_error(self, enabled_buf):
        set_sensors(enabled_buf, empty=True, full=True)
        enabled_buf._update_rotation_distance(1.0)
        # First tick: deferred (might be transient out-of-order MCU
        # report).  Control-timer promotes a persistent conflict to a
        # hard error once it survives control_interval.
        assert enabled_buf.state != STATE_ERROR
        enabled_buf._control_timer_cb(1.0 + enabled_buf.control_interval)
        assert enabled_buf.state == STATE_ERROR
        assert "conflict" in enabled_buf.error_msg.lower()

    def test_error_stops_motor(self, enabled_buf):
        set_sensors(enabled_buf, empty=True, full=True)
        enabled_buf._update_rotation_distance(1.0)
        enabled_buf._control_timer_cb(1.0 + enabled_buf.control_interval)
        assert enabled_buf.motor_direction == "stop"

    def test_transient_conflict_does_not_error(self, enabled_buf):
        # First sensor flip leaves empty+full both true momentarily;
        # the next flip resolves it.  Buffer must not crash the print.
        set_sensors(enabled_buf, empty=True, full=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf.state != STATE_ERROR
        # Resolve the transient — empty inactive, normal FULL_MIDDLE.
        set_sensors(enabled_buf, empty=False, middle=True, full=True)
        enabled_buf._update_rotation_distance(1.05)
        # _conflict_since cleared, state stable.
        enabled_buf._control_timer_cb(1.0 + enabled_buf.control_interval)
        assert enabled_buf.state != STATE_ERROR
        assert enabled_buf._conflict_since == 0.0


class TestErrorBlocking:
    def test_error_blocks_sensor_callback(self, enabled_buf, buttons):
        enabled_buf.state = STATE_ERROR
        enabled_buf.error_msg = "test error"
        set_sensors(enabled_buf, middle=True)
        # Sensor callback should not update rd when in error state
        old_mult = enabled_buf._rd_multiplier
        # Directly call the sensor callback
        buttons.callbacks["PE1"](10.0, 0)  # trigger middle
        # Multiplier unchanged because error state blocks update
        assert enabled_buf.state == STATE_ERROR

    def test_error_blocks_timer_sync(self, enabled_buf, reactor):
        enabled_buf.state = STATE_ERROR
        t = 10.0
        reactor._monotonic = t
        ret = enabled_buf._control_timer_cb(t)
        assert ret == pytest.approx(t + enabled_buf.control_interval)


class TestUnsyncClearsSafetyTimer:
    def test_unsync_clears_safety_timer(self, enabled_buf):
        """Explicit _unsync (not the recovery-internal one) must clear
        the safety arming so a subsequent re-sync starts fresh."""
        enabled_buf._safety_zone_start = 1.0
        enabled_buf._unsync()
        assert enabled_buf._safety_zone_start == 0.0


class TestClearError:
    def test_clear_error_returns_to_stopped(self, enabled_buf):
        enabled_buf.state = STATE_ERROR
        enabled_buf.error_msg = "test error"
        enabled_buf._clear_error()
        assert enabled_buf.state == STATE_STOPPED
        assert enabled_buf.error_msg == ""

    def test_clear_error_returns_to_idle_if_not_enabled(self, buf):
        buf.state = STATE_ERROR
        buf.auto_enabled = False
        buf._clear_error()
        assert buf.state == "idle"


class TestPolicyKnobs:
    """_is_printing fails safe without print_stats, and pause_on_error
    (default-chained to pause_on_runout) gates error pausing
    independently of runout pausing."""

    def _make_buf(self, printer, **overrides):
        import buffer as buffer_module
        from conftest import MockConfig, DEFAULT_CONFIG
        values = dict(DEFAULT_CONFIG)
        values.update(overrides)
        b = buffer_module.Buffer(MockConfig(printer, values))
        for handler in printer.event_handlers.get("klippy:ready", []):
            handler()
        b._initial_state_received = True
        b._any_sensor_reported = True
        b.sensor_states = {"empty": False, "middle": False,
                           "full": False}
        return b

    def test_is_printing_false_without_print_stats(self, buf):
        buf._print_stats = None
        assert buf._is_printing() is False

    def test_no_recovery_without_print_stats(
            self, enabled_buf, reactor):
        from conftest import set_sensors
        vactual_writes = (
            enabled_buf.printer.tmc2208.mcu_tmc.vactual_writes)
        enabled_buf._print_stats = None
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(1.0)
        assert enabled_buf._extreme_recovery_active is None
        assert vactual_writes == []

    def test_no_safety_timeout_without_print_stats(
            self, enabled_buf, reactor):
        from conftest import set_sensors, STATE_ERROR
        enabled_buf._print_stats = None
        set_sensors(enabled_buf, empty=True)
        enabled_buf._update_rotation_distance(1.0)
        enabled_buf._control_timer_cb(
            1.0 + enabled_buf.empty_safety_timeout + 1.0)
        assert enabled_buf.state != STATE_ERROR

    def test_pause_on_error_defaults_to_pause_on_runout(
            self, printer, gcode):
        b = self._make_buf(printer, pause_on_runout=False)
        assert b.pause_on_error is False
        b._handle_error("test error")
        assert "PAUSE" not in gcode.scripts_run

    def test_pause_on_error_override_true(self, printer, gcode):
        b = self._make_buf(printer, pause_on_runout=False,
                           pause_on_error=True)
        b._handle_error("test error")
        assert "PAUSE" in gcode.scripts_run

    def test_pause_on_error_override_false(
            self, printer, gcode, buttons, reactor):
        b = self._make_buf(printer, pause_on_runout=True,
                           pause_on_error=False)
        # Error path: no pause.
        b._handle_error("test error")
        assert "PAUSE" not in gcode.scripts_run
        b._clear_error()
        # Runout path: still pauses.
        b.material_present = True
        b.auto_enabled = True
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 0)
        assert "PAUSE" in gcode.scripts_run
