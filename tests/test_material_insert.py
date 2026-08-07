"""Tests for material insertion/removal and auto-enable."""

import pytest
from conftest import (
    STOP,
    STATE_IDLE,
    STATE_STOPPED,
    STATE_FEEDING,
    STATE_DISABLED,
)


class TestMaterialInsertion:
    def test_auto_enables_on_insert(self, buf, buttons, reactor):
        reactor._monotonic = 10.0
        assert buf.auto_enabled is False
        buttons.callbacks["PE3"](10.0, 1)
        assert buf.auto_enabled is True
        assert buf.state == STATE_FEEDING
        assert buf.material_present is True

    def test_resets_error_on_insert(self, buf, buttons, reactor):
        buf.error_msg = "old error"
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 1)
        assert buf.error_msg == ""

    def test_disabled_ignores_insert(self, buf, buttons, reactor):
        buf.state = STATE_DISABLED
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 1)
        assert buf.state == STATE_DISABLED
        assert buf.auto_enabled is False


class TestMaterialRemoval:
    def test_stops_on_removal(self, enabled_buf, buttons, reactor):
        enabled_buf.material_present = True
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 0)
        assert enabled_buf.material_present is False
        assert enabled_buf.motor_direction == STOP
        assert enabled_buf.state == STATE_IDLE

    def test_triggers_pause_on_runout(self, enabled_buf, buttons, reactor,
                                      gcode):
        enabled_buf.material_present = True
        enabled_buf.pause_on_runout = True
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 0)
        assert "PAUSE" in gcode.scripts_run

    def test_no_pause_when_disabled(self, enabled_buf, buttons, reactor,
                                     gcode):
        enabled_buf.material_present = True
        enabled_buf.pause_on_runout = False
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 0)
        assert "PAUSE" not in gcode.scripts_run

    def test_removal_when_not_enabled_no_pause(self, buf, buttons, reactor,
                                                gcode):
        buf.material_present = True
        buf.auto_enabled = False
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 0)
        assert buf.material_present is False


class TestRunoutNoResync:
    """After a runout the buffer must stay unsynced until filament is
    re-inserted: _material_callback leaves auto_enabled=True in IDLE,
    and filament is still moving during the runout, so without the
    material gate the very next hall edge would re-sync a buffer with
    nothing to feed."""

    def test_runout_hall_edge_does_not_resync(
            self, enabled_buf, buttons, reactor):
        from conftest import trigger_sensor
        enabled_buf.material_present = True
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 0)  # runout
        assert enabled_buf._synced_to is None
        trigger_sensor(buttons, "PE1", True, 10.1)  # middle edge
        assert enabled_buf._synced_to is None
        assert enabled_buf.state == STATE_IDLE

    def test_runout_control_tick_does_not_resync(
            self, enabled_buf, buttons, reactor):
        from conftest import set_sensors
        vactual_writes = (
            enabled_buf.printer.tmc2208.mcu_tmc.vactual_writes)
        enabled_buf.material_present = True
        enabled_buf._print_stats.state = "printing"
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 0)  # runout mid-print
        set_sensors(enabled_buf, empty=True)
        enabled_buf._control_timer_cb(10.5)
        # No re-sync, and no surprise EMPTY recovery without filament.
        assert enabled_buf._synced_to is None
        assert vactual_writes == []
