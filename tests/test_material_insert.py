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
        # Trigger material switch (state=1 means material present)
        buttons.callbacks["PE3"](10.0, 1)
        assert buf.auto_enabled is True
        # All sensors false = EMPTY_MIDDLE zone, which feeds forward
        assert buf.state == STATE_FEEDING
        assert buf.material_present is True
        assert buf._initial_fill_until > 0.0

    def test_resets_state_on_insert(self, buf, buttons, reactor):
        buf._burst_count = 3
        buf._extruder_retracting = True
        buf.error_msg = "old error"
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 1)
        assert buf._burst_count == 0
        assert buf._extruder_retracting is False
        assert buf.error_msg == ""
        assert buf._initial_fill_until > 0.0


class TestMaterialRemoval:
    def test_stops_motor_on_removal(self, enabled_buf, buttons, reactor):
        enabled_buf.material_present = True
        reactor._monotonic = 10.0
        # Remove material (state=0)
        buttons.callbacks["PE3"](10.0, 0)
        assert enabled_buf.material_present is False
        assert enabled_buf.motor_direction == STOP
        assert enabled_buf.state == STATE_IDLE

    def test_triggers_pause_on_runout(self, enabled_buf, buttons, reactor, gcode):
        enabled_buf.material_present = True
        enabled_buf.pause_on_runout = True
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 0)
        assert "PAUSE" in gcode.scripts_run

    def test_no_pause_when_disabled(self, enabled_buf, buttons, reactor, gcode):
        enabled_buf.material_present = True
        enabled_buf.pause_on_runout = False
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 0)
        assert "PAUSE" not in gcode.scripts_run

    def test_removal_when_not_enabled_no_pause(self, buf, buttons, reactor, gcode):
        buf.material_present = True
        buf.auto_enabled = False
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 0)
        # Not auto_enabled, so no pause/stop logic triggered
        assert buf.material_present is False
