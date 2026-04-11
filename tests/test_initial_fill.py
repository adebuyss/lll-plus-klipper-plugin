"""Tests for initial fill mode on material insertion."""

from conftest import (
    FORWARD,
    STOP,
    STATE_STOPPED,
    STATE_FEEDING,
    MockGcmd,
    set_sensors,
)


class TestInitialFillActivation:
    def test_material_insert_starts_fill(self, buf, buttons, reactor,
                                         force_move):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        assert buf.state == STATE_FEEDING
        assert buf.motor_direction == FORWARD
        # Should have issued a force_move for the fill
        assert len(force_move.moves) > 0
        assert force_move.moves[-1][1] > 0  # positive distance

    def test_fill_timeout_stored(self, buf, buttons, reactor):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)
        # After fill completes, initial_fill_until should be cleared
        assert buf._initial_fill_until == 0.0


class TestInitialFillClear:
    def test_disable_during_fill(self, buf, buttons, reactor):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)

        buf.cmd_BUFFER_DISABLE(MockGcmd("BUFFER_DISABLE"))
        assert buf._synced_to is None

    def test_enable_does_not_set_fill(self, buf):
        buf.cmd_BUFFER_ENABLE(MockGcmd("BUFFER_ENABLE"))
        assert buf._initial_fill_until == 0.0

    def test_material_removal_during_fill(self, buf, buttons, reactor):
        t = 10.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 1)

        # Remove material
        t += 2.0
        reactor._monotonic = t
        buttons.callbacks["PE3"](t, 0)
        assert buf.motor_direction == STOP
        assert buf.material_present is False
