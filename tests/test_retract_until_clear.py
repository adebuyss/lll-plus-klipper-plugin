"""Tests for BUFFER_RETRACT_UNTIL_CLEAR command."""

from conftest import (
    BACK,
    STOP,
    STATE_MANUAL_RETRACT,
    STATE_IDLE,
    STATE_ERROR,
    MockGcmd,
)


class TestRetractUntilClear:
    def test_basic_retract_until_clear(self, buf, force_move, reactor):
        """Retracts in chunks until material_present becomes False."""
        buf.material_present = True
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_RETRACT_UNTIL_CLEAR")
        buf.cmd_BUFFER_RETRACT_UNTIL_CLEAR(gcmd)
        assert buf.state == STATE_MANUAL_RETRACT
        assert buf.motor_direction == BACK
        assert len(force_move.moves) > 0
        assert force_move.moves[-1][1] < 0  # negative = retract

        # Simulate filament clearing the switch, then flush
        buf.material_present = False
        reactor._monotonic = 2.0
        reactor.flush_callbacks()

        assert buf.state == STATE_IDLE
        assert buf._retract_until_clear is False

    def test_blocked_in_error(self, buf):
        buf.state = STATE_ERROR
        gcmd = MockGcmd("BUFFER_RETRACT_UNTIL_CLEAR")
        buf.cmd_BUFFER_RETRACT_UNTIL_CLEAR(gcmd)
        assert buf.state == STATE_ERROR

    def test_cancelled_by_buffer_stop(self, buf, force_move, reactor):
        buf.material_present = True
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_RETRACT_UNTIL_CLEAR")
        buf.cmd_BUFFER_RETRACT_UNTIL_CLEAR(gcmd)
        assert buf._retract_until_clear is True

        buf.cmd_BUFFER_STOP(MockGcmd("BUFFER_STOP"))
        assert buf._retract_until_clear is False

        # Pending callbacks should not issue more chunks
        chunks_before = len(force_move.moves)
        reactor.flush_callbacks()
        assert len(force_move.moves) == chunks_before

    def test_custom_speed(self, buf, force_move, reactor):
        buf.material_present = True
        reactor._monotonic = 1.0
        gcmd = MockGcmd("BUFFER_RETRACT_UNTIL_CLEAR", {"SPEED": 30.0})
        buf.cmd_BUFFER_RETRACT_UNTIL_CLEAR(gcmd)
        assert force_move.moves[-1][2] == 30.0
