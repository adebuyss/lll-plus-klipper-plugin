"""Tests for debug observability: the periodic heartbeat, sync/unsync
reason slugs, and successful-VACTUAL-write logging.

These exist because the Aug 2026 MCU-shutdown forensics hit a 433 s
log blind spot before the fault — every log site was edge-triggered,
590 recoveries left no record of the registers written, and a bare
"unsynced" line couldn't be attributed to any of its 13 call sites.
"""

import logging

import pytest
from conftest import set_sensors


class TestHeartbeat:
    def test_heartbeat_emitted_when_debug(self, buf, caplog):
        caplog.set_level(logging.INFO)
        buf.debug = True
        buf._control_timer_cb(20.0)
        assert "heartbeat" in caplog.text
        assert "zone=" in caplog.text
        assert "state=" in caplog.text
        assert "margin=" in caplog.text

    def test_heartbeat_throttles_to_interval(self, buf, caplog):
        caplog.set_level(logging.INFO)
        buf.debug = True
        buf._control_timer_cb(20.0)
        buf._control_timer_cb(20.5)
        buf._control_timer_cb(25.0)
        assert caplog.text.count("heartbeat") == 1
        buf._control_timer_cb(31.0)
        assert caplog.text.count("heartbeat") == 2

    def test_heartbeat_silent_without_debug(self, buf, caplog):
        caplog.set_level(logging.INFO)
        assert buf.debug is False
        buf._control_timer_cb(20.0)
        assert "heartbeat" not in caplog.text

    def test_heartbeat_margin_uses_nonflushing_status(
            self, buf, caplog):
        """Margin must come from toolhead.get_status print_time minus
        the stepper MCU's estimated_print_time — with the mocks that
        is _last_move_time - eventtime, proving the wiring (and that
        the flushing get_last_move_time path is not involved: the
        mock records flush-free access via get_status)."""
        caplog.set_level(logging.INFO)
        buf.debug = True
        buf.toolhead._last_move_time = 5.0
        flushes_before = buf.toolhead.flush_count
        buf._control_timer_cb(20.0)
        assert "margin=-15.000" in caplog.text
        assert buf.toolhead.flush_count == flushes_before

    def test_heartbeat_reports_in_manual_state(self, buf, caplog):
        """Placed before the MANUAL_* early returns on purpose."""
        from conftest import STATE_MANUAL_FEED
        caplog.set_level(logging.INFO)
        buf.debug = True
        buf.state = STATE_MANUAL_FEED
        buf._control_timer_cb(20.0)
        assert "heartbeat" in caplog.text


class TestSyncReasons:
    def test_unsync_reason_in_log(self, enabled_buf, caplog):
        from conftest import MockGcmd
        caplog.set_level(logging.INFO)
        enabled_buf.debug = True
        enabled_buf.cmd_BUFFER_DISABLE(MockGcmd())
        assert "unsynced (disable)" in caplog.text

    def test_sync_reason_in_log(self, buf, caplog):
        from conftest import MockGcmd
        caplog.set_level(logging.INFO)
        buf.debug = True
        buf.cmd_BUFFER_ENABLE(MockGcmd())
        assert "(enable)" in caplog.text

    def test_runout_reason_in_log(self, enabled_buf, buttons, reactor,
                                  caplog):
        caplog.set_level(logging.INFO)
        enabled_buf.debug = True
        enabled_buf.material_present = True
        reactor._monotonic = 10.0
        buttons.callbacks["PE3"](10.0, 0)
        assert "unsynced (runout)" in caplog.text


class TestVactualWriteLogging:
    def test_successful_write_logged(self, buf, caplog):
        caplog.set_level(logging.INFO)
        buf.debug = True
        buf._write_vactual(1234)
        assert "VACTUAL write 1234 ok" in caplog.text

    def test_failed_write_not_logged_as_ok(self, buf, mcu_tmc, caplog):
        caplog.set_level(logging.INFO)
        buf.debug = True
        mcu_tmc.fail_registers.add("VACTUAL")
        buf._write_vactual(1234)
        assert "VACTUAL write 1234 ok" not in caplog.text
        assert "VACTUAL write failed" in caplog.text
