"""Tests for multi-buffer support with optional extruder binding.

Covers:
- Config parsing: extruder param present/absent, short_name extraction
- Mux command routing via BUFFER=name
- Backward compat: BUFFER_STATUS without BUFFER= works for [buffer]
- Extruder-bound sync/unsync
- Tool-change polling through the control timer
- Unbound buffers re-sync on tool change
- Multiple independent buffers
"""

import logging

import pytest

from conftest import (
    DEFAULT_CONFIG,
    MockConfig,
    MockExtruder,
    MockGcmd,
    MockPrinter,
    MockPrinterExtruderStepper,
    STATE_FEEDING,
    STATE_IDLE,
    STATE_STOPPED,
)

import buffer as buffer_module


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_printer_with_extruders(extruder_names=("extruder",),
                                 stepper_names=("buffer_stepper",)):
    """Construct a MockPrinter with multiple extruders/steppers registered."""
    printer = MockPrinter()
    # Register additional extruder steppers beyond the default one.
    for name in stepper_names:
        key = "extruder_stepper %s" % name
        if key not in printer._objects:
            printer.add_object(key, MockPrinterExtruderStepper())
    # Default toolhead already has MockExtruder(name="extruder").
    # Nothing to do unless a test later calls toolhead.set_extruder().
    return printer


def _make_buffer(printer, section_name, stepper_name="buffer_stepper",
                 extruder=None):
    """Construct a Buffer with the given config and fire klippy:ready."""
    values = dict(DEFAULT_CONFIG)
    values["_name"] = section_name
    values["stepper"] = stepper_name
    if extruder is not None:
        values["extruder"] = extruder
    # Each buffer needs distinct sensor pins to avoid collisions when
    # multiple buffers are registered in one printer.  The MockButtons
    # mock doesn't validate, but we still give each one a unique set to
    # keep callback registration unambiguous.
    suffix = section_name.split(None, 1)[-1]
    for i, k in enumerate(("sensor_empty_pin", "sensor_middle_pin",
                           "sensor_full_pin", "material_switch_pin",
                           "feed_button_pin", "retract_button_pin")):
        values[k] = "P%s%d" % (suffix[:2].upper(), i)
    config = MockConfig(printer, values)
    buf = buffer_module.Buffer(config)
    # Register this buffer for lookup_objects("buffer") so the unbound
    # warning can inspect sibling buffers.
    printer.add_object(section_name, buf)
    return buf


def _fire_ready(printer):
    for handler in printer.event_handlers.get("klippy:ready", []):
        handler()


# ---------------------------------------------------------------------------
# Config parsing
# ---------------------------------------------------------------------------


class TestConfigParsing:
    def test_bare_buffer_short_name(self, buf):
        assert buf.short_name == "buffer"
        assert buf.extruder_name is None

    def test_named_buffer_short_name(self):
        printer = _make_printer_with_extruders()
        buf = _make_buffer(printer, "buffer my_buf")
        assert buf.short_name == "my_buf"

    def test_extruder_param_parsed(self):
        printer = _make_printer_with_extruders()
        buf = _make_buffer(printer, "buffer t0", extruder="extruder1")
        assert buf.extruder_name == "extruder1"

    def test_get_status_includes_name_and_extruder(self, buf):
        status = buf.get_status(0.0)
        assert status["name"] == "buffer"
        assert status["extruder"] is None


# ---------------------------------------------------------------------------
# Mux command registration and dispatch
# ---------------------------------------------------------------------------


class TestMuxDispatch:
    def test_bare_buffer_registers_none_default(self, buf, printer):
        # BUFFER_STATUS should be routable without a BUFFER= param for the
        # bare [buffer] section.
        entry = printer.gcode.mux_commands["BUFFER_STATUS"]
        key, values = entry
        assert key == "BUFFER"
        assert "buffer" in values
        assert None in values  # backward-compat default

    def test_named_buffer_does_not_register_none_default(self):
        printer = _make_printer_with_extruders(
            stepper_names=("buffer_stepper",))
        _make_buffer(printer, "buffer t0", extruder="extruder")
        _fire_ready(printer)
        entry = printer.gcode.mux_commands["BUFFER_STATUS"]
        _, values = entry
        assert "t0" in values
        assert None not in values

    def test_dispatch_with_buffer_param_routes_to_named(self):
        printer = _make_printer_with_extruders(
            stepper_names=("buffer_stepper", "stepper_t1"))
        buf_bare = _make_buffer(printer, "buffer")
        buf_t1 = _make_buffer(printer, "buffer t1",
                              stepper_name="stepper_t1",
                              extruder="extruder1")
        _fire_ready(printer)

        gcmd = MockGcmd("BUFFER_STATUS", params={"BUFFER": "t1"})
        printer.gcode.dispatch_mux("BUFFER_STATUS", gcmd)
        # The message produced by cmd_BUFFER_STATUS on buf_t1 should
        # include its name in the header.
        assert any("Buffer status [t1]" in r for r in gcmd.responses)

    def test_dispatch_without_param_hits_bare_default(self):
        printer = _make_printer_with_extruders(
            stepper_names=("buffer_stepper", "stepper_t1"))
        buf_bare = _make_buffer(printer, "buffer")
        buf_t1 = _make_buffer(printer, "buffer t1",
                              stepper_name="stepper_t1",
                              extruder="extruder1")
        _fire_ready(printer)

        gcmd = MockGcmd("BUFFER_STATUS")
        printer.gcode.dispatch_mux("BUFFER_STATUS", gcmd)
        assert any("Buffer status [buffer]" in r for r in gcmd.responses)

    def test_dispatch_without_param_errors_when_no_default(self):
        printer = _make_printer_with_extruders(
            stepper_names=("buffer_stepper", "stepper_t1"))
        _make_buffer(printer, "buffer t0", extruder="extruder")
        _make_buffer(printer, "buffer t1", stepper_name="stepper_t1",
                     extruder="extruder1")
        _fire_ready(printer)

        gcmd = MockGcmd("BUFFER_STATUS")
        with pytest.raises(Exception):
            printer.gcode.dispatch_mux("BUFFER_STATUS", gcmd)

    def test_dispatch_unknown_name_errors(self, buf, printer):
        gcmd = MockGcmd("BUFFER_STATUS", params={"BUFFER": "nope"})
        with pytest.raises(Exception):
            printer.gcode.dispatch_mux("BUFFER_STATUS", gcmd)


# ---------------------------------------------------------------------------
# Extruder-bound sync
# ---------------------------------------------------------------------------


class TestBoundSync:
    def test_bound_buffer_syncs_when_its_extruder_is_active(self):
        printer = _make_printer_with_extruders()
        buf = _make_buffer(printer, "buffer t0", extruder="extruder")
        _fire_ready(printer)
        # Toolhead's default extruder is "extruder" which matches.
        buf.auto_enabled = True
        buf.state = STATE_STOPPED
        buf._sync()
        assert buf._synced_to == "extruder"
        assert buf.state == STATE_FEEDING

    def test_bound_buffer_does_not_sync_for_wrong_extruder(self):
        printer = _make_printer_with_extruders()
        buf = _make_buffer(printer, "buffer t1", extruder="extruder1")
        _fire_ready(printer)
        # Active is "extruder", but this buffer is bound to "extruder1".
        buf.auto_enabled = True
        buf.state = STATE_STOPPED
        buf._sync()
        assert buf._synced_to is None
        # State remains STOPPED because the sync was skipped.
        assert buf.state == STATE_STOPPED


# ---------------------------------------------------------------------------
# Tool-change polling via control timer
# ---------------------------------------------------------------------------


class TestToolChangePolling:
    def test_bound_buffer_syncs_on_tool_change_to_its_extruder(self):
        printer = _make_printer_with_extruders()
        # Start with "extruder" active; buffer is bound to "extruder1".
        buf = _make_buffer(printer, "buffer t1", extruder="extruder1")
        _fire_ready(printer)
        buf.auto_enabled = True
        buf.state = STATE_STOPPED
        # Initial sync attempt is skipped (wrong extruder).
        buf._sync()
        assert buf._synced_to is None

        # Simulate tool change: switch active extruder to "extruder1".
        printer.toolhead.set_extruder(MockExtruder(name="extruder1"))
        # Fire the control timer to trigger polling.
        now = printer.reactor.monotonic() + 2.0
        printer.reactor._monotonic = now
        printer.reactor._fire_timers()

        assert buf._synced_to == "extruder1"
        assert buf.state == STATE_FEEDING

    def test_bound_buffer_unsyncs_when_its_extruder_deactivates(self):
        printer = _make_printer_with_extruders()
        buf = _make_buffer(printer, "buffer t0", extruder="extruder")
        _fire_ready(printer)
        buf.auto_enabled = True
        buf.state = STATE_STOPPED
        buf._sync()
        assert buf._synced_to == "extruder"

        # Switch away to "extruder1".
        printer.toolhead.set_extruder(MockExtruder(name="extruder1"))
        now = printer.reactor.monotonic() + 2.0
        printer.reactor._monotonic = now
        printer.reactor._fire_timers()

        assert buf._synced_to is None
        assert buf.state == STATE_STOPPED

    def test_unbound_buffer_resyncs_to_new_extruder(self, buf, printer):
        # Default buf has no extruder binding.
        buf.auto_enabled = True
        buf.state = STATE_STOPPED
        buf._sync()
        assert buf._synced_to == "extruder"

        # Switch active extruder.
        printer.toolhead.set_extruder(MockExtruder(name="extruder1"))
        now = printer.reactor.monotonic() + 2.0
        printer.reactor._monotonic = now
        printer.reactor._fire_timers()

        # Unbound buffer should follow the new extruder.
        assert buf._synced_to == "extruder1"

    def test_handle_extruder_change_noop_when_disabled(self, buf, printer):
        # Buffer starts in IDLE without auto_enabled.
        assert not buf.auto_enabled
        printer.toolhead.set_extruder(MockExtruder(name="extruder1"))
        now = printer.reactor.monotonic() + 2.0
        printer.reactor._monotonic = now
        printer.reactor._fire_timers()
        # Disabled/not-auto-enabled buffer should not have synced.
        assert buf._synced_to is None


# ---------------------------------------------------------------------------
# Multiple independent buffers
# ---------------------------------------------------------------------------


class TestMultipleBuffers:
    def test_two_bound_buffers_track_independently(self):
        printer = _make_printer_with_extruders(
            stepper_names=("buffer_stepper", "stepper_t1"))
        buf0 = _make_buffer(printer, "buffer t0", extruder="extruder")
        buf1 = _make_buffer(printer, "buffer t1",
                            stepper_name="stepper_t1",
                            extruder="extruder1")
        _fire_ready(printer)

        buf0.auto_enabled = True
        buf0.state = STATE_STOPPED
        buf1.auto_enabled = True
        buf1.state = STATE_STOPPED

        # Active is "extruder": buf0 syncs, buf1 skips.
        buf0._sync()
        buf1._sync()
        assert buf0._synced_to == "extruder"
        assert buf1._synced_to is None

        # Tool change: now "extruder1" is active.
        printer.toolhead.set_extruder(MockExtruder(name="extruder1"))
        now = printer.reactor.monotonic() + 2.0
        printer.reactor._monotonic = now
        printer.reactor._fire_timers()

        # buf0 should have unsynced; buf1 should now be synced.
        assert buf0._synced_to is None
        assert buf1._synced_to == "extruder1"

    def test_multiple_unbound_logs_warning(self, caplog):
        printer = _make_printer_with_extruders(
            stepper_names=("buffer_stepper", "stepper_t1"))
        _make_buffer(printer, "buffer")
        _make_buffer(printer, "buffer spare",
                     stepper_name="stepper_t1")
        with caplog.at_level(logging.WARNING):
            _fire_ready(printer)
        # The second buffer's _handle_ready should log about multiple
        # unbound buffers.
        assert any("multiple buffers without 'extruder'" in r.message
                   for r in caplog.records)
