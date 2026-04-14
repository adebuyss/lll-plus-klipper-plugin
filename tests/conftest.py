"""Mock Klipper interfaces and pytest fixtures for buffer plugin testing.

Adapted for the AFC-style trapq sync architecture. Pure mocks, no Klipper
dependency.
"""

import sys
import pytest

# Ensure klipper/ is importable
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent / "klipper"))

import buffer as buffer_module

# Re-export constants for convenience in tests
FORWARD = buffer_module.FORWARD
BACK = buffer_module.BACK
STOP = buffer_module.STOP
STATE_DISABLED = buffer_module.STATE_DISABLED
STATE_IDLE = buffer_module.STATE_IDLE
STATE_FEEDING = buffer_module.STATE_FEEDING
STATE_STOPPED = buffer_module.STATE_STOPPED
STATE_RETRACTING = buffer_module.STATE_RETRACTING
STATE_ERROR = buffer_module.STATE_ERROR
STATE_MANUAL_FEED = buffer_module.STATE_MANUAL_FEED
STATE_MANUAL_RETRACT = buffer_module.STATE_MANUAL_RETRACT
ZONE_EMPTY = buffer_module.ZONE_EMPTY
ZONE_FULL = buffer_module.ZONE_FULL
ZONE_FULL_MIDDLE = buffer_module.ZONE_FULL_MIDDLE
ZONE_MIDDLE = buffer_module.ZONE_MIDDLE
ZONE_EMPTY_MIDDLE = buffer_module.ZONE_EMPTY_MIDDLE


# ---------------------------------------------------------------------------
# Sentinel for missing default arguments
# ---------------------------------------------------------------------------
_SENTINEL = object()


# ---------------------------------------------------------------------------
# Mock classes
# ---------------------------------------------------------------------------


class MockReactor:
    """Controllable time source. Tests set _monotonic directly or via
    advance_time() which also fires due timers."""

    NEVER = float("inf")

    def __init__(self):
        self._monotonic = 0.0
        self._timers = []  # [(callback, waketime)]
        self._pending_callbacks = []  # deferred callbacks

    def monotonic(self):
        return self._monotonic

    def advance_time(self, seconds):
        self._monotonic += seconds
        self._fire_callbacks()
        self._fire_timers()

    def _fire_callbacks(self):
        """Fire all pending deferred callbacks in FIFO order."""
        while self._pending_callbacks:
            cb = self._pending_callbacks.pop(0)
            cb(self._monotonic)

    def _fire_timers(self):
        for i, (cb, wake) in enumerate(list(self._timers)):
            if self._monotonic >= wake:
                next_wake = cb(self._monotonic)
                if next_wake is not None:
                    self._timers[i] = (cb, next_wake)

    def register_callback(self, callback):
        self._pending_callbacks.append(callback)

    def flush_callbacks(self):
        self._fire_callbacks()

    def register_timer(self, callback, waketime):
        handle = len(self._timers)
        self._timers.append((callback, waketime))
        return handle

    def update_timer(self, handle, waketime):
        cb, _ = self._timers[handle]
        self._timers[handle] = (cb, waketime)


class MockGcode:
    def __init__(self):
        self.commands = {}
        self.responses = []
        self.ready_gcode_handlers = {}
        self.scripts_run = []
        # mux_commands[cmd] = (key, {value: handler})
        self.mux_commands = {}

    def register_command(self, name, handler, desc=""):
        self.commands[name] = handler
        self.ready_gcode_handlers[name] = handler

    def register_mux_command(self, cmd, key, value, handler, desc=""):
        """Mimic Klipper's mux command dispatch.

        Tests can dispatch with dispatch_mux(cmd, gcmd) which reads the
        key param from gcmd and routes to the right handler.  For
        backward-compat with tests that call mock.commands[name] directly,
        the default (value=None) handler is also exposed as commands[name].
        """
        entry = self.mux_commands.setdefault(cmd, (key, {}))
        _, values = entry
        values[value] = handler
        self.ready_gcode_handlers[cmd] = handler
        # Expose a plain-command alias for simple tests.  If a value=None
        # handler is registered, that's the default; otherwise expose the
        # most recently registered handler so existing single-buffer
        # tests that look up commands[name] still work.
        if None in values:
            self.commands[cmd] = values[None]
        elif cmd not in self.commands:
            self.commands[cmd] = handler

    def dispatch_mux(self, cmd, gcmd):
        """Simulate Klipper's mux dispatch: read the key param and route.

        Matches Klipper's _cmd_mux_wrapper semantics:
        - If None is registered, the key param is optional (missing -> None)
        - If None is not registered, the key param is required
        - If the param is provided but not registered, raise Unknown
        """
        entry = self.mux_commands.get(cmd)
        if entry is None:
            raise Exception("No mux command registered: %s" % cmd)
        key, values = entry
        if None in values:
            key_val = gcmd._params.get(key, None)
        else:
            if key not in gcmd._params:
                raise Exception("Missing required param: %s" % key)
            key_val = gcmd._params[key]
        if key_val not in values:
            raise Exception("Unknown %s: %s" % (key, key_val))
        return values[key_val](gcmd)

    def respond_info(self, msg):
        self.responses.append(msg)

    def run_script(self, script):
        self.scripts_run.append(script)


class MockGcmd:
    """Simulates a gcode command with parameters."""

    def __init__(self, command="G1", params=None):
        self._command = command
        self._params = params or {}
        self.responses = []

    def get_command(self):
        return self._command

    def get_float(self, name, default=_SENTINEL, above=None, minval=None):
        val = self._params.get(name, default)
        if val is _SENTINEL:
            raise self._error("Missing required param: %s" % name)
        return float(val)

    def get(self, name, default=_SENTINEL):
        val = self._params.get(name, default)
        if val is _SENTINEL:
            raise self._error("Missing required param: %s" % name)
        return val

    def respond_info(self, msg):
        self.responses.append(msg)

    @staticmethod
    def _error(msg):
        return Exception(msg)


class MockConfig:
    """Returns typed config values from a dictionary."""

    def __init__(self, printer, values=None):
        self._printer = printer
        self._values = values or {}

    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._values.get("_name", "buffer")

    def get(self, key, default=_SENTINEL):
        if key in self._values:
            return self._values[key]
        if default is not _SENTINEL:
            return default
        raise KeyError(key)

    def getfloat(self, key, default=_SENTINEL, minval=None, above=None,
                 below=None, maxval=None):
        if key in self._values:
            return float(self._values[key])
        if default is not _SENTINEL:
            return float(default)
        raise KeyError(key)

    def getboolean(self, key, default=_SENTINEL):
        if key in self._values:
            v = self._values[key]
            if isinstance(v, bool):
                return v
            return str(v).lower() in ("true", "1", "yes")
        if default is not _SENTINEL:
            return default
        raise KeyError(key)


class MockStepper:
    """Mock for PrinterStepper — tracks rotation_distance changes."""

    def __init__(self, rotation_distance=19.2357):
        self._rotation_distance = rotation_distance
        self.rd_log = []  # track all set_rotation_distance calls

    def get_rotation_distance(self):
        return (self._rotation_distance, False)

    def set_rotation_distance(self, rd):
        self._rotation_distance = rd
        self.rd_log.append(rd)


class MockExtruderStepper:
    """Mock for kinematics.extruder.ExtruderStepper."""

    def __init__(self):
        self.stepper = MockStepper()
        self.motion_queue = None
        self._synced_to = None

    def sync_to_extruder(self, extruder_name):
        self._synced_to = extruder_name
        self.motion_queue = extruder_name

    def get_status(self, eventtime):
        return {"motion_queue": self.motion_queue}


class MockPrinterExtruderStepper:
    """Mock for extras.extruder_stepper.PrinterExtruderStepper."""

    def __init__(self):
        self.extruder_stepper = MockExtruderStepper()

    def get_status(self, eventtime):
        return self.extruder_stepper.get_status(eventtime)


class MockExtruder:
    """Mock for kinematics.extruder.PrinterExtruder."""

    def __init__(self, name="extruder"):
        self._name = name

    def get_name(self):
        return self._name


class MockToolhead:
    """Mock for toolhead.ToolHead."""

    def __init__(self):
        self._extruder = MockExtruder()
        self.flush_count = 0

    def get_extruder(self):
        return self._extruder

    def set_extruder(self, extruder):
        """Test helper: swap the active extruder."""
        self._extruder = extruder

    def flush_step_generation(self):
        self.flush_count += 1

    def get_last_move_time(self):
        return 0.0

    def dwell(self, delay):
        pass


class MockForceMove:
    """Mock for extras.force_move.ForceMove."""

    def __init__(self):
        self.moves = []  # [(stepper, dist, speed, accel)]

    def manual_move(self, stepper, dist, speed, accel=0.0):
        self.moves.append((stepper, dist, speed, accel))


class MockButtons:
    """Captures registered callbacks so tests can fire sensor events."""

    def __init__(self):
        self.callbacks = {}  # pin_name -> callback

    def register_buttons(self, pins, callback):
        for pin in pins:
            self.callbacks[pin] = callback


class MockPrintStats:
    def __init__(self):
        self.state = "standby"

    def get_status(self, eventtime):
        return {"state": self.state}


class MockPauseResume:
    def __init__(self):
        self.is_paused = False


class MockPrinter:
    """Wires all mock objects together, matching Klipper's lookup_object."""

    def __init__(self):
        self.reactor = MockReactor()
        self.gcode = MockGcode()
        self.buttons = MockButtons()
        self.print_stats = MockPrintStats()
        self.pause_resume = MockPauseResume()
        self.toolhead = MockToolhead()
        self.printer_es = MockPrinterExtruderStepper()
        self.force_move = MockForceMove()
        self.event_handlers = {}

        self._objects = {
            "gcode": self.gcode,
            "toolhead": self.toolhead,
            "extruder_stepper buffer_stepper": self.printer_es,
            "force_move": self.force_move,
            "print_stats": self.print_stats,
            "pause_resume": self.pause_resume,
        }

    def lookup_object(self, name, default=_SENTINEL):
        if name in self._objects:
            return self._objects[name]
        if default is not _SENTINEL:
            return default
        raise Exception("Unknown object: %s" % name)

    def lookup_objects(self, module=None):
        """Return [(name, obj)] for all registered objects optionally
        filtered to those whose config section starts with `module`."""
        if module is None:
            return list(self._objects.items())
        prefix = module
        return [(name, obj) for name, obj in self._objects.items()
                if name == prefix or name.startswith(prefix + " ")]

    def add_object(self, name, obj):
        """Test helper: register an object for lookup_object/lookup_objects."""
        self._objects[name] = obj

    def load_object(self, config, name):
        if name == "buttons":
            return self.buttons
        return self._objects.get(name)

    def get_reactor(self):
        return self.reactor

    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)


# ---------------------------------------------------------------------------
# Default config values matching buffer.py expectations
# ---------------------------------------------------------------------------
DEFAULT_CONFIG = {
    "_name": "buffer",
    "stepper": "buffer_stepper",
    "sensor_empty_pin": "PE0",
    "sensor_middle_pin": "PE1",
    "sensor_full_pin": "PE2",
    "material_switch_pin": "PE3",
    "feed_button_pin": "PE4",
    "retract_button_pin": "PE5",
}


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def printer():
    return MockPrinter()


@pytest.fixture
def config(printer):
    return MockConfig(printer, dict(DEFAULT_CONFIG))


@pytest.fixture
def buf(config, printer):
    """Construct Buffer and fire klippy:ready."""
    b = buffer_module.Buffer(config)
    # Fire ready
    for handler in printer.event_handlers.get("klippy:ready", []):
        handler()
    return b


@pytest.fixture
def enabled_buf(buf, printer):
    """Buffer that is auto-enabled with material present and synced."""
    buf.material_present = True
    buf.auto_enabled = True
    buf.state = STATE_STOPPED
    buf._sync()
    return buf


@pytest.fixture
def buttons(printer):
    return printer.buttons


@pytest.fixture
def reactor(printer):
    return printer.reactor


@pytest.fixture
def gcode(printer):
    return printer.gcode


@pytest.fixture
def force_move(printer):
    return printer.force_move


@pytest.fixture
def stepper(printer):
    """The mock PrinterStepper underlying the buffer's extruder_stepper."""
    return printer.printer_es.extruder_stepper.stepper


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------


def set_sensors(b, empty=False, middle=False, full=False):
    """Directly set sensor states without triggering callbacks."""
    b.sensor_states["empty"] = empty
    b.sensor_states["middle"] = middle
    b.sensor_states["full"] = full


def trigger_sensor(buttons, pin, triggered, eventtime):
    """Fire a sensor callback as the buttons module would.
    triggered=True means sensor is blocked (buttons passes state=0 for
    inverted pins)."""
    cb = buttons.callbacks[pin]
    cb(eventtime, 0 if triggered else 1)
