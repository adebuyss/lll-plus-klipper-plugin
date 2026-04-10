"""Mock Klipper interfaces and pytest fixtures for buffer plugin testing.

Inspired by Kalico's PrinterShim pattern — pure mocks, no Klipper dependency.
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
        """Fire all pending deferred callbacks (registered via
        register_callback) in FIFO order."""
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
        """Queue a callback to fire on the next reactor iteration
        (simulated by advance_time or flush_callbacks)."""
        self._pending_callbacks.append(callback)

    def flush_callbacks(self):
        """Fire pending callbacks without advancing time."""
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

    def register_command(self, name, handler, desc=""):
        self.commands[name] = handler
        self.ready_gcode_handlers[name] = handler

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

    def getfloat(self, key, default=_SENTINEL, minval=None, above=None, below=None, maxval=None):
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


class MockTmcFields:
    def __init__(self, mcu_tmc):
        self._mcu_tmc = mcu_tmc

    def set_field(self, field_name, value):
        reg = self._mcu_tmc.registers.get("GCONF", 0)
        if field_name == "shaft":
            reg = (reg & ~(1 << 3)) | (value << 3)
        self._mcu_tmc.registers["GCONF"] = reg
        return reg


class MockMcuTmc:
    def __init__(self):
        # CHOPCONF with mres=4 (64 microsteps): 0x04 in bits 27-24
        self.registers = {
            "CHOPCONF": 0x04000000,
            "GCONF": 0,
            "VACTUAL": 0,
        }
        self.write_log = []
        self.fields = MockTmcFields(self)
        self.fail_writes = False

    def get_register(self, name):
        return self.registers.get(name, 0)

    def set_register(self, name, value):
        if self.fail_writes:
            raise Exception("simulated write failure")
        self.registers[name] = value
        self.write_log.append((name, value))


class MockTmcObj:
    """Wraps MockMcuTmc to match printer.lookup_object('tmc2208 ...') result."""

    def __init__(self, mcu_tmc):
        self.mcu_tmc = mcu_tmc


class MockStepper:
    def get_rotation_distance(self):
        return (23.2, False)


class MockManualStepper:
    def __init__(self):
        self.enabled = False
        self.steppers = [MockStepper()]

    def do_enable(self, enable):
        self.enabled = enable


class MockButtons:
    """Captures registered callbacks so tests can fire sensor events."""

    def __init__(self):
        self.callbacks = {}  # pin_name -> callback

    def register_buttons(self, pins, callback):
        for pin in pins:
            self.callbacks[pin] = callback


class MockGcodeMove:
    def __init__(self):
        self.last_position = [0.0, 0.0, 0.0, 0.0]
        self.speed = 50.0


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
        self.gcode_move = MockGcodeMove()
        self.print_stats = MockPrintStats()
        self.pause_resume = MockPauseResume()
        self.manual_stepper = MockManualStepper()
        self.mcu_tmc = MockMcuTmc()
        self.tmc_obj = MockTmcObj(self.mcu_tmc)
        self.event_handlers = {}

        stepper_key = "manual_stepper buffer_stepper"
        tmc_key = "tmc2208 %s" % stepper_key

        self._objects = {
            "gcode": self.gcode,
            stepper_key: self.manual_stepper,
            tmc_key: self.tmc_obj,
            "gcode_move": self.gcode_move,
            "print_stats": self.print_stats,
            "pause_resume": self.pause_resume,
        }

    def lookup_object(self, name):
        if name not in self._objects:
            raise Exception("Unknown object: %s" % name)
        return self._objects[name]

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
    "speed_rpm": 260.0,
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
    """Buffer that is auto-enabled with material present."""
    buf.material_present = True
    buf.auto_enabled = True
    buf.state = STATE_STOPPED
    return buf


@pytest.fixture
def buttons(printer):
    return printer.buttons


@pytest.fixture
def reactor(printer):
    return printer.reactor


@pytest.fixture
def mcu_tmc(printer):
    return printer.mcu_tmc


@pytest.fixture
def gcode(printer):
    return printer.gcode


@pytest.fixture
def gcode_move(printer):
    return printer.gcode_move


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
    # buttons state: 0 = triggered (inverted), 1 = not triggered
    cb(eventtime, 0 if triggered else 1)


def simulate_e_move(b, e_delta, xyz_dist=0.0, speed=50.0):
    """Simulate a G1 move with E component by calling _on_e_movement.
    Flushes deferred drive callbacks so motor writes happen immediately
    in tests (mirrors reactor iteration after gcode handler returns)."""
    gm = b._gcode_move
    prev_pos = list(gm.last_position)
    gm.speed = speed
    gm.last_position[3] += e_delta
    if xyz_dist > 0:
        gm.last_position[0] += xyz_dist
    b._on_e_movement(e_delta, prev_pos)
    b.reactor.flush_callbacks()
