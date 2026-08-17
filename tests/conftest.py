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
        # Index of the timer whose callback is currently executing.
        # Mirrors production Klipper's ReactorTimer.timer_is_running:
        # update_timer is a no-op for the firing timer.  Without this
        # guard, the "update_timer(...) + return NEVER" anti-pattern
        # would appear to work in tests but silently disarm the timer
        # on production hardware after exactly one fire.
        self._firing_timer_idx = None

    def monotonic(self):
        return self._monotonic

    def advance_time(self, seconds):
        self._monotonic += seconds
        self._fire_callbacks()
        self._fire_timers()

    def _fire_callbacks(self):
        """Fire pending deferred callbacks in FIFO order.  Snapshot
        the list at entry so callbacks that re-register themselves
        (the chained-chunk pattern used by manual feed/retract/
        initial fill) schedule for the NEXT advance_time call, not
        recursively within this one — matching production reactor
        semantics where register_callback fires on the next reactor
        iteration."""
        pending = self._pending_callbacks
        self._pending_callbacks = []
        for cb in pending:
            cb(self._monotonic)

    def _fire_timers(self):
        for i, (cb, wake) in enumerate(list(self._timers)):
            if self._monotonic >= wake:
                # Mirror production Klipper: update_timer is a no-op
                # while the timer's callback runs (timer_is_running
                # guard), and the callback's return value is what
                # rearms the timer.  See klippy/reactor.py upstream.
                self._firing_timer_idx = i
                try:
                    next_wake = cb(self._monotonic)
                finally:
                    self._firing_timer_idx = None
                if next_wake is not None:
                    self._timers[i] = (cb, next_wake)

    def flush_callbacks(self):
        """Fire pending register_callback callbacks AND any timers
        whose wake is at or before the current monotonic.  Mirrors how
        production drains the reactor between scheduled tasks."""
        self._fire_callbacks()
        self._fire_timers()

    def register_callback(self, callback, waketime=None):
        # Production Klipper's register_callback takes an optional
        # waketime; the mock ignores it (callbacks always fire on
        # the next advance_time / flush_callbacks).  Tests that need
        # to verify timing should use register_timer instead.
        self._pending_callbacks.append(callback)

    def register_timer(self, callback, waketime):
        handle = len(self._timers)
        self._timers.append((callback, waketime))
        return handle

    def update_timer(self, handle, waketime):
        # Production semantics: a timer's update_timer is a no-op
        # while that timer's callback is running.  Catches the
        # "update_timer(...) + return NEVER" pattern at test time.
        if handle == self._firing_timer_idx:
            return
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
    """Mock for PrinterStepper — tracks rotation_distance changes.

    Defaults match the LLL Plus reference config: 200 motor full
    steps/rev × 16 microsteps × 50/17 gear_ratio / 19.2357 mm
    rotation_distance ≈ 489.7 microsteps/mm ⇒ step_dist ≈ 0.002042 mm.
    Used by the VACTUAL formula tests to assert register_value
    = mm/s × ~489.7 / 0.715.
    """

    def __init__(self, rotation_distance=19.2357,
                 step_dist=1.0 / 489.71524, mcu=None):
        self._rotation_distance = rotation_distance
        self._step_dist = step_dist
        self.rd_log = []  # track all set_rotation_distance calls
        self._mcu = mcu

    def get_rotation_distance(self):
        return (self._rotation_distance, False)

    def set_rotation_distance(self, rd):
        self._rotation_distance = rd
        self.rd_log.append(rd)

    def get_step_dist(self):
        return self._step_dist

    def get_mcu(self):
        return self._mcu


class MockExtruderStepper:
    """Mock for kinematics.extruder.ExtruderStepper.

    Real Klipper's ExtruderStepper.sync_to_extruder() calls
    toolhead.flush_step_generation() before rebinding the trapq.
    The mock mirrors that so buffer.py's flushing invariants are
    exercised faithfully by tests."""

    def __init__(self, toolhead=None):
        mcu = toolhead.get_mcu() if toolhead is not None else None
        self.stepper = MockStepper(mcu=mcu)
        self.motion_queue = None
        self._synced_to = None
        self._toolhead = toolhead

    def sync_to_extruder(self, extruder_name):
        if self._toolhead is not None:
            self._toolhead.flush_step_generation()
        self._synced_to = extruder_name
        self.motion_queue = extruder_name

    def get_status(self, eventtime):
        return {"motion_queue": self.motion_queue}


class MockPrinterExtruderStepper:
    """Mock for extras.extruder_stepper.PrinterExtruderStepper."""

    def __init__(self, toolhead=None):
        self.extruder_stepper = MockExtruderStepper(toolhead=toolhead)

    def get_status(self, eventtime):
        return self.extruder_stepper.get_status(eventtime)


class MockExtruder:
    """Mock for kinematics.extruder.PrinterExtruder.

    The buffer's _estimated_extruder_rate samples
    extruder.extruder_stepper.stepper.get_commanded_position() across
    eventtime samples to derive a signed mm/s rate. Tests can drive
    this either by setting `_position` directly between sensor ticks
    or by calling `set_rate(mm_per_s, t0)` to have the position
    auto-advance with eventtime queries.
    """

    def __init__(self, name="extruder"):
        self._name = name
        self.extruder_stepper = _MockMainExtruderStepper()

    def get_name(self):
        return self._name

    def set_rate(self, mm_per_s, t0=0.0):
        """Configure auto-advancing position: pos(t) = mm_per_s * (t - t0)."""
        self.extruder_stepper.stepper.set_rate(mm_per_s, t0)


class _MockMainExtruderStepper:
    """Stepper-host wrapper so extruder.extruder_stepper.stepper resolves."""

    def __init__(self):
        self.stepper = _MockExtrusionStepper()


class _MockExtrusionStepper:
    """Position source for the active extruder. Either a static value the
    test sets via _position, or an auto-advancing value driven by set_rate."""

    def __init__(self):
        self._position = 0.0
        self._rate = None  # (mm_per_s, t0) when auto-advancing
        self._eventtime_provider = None

    def set_rate(self, mm_per_s, t0=0.0):
        self._rate = (mm_per_s, t0)

    def get_commanded_position(self):
        if self._rate is None or self._eventtime_provider is None:
            return self._position
        eventtime = self._eventtime_provider()
        rate, t0 = self._rate
        return rate * (eventtime - t0)


class MockMcu:
    """Minimal MCU mock — estimated_print_time only (matches Klipper's
    MCU.estimated_print_time signature)."""

    def __init__(self, reactor=None):
        self._reactor = reactor

    def estimated_print_time(self, eventtime):
        return eventtime


class MockTmc2208:
    """Mock for the [tmc2208 extruder_stepper buffer_stepper] object.

    Exposes a single `mcu_tmc` attribute whose set_register() captures
    every call into `writes` for assertion.  The latency-regression
    guard asserts that print_time is always None — i.e. that the
    plugin never schedules VACTUAL writes at the lookahead tail.
    """

    def __init__(self):
        self.mcu_tmc = MockMcuTmc()


class MockMcuTmc:
    """Backing for MockTmc2208.mcu_tmc.  set_register signature
    matches Klipper's: (reg_name, value, print_time=None)."""

    def __init__(self):
        # All writes: [(reg_name, value, print_time), ...]
        self.writes = []
        # Convenience: VACTUAL writes only, just the integer value.
        self.vactual_writes = []
        # Failure injection.  Add a register name (e.g. "VACTUAL") to
        # make set_register raise for it, modelling Klipper's
        # MCU_TMC_uart.set_register raising command_error once its
        # IFCNT read-back retries are exhausted.
        self.fail_registers = set()

    def set_register(self, reg_name, value, print_time=None):
        # Record the attempt even when failing.  This is faithful to
        # the hardware: mcu_uart.reg_write is fire-and-forget, so the
        # frame may well have reached the chip — set_register raises
        # only because the IFCNT read-back could not confirm it.  That
        # ambiguity is precisely what _vactual_maybe_running exists
        # to model, so the mock must not pretend the write never left.
        self.writes.append((reg_name, value, print_time))
        if reg_name == "VACTUAL":
            self.vactual_writes.append(value)
        if reg_name in self.fail_registers:
            raise Exception(
                "Unable to write tmc uart 'buffer_stepper' register %s"
                % reg_name)


class MockToolhead:
    """Mock for toolhead.ToolHead."""

    def __init__(self, reactor=None):
        self._reactor = reactor
        self._mcu = MockMcu(reactor=reactor)
        self._extruder = MockExtruder()
        self._wire_extruder(self._extruder)
        self.flush_count = 0
        self.dwell_calls = []
        self._last_move_time = 0.0

    def _wire_extruder(self, extruder):
        if self._reactor is not None:
            extruder.extruder_stepper.stepper._eventtime_provider = (
                self._reactor.monotonic)

    def get_extruder(self):
        return self._extruder

    def set_extruder(self, extruder):
        """Test helper: swap the active extruder."""
        self._extruder = extruder
        self._wire_extruder(extruder)

    def get_mcu(self):
        return self._mcu

    def flush_step_generation(self):
        self.flush_count += 1

    def get_last_move_time(self):
        return self._last_move_time

    def get_status(self, eventtime):
        """Non-flushing status snapshot, mirroring production
        toolhead.get_status — the heartbeat must use this, never
        get_last_move_time (which flushes the lookahead)."""
        return {"print_time": self._last_move_time,
                "estimated_print_time": eventtime}

    def dwell(self, delay):
        self.dwell_calls.append(delay)
        self._last_move_time += max(0.0, delay)


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
        self.toolhead = MockToolhead(reactor=self.reactor)
        self.printer_es = MockPrinterExtruderStepper(toolhead=self.toolhead)
        self.force_move = MockForceMove()
        # MockTmc2208 captures direct VACTUAL register writes so
        # recovery tests can assert against them.
        self.tmc2208 = MockTmc2208()
        self.event_handlers = {}

        self._objects = {
            "gcode": self.gcode,
            "toolhead": self.toolhead,
            "extruder_stepper buffer_stepper": self.printer_es,
            "tmc2208 extruder_stepper buffer_stepper": self.tmc2208,
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

    def send_event(self, event, *params):
        """Mirror production Printer.send_event: run handlers in
        registration order, return their results."""
        return [cb(*params)
                for cb in self.event_handlers.get(event, [])]


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
    # _handle_ready runs the boot-time VACTUAL hygiene sequence (the
    # chip state is unknown at boot: zero + nudge + deferred zero).
    # Drain the deferred clear and drop the captured writes so tests
    # assert only the writes they themselves provoke.  Dedicated
    # boot-hygiene tests construct their own Buffer instead of using
    # this fixture.
    printer.reactor.flush_callbacks()
    printer.tmc2208.mcu_tmc.writes.clear()
    printer.tmc2208.mcu_tmc.vactual_writes.clear()
    # Simulate the initial pin-state report from Klipper's buttons module
    # (material switch = no filament).  This consumes the _initial_state_received
    # guard so subsequent test callbacks are treated as real events.
    b._initial_state_received = True
    # Tests use set_sensors() which bypasses callbacks; pretend at least
    # one sensor callback has fired so _update_rotation_distance runs.
    b._any_sensor_reported = True
    # Reset sensor_states to all-False — the "inactive" baseline most
    # tests expect.  In production, inactive sensors flip to False via
    # their initial callbacks; active sensors stay True.  Tests that care
    # about sensor state call set_sensors() explicitly.
    b.sensor_states = {"empty": False, "middle": False, "full": False}
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
def printing_buf(enabled_buf):
    """enabled_buf with _print_stats.state flipped to 'printing'.

    Recovery entry is gated on _is_printing() to avoid surprise motion
    outside a print (user loading/unloading filament).  Tests that
    exercise the recovery path use this fixture instead of plain
    enabled_buf.
    """
    enabled_buf._print_stats.state = "printing"
    return enabled_buf


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
def sidecar_moves(force_move):
    """Backward-compat alias: post-VACTUAL-refactor, all chunked
    motion is back on force_move.manual_move so this fixture aliases
    to force_move.moves.  Existing tests that look up sidecar_moves
    keep working without per-file rewrites."""
    return force_move.moves


@pytest.fixture
def vactual_writes(printer):
    """Returns the list of VACTUAL register values written via
    mcu_tmc.set_register('VACTUAL', N).  Recovery tests assert
    against this directly."""
    return printer.tmc2208.mcu_tmc.vactual_writes


@pytest.fixture
def tmc_writes(printer):
    """All TMC register writes including (reg_name, value, print_time).
    Used by the latency-regression guard to assert print_time=None."""
    return printer.tmc2208.mcu_tmc.writes


@pytest.fixture
def mcu_tmc(printer):
    """The mock mcu_tmc itself.  Tests use `mcu_tmc.fail_registers.add(
    "VACTUAL")` to simulate an unresponsive TMC UART."""
    return printer.tmc2208.mcu_tmc


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
