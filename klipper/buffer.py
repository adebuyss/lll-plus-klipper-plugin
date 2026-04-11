# Klipper extras module for Mellow LLL Plus filament buffer
#
# Synchronizes the buffer stepper with the main extruder via shared trapq
# (the AFC/Happy Hare pattern). Hall effect sensors provide reactive
# feedback by adjusting the buffer stepper's rotation_distance to keep
# the filament loop centered on the middle sensor.
#
# Copyright (C) 2026  adebuyss
# License: GPLv3

import logging

FORWARD = "forward"
BACK = "back"
STOP = "stop"

STATE_DISABLED = "disabled"
STATE_IDLE = "idle"
STATE_FEEDING = "feeding"
STATE_STOPPED = "stopped"
STATE_RETRACTING = "retracting"
STATE_ERROR = "error"
STATE_MANUAL_FEED = "manual_feed"
STATE_MANUAL_RETRACT = "manual_retract"

# Sensor zone identifiers (kept for diagnostics in BUFFER_STATUS)
ZONE_EMPTY = "empty"
ZONE_FULL = "full"
ZONE_FULL_MIDDLE = "full_middle"
ZONE_MIDDLE = "middle"
ZONE_EMPTY_MIDDLE = "empty_middle"

# State machine transitions:
#   DISABLED -> IDLE          on klippy:ready
#   IDLE -> STOPPED           on material insert (auto-enable) or BUFFER_ENABLE
#   IDLE -> DISABLED          on BUFFER_DISABLE
#   STOPPED -> FEEDING        on sync to extruder (auto-control active)
#   STOPPED -> RETRACTING     on full-zone forced retract
#   FEEDING -> STOPPED        on unsync / disable
#   RETRACTING -> STOPPED     on retract complete
#   * -> ERROR                on _handle_error (sensor conflict, safety timeout)
#   ERROR -> STOPPED/IDLE     on BUFFER_CLEAR_ERROR or 2s both-button hold
#   * -> MANUAL_FEED          on feed button press / BUFFER_FEED
#   * -> MANUAL_RETRACT       on retract button press / BUFFER_RETRACT
#   MANUAL_* -> STOPPED/IDLE  on button release (restores prior auto_enabled)
#   * (both buttons)          toggle auto_enabled (IDLE <-> STOPPED)
#   FEEDING/STOPPED/RETRACTING -> IDLE  on filament runout
#   * -> DISABLED             on BUFFER_DISABLE / klippy:shutdown
#
# Invariants:
# - ERROR blocks all gcode feed/retract; only BUFFER_CLEAR_ERROR or
#   2-second both-button hold exits it.
# - MANUAL_FEED/MANUAL_RETRACT bypass rotation_distance feedback.
# - DISABLED blocks material-insert auto-enable; only BUFFER_ENABLE exits.

# Deprecated config params — accepted with warning, values ignored
_DEPRECATED_PARAMS = [
    ("velocity_factor", 1.05, "above", 0.9),
    ("correction_factor", 1.3, "above", 1.0),
    ("slowdown_factor", 0.5, "above", 0.0),
    ("burst_feed_time", 0.5, "above", 0.0),
    ("burst_delay", 0.5, "minval", 0.0),
    ("velocity_window", 0.3, "above", 0.0),
    ("drive_interval", 0.1, "above", 0.02),
    ("full_zone_timeout", 3.0, "above", 0.0),
    ("full_zone_retract_length", 0.0, "minval", 0.0),
    ("min_extrusion_velocity", 0.05, "minval", 0.0),
]


class Buffer:
    """Klipper extras module for filament buffer control.

    Synchronizes the buffer stepper with the main extruder via a shared
    trapq (extruder_stepper). Hall sensors adjust rotation_distance to
    keep the filament loop centered on the middle sensor.
    """

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.name = config.get_name()

        # Config — new AFC-style parameters
        self.stepper_name = config.get("stepper")
        self.drift_gain = config.getfloat("drift_gain", 0.02, minval=0.0,
                                          maxval=0.5)
        self.safety_gain = config.getfloat("safety_gain", 0.05, minval=0.0,
                                           maxval=0.5)
        self.fault_escalation_time = config.getfloat("fault_escalation_time",
                                                     5.0, above=0.0)
        self.empty_safety_timeout = config.getfloat("empty_safety_timeout",
                                                    30.0, above=0.0)
        self.full_safety_timeout = config.getfloat("full_safety_timeout",
                                                   10.0, above=0.0)
        self.error_clear_hold_time = config.getfloat("error_clear_hold_time",
                                                     2.0, above=0.0)
        self.manual_speed = config.getfloat("manual_speed", 10.0, above=0.0)
        self.manual_accel = config.getfloat("manual_accel", 100.0, above=0.0)
        self.pause_on_runout = config.getboolean("pause_on_runout", True)
        self.debug = config.getboolean("debug", False)
        self.control_interval = config.getfloat("control_interval", 0.5,
                                                above=0.05)
        self.initial_fill_timeout = config.getfloat("initial_fill_timeout",
                                                    10.0, above=0.0)
        self.manual_feed_full_timeout = config.getfloat(
            "manual_feed_full_timeout", 5.0, above=0.0)
        self.forward_timeout = config.getfloat("forward_timeout", 60.0,
                                               minval=0.0)

        # Sensor pin names
        self.sensor_empty_pin = config.get("sensor_empty_pin")
        self.sensor_middle_pin = config.get("sensor_middle_pin")
        self.sensor_full_pin = config.get("sensor_full_pin")
        self.material_switch_pin = config.get("material_switch_pin")
        self.feed_button_pin = config.get("feed_button_pin", None)
        self.retract_button_pin = config.get("retract_button_pin", None)

        # Deprecated config — parse with warning, ignore values
        self._deprecated_warnings = []
        for name, default, constraint_type, constraint_val in _DEPRECATED_PARAMS:
            kwargs = {constraint_type: constraint_val}
            try:
                val = config.getfloat(name, default, **kwargs)
                if val != default:
                    self._deprecated_warnings.append(name)
            except Exception:
                pass

        # Extruder stepper (resolved at ready time)
        self.toolhead = None
        self.extruder_stepper = None  # ExtruderStepper from kinematics
        self.force_move = None
        self._base_rd = 0.0
        self._rd_multiplier = 1.0
        self._synced_to = None  # extruder name when synced, None when not

        # State
        self.state = STATE_DISABLED
        self.auto_enabled = False
        self.sensor_states = {"empty": False, "middle": False, "full": False}
        self.material_present = False
        self.error_msg = ""
        self.motor_direction = STOP
        self._current_zone = None
        self._prev_zone = None

        # Safety timeout tracking
        self._safety_zone_start = 0.0  # when we entered EMPTY or FULL
        self._safety_escalated = False
        self._forward_start_time = 0.0
        self._forward_elapsed = 0.0

        # Initial fill
        self._initial_fill_until = 0.0

        # Button state
        self._feed_button_pressed = False
        self._retract_button_pressed = False
        self._manual_feed_full_start = 0.0
        self._error_clear_hold_start = 0.0
        self._error_clear_timer = None

        # Manual move tracking
        self._manual_move_timer = None
        self._manual_chunk_dist = 20.0  # mm per button-held chunk

        # Print state tracking
        self._print_stats = None

        # Timer handles
        self._control_timer = None

        # Register events
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)

        # Register gcode commands
        self.gcode.register_command(
            "BUFFER_STATUS", self.cmd_BUFFER_STATUS,
            desc="Report buffer status")
        self.gcode.register_command(
            "BUFFER_ENABLE", self.cmd_BUFFER_ENABLE,
            desc="Enable automatic buffer control")
        self.gcode.register_command(
            "BUFFER_DISABLE", self.cmd_BUFFER_DISABLE,
            desc="Disable buffer control and stop motor")
        self.gcode.register_command(
            "BUFFER_FEED", self.cmd_BUFFER_FEED,
            desc="Manually feed filament forward")
        self.gcode.register_command(
            "BUFFER_RETRACT", self.cmd_BUFFER_RETRACT,
            desc="Manually retract filament")
        self.gcode.register_command(
            "BUFFER_STOP", self.cmd_BUFFER_STOP,
            desc="Stop motor and return to auto mode")
        self.gcode.register_command(
            "BUFFER_SET_SPEED", self.cmd_BUFFER_SET_SPEED,
            desc="Set manual feed/retract speed in mm/s")
        self.gcode.register_command(
            "BUFFER_CLEAR_ERROR", self.cmd_BUFFER_CLEAR_ERROR,
            desc="Clear buffer error state")

        # Register sensor pins via buttons module
        self._register_sensors(config)

    # --- Lifecycle ---

    def _handle_ready(self):
        # Look up extruder_stepper and force_move
        self.toolhead = self.printer.lookup_object("toolhead")
        es_key = "extruder_stepper %s" % self.stepper_name
        printer_es = self.printer.lookup_object(es_key)
        self.extruder_stepper = printer_es.extruder_stepper
        self.force_move = self.printer.lookup_object("force_move")
        # Capture baseline rotation_distance
        self._base_rd = (
            self.extruder_stepper.stepper.get_rotation_distance()[0])
        self._rd_multiplier = 1.0
        # Transition to IDLE
        self.state = STATE_IDLE
        # Look up print_stats
        try:
            self._print_stats = self.printer.lookup_object("print_stats")
        except Exception:
            logging.info("buffer: print_stats not available")
        # Emit deprecation warnings
        if self._deprecated_warnings:
            msg = ("Buffer: deprecated config params ignored: %s. "
                   "These will be removed in a future release."
                   % ", ".join(self._deprecated_warnings))
            logging.warning("buffer: %s" % msg)
            self.gcode.respond_info(msg)
        # Start control timer
        self._control_timer = self.reactor.register_timer(
            self._control_timer_cb, self.reactor.monotonic() + 1.0)
        logging.info("buffer: ready (base_rd=%.4f)" % self._base_rd)

    def _handle_shutdown(self):
        self._unsync()
        self.state = STATE_DISABLED
        self.auto_enabled = False
        logging.info("buffer: shutdown")

    # --- Sensor registration ---

    def _register_sensors(self, config):
        buttons = self.printer.load_object(config, "buttons")
        for sensor_name, pin_name in [
            ("empty", self.sensor_empty_pin),
            ("middle", self.sensor_middle_pin),
            ("full", self.sensor_full_pin),
        ]:
            buttons.register_buttons(
                [pin_name], self._make_sensor_callback(sensor_name))
        buttons.register_buttons(
            [self.material_switch_pin], self._material_callback)
        if self.feed_button_pin is not None:
            buttons.register_buttons(
                [self.feed_button_pin], self._feed_button_callback)
        if self.retract_button_pin is not None:
            buttons.register_buttons(
                [self.retract_button_pin], self._retract_button_callback)

    # --- Sensor callbacks ---

    def _make_sensor_callback(self, sensor_name):
        def callback(eventtime, state):
            self.sensor_states[sensor_name] = not bool(state)
            if self.auto_enabled and self.state not in (
                    STATE_MANUAL_FEED, STATE_MANUAL_RETRACT,
                    STATE_ERROR, STATE_DISABLED):
                self._update_rotation_distance(eventtime)
        return callback

    def _material_callback(self, eventtime, state):
        was_present = self.material_present
        self.material_present = bool(state)
        if not self.material_present and self.auto_enabled:
            self._unsync()
            self.state = STATE_IDLE
            if self.pause_on_runout:
                self._trigger_pause("buffer: filament runout detected")
        elif self.material_present and not was_present:
            if self.state == STATE_DISABLED:
                logging.info(
                    "buffer: filament detected but buffer is disabled; "
                    "use BUFFER_ENABLE to activate")
                return
            self.auto_enabled = True
            self.state = STATE_STOPPED
            self.error_msg = ""
            self._initial_fill_until = eventtime + self.initial_fill_timeout
            logging.info("buffer: filament detected, starting fill")
            self.gcode.respond_info(
                "Buffer: filament detected, starting fill")
            self._do_initial_fill(eventtime)

    # --- Extruder sync/unsync ---

    def _sync(self):
        """Sync the buffer stepper to the active extruder's trapq."""
        if self._synced_to is not None:
            return
        try:
            extruder = self.toolhead.get_extruder()
            extruder_name = extruder.get_name()
            self.extruder_stepper.sync_to_extruder(extruder_name)
            self._synced_to = extruder_name
            self._apply_multiplier(self._rd_multiplier)
            if self.state == STATE_STOPPED:
                self.state = STATE_FEEDING
                self.motor_direction = FORWARD
                self._forward_start_time = self.reactor.monotonic()
            if self.debug:
                self.gcode.respond_info(
                    "Buffer debug: synced to %s" % extruder_name)
        except Exception as e:
            logging.warning("buffer: sync failed: %s" % e)

    def _unsync(self):
        """Unsync the buffer stepper from the extruder."""
        if self._synced_to is None:
            return
        try:
            self.extruder_stepper.sync_to_extruder(None)
        except Exception as e:
            logging.warning("buffer: unsync failed: %s" % e)
        self._synced_to = None
        self._rd_multiplier = 1.0
        self.motor_direction = STOP
        if self.debug:
            self.gcode.respond_info("Buffer debug: unsynced")

    # --- Rotation distance feedback ---

    def _compute_zone(self):
        """Pure zone classification from sensor states (diagnostics only)."""
        empty = self.sensor_states["empty"]
        middle = self.sensor_states["middle"]
        full = self.sensor_states["full"]
        if empty and full:
            return None  # sensor conflict
        if empty:
            return ZONE_EMPTY
        if full and not middle:
            return ZONE_FULL
        if full and middle:
            return ZONE_FULL_MIDDLE
        if middle:
            return ZONE_MIDDLE
        return ZONE_EMPTY_MIDDLE

    def _zone_to_multiplier(self, zone):
        """Map a sensor zone to a rotation_distance multiplier.

        multiplier > 1.0 -> smaller rd -> more steps/mm -> deliver MORE
        multiplier < 1.0 -> larger rd  -> fewer steps/mm -> deliver LESS
        Formula: rd_new = base_rd / multiplier (matches AFC convention)
        """
        if zone == ZONE_MIDDLE:
            return 1.0
        if zone == ZONE_EMPTY_MIDDLE:
            return 1.0 + self.drift_gain
        if zone == ZONE_EMPTY:
            return 1.0 + self.safety_gain
        if zone == ZONE_FULL_MIDDLE:
            return 1.0 - self.drift_gain
        if zone == ZONE_FULL:
            return 1.0 - self.safety_gain
        return 1.0

    def _apply_multiplier(self, multiplier):
        """Set the buffer stepper's rotation_distance based on multiplier."""
        if multiplier <= 0.0:
            multiplier = 0.01
        self._rd_multiplier = multiplier
        new_rd = self._base_rd / multiplier
        self.extruder_stepper.stepper.set_rotation_distance(new_rd)
        if self.debug:
            self.gcode.respond_info(
                "Buffer debug: rd_mult=%.4f rd=%.4f zone=%s"
                % (multiplier, new_rd, self._current_zone))

    def _update_rotation_distance(self, eventtime):
        """Evaluate sensors and update rotation_distance multiplier."""
        zone = self._compute_zone()
        if zone is None:
            self._handle_error(
                "Sensor conflict: empty and full both triggered")
            return

        # Track zone transitions
        entered = zone != self._current_zone
        if entered:
            if self.debug:
                self.gcode.respond_info(
                    "Buffer debug: zone %s -> %s"
                    % (self._current_zone, zone))
            self._prev_zone = self._current_zone
        self._current_zone = zone

        # Ensure we're synced when auto-enabled
        if self._synced_to is None and self._initial_fill_until <= 0.0:
            self._sync()

        # Safety timeout tracking
        if zone in (ZONE_EMPTY, ZONE_FULL):
            if entered:
                self._safety_zone_start = eventtime
                self._safety_escalated = False
        else:
            self._safety_zone_start = 0.0
            self._safety_escalated = False

        # Clear initial fill once filament reaches middle or beyond
        if (self._initial_fill_until > 0.0
                and zone not in (ZONE_EMPTY, ZONE_EMPTY_MIDDLE)):
            self._initial_fill_until = 0.0
            self._sync()

        # Compute and apply multiplier
        multiplier = self._zone_to_multiplier(zone)

        # Fault escalation: if in safety zone too long, increase gain
        if (self._safety_zone_start > 0.0
                and not self._safety_escalated
                and eventtime - self._safety_zone_start
                >= self.fault_escalation_time):
            self._safety_escalated = True
            if multiplier > 1.0:
                multiplier = 1.0 + self.safety_gain * 1.5
            elif multiplier < 1.0:
                multiplier = 1.0 - self.safety_gain * 1.5

        if self._synced_to is not None:
            self._apply_multiplier(multiplier)

    # --- Initial fill ---

    def _do_initial_fill(self, eventtime):
        """Start forward feed to fill the buffer tube on filament insert."""
        if self.force_move is None:
            return
        stepper = self.extruder_stepper.stepper
        # Unsync to do an independent forward move
        self._unsync()
        self.state = STATE_FEEDING
        self.motor_direction = FORWARD
        # Calculate distance from fill timeout and speed
        dist = self.manual_speed * self.initial_fill_timeout
        try:
            self.force_move.manual_move(
                stepper, dist, self.manual_speed, self.manual_accel)
        except Exception as e:
            logging.warning("buffer: initial fill move failed: %s" % e)
        # After fill completes or if middle sensor triggered, sync
        if self._initial_fill_until > 0.0:
            self._initial_fill_until = 0.0
        self._sync()

    # --- Control timer ---

    def _control_timer_cb(self, eventtime):
        # Manual feed auto-stop on sustained full sensor
        if self.state == STATE_MANUAL_FEED:
            if self.sensor_states["full"]:
                if self._manual_feed_full_start == 0.0:
                    self._manual_feed_full_start = eventtime
                elif (eventtime - self._manual_feed_full_start
                      >= self.manual_feed_full_timeout):
                    self._stop_manual()
                    self.state = STATE_IDLE
                    self.auto_enabled = False
                    self._manual_feed_full_start = 0.0
                    self.gcode.respond_info(
                        "Buffer: manual feed stopped - full zone "
                        "sustained for %.0fs"
                        % self.manual_feed_full_timeout)
            else:
                self._manual_feed_full_start = 0.0
            return eventtime + self.control_interval

        if not self.auto_enabled:
            return eventtime + self.control_interval

        if self.state in (STATE_MANUAL_FEED, STATE_MANUAL_RETRACT,
                          STATE_ERROR, STATE_DISABLED):
            return eventtime + self.control_interval

        # Forward timeout
        if (self.state == STATE_FEEDING and self.forward_timeout > 0.0
                and self._forward_start_time > 0.0):
            self._forward_elapsed = eventtime - self._forward_start_time
            if self._forward_elapsed > self.forward_timeout:
                self._handle_error(
                    "Continuous forward motion exceeded %.0fs timeout"
                    % self.forward_timeout)
                return eventtime + self.control_interval

        # Safety timeout check
        zone = self._compute_zone()
        if zone is not None and self._safety_zone_start > 0.0:
            elapsed = eventtime - self._safety_zone_start
            if zone == ZONE_EMPTY and elapsed >= self.empty_safety_timeout:
                self._handle_error(
                    "Buffer stuck in empty zone for %.0fs"
                    % self.empty_safety_timeout)
                return eventtime + self.control_interval
            if zone == ZONE_FULL and elapsed >= self.full_safety_timeout:
                self._do_safety_retract(eventtime)
                return eventtime + self.control_interval

        # Periodic re-evaluation of rotation_distance
        if self._synced_to is not None:
            self._update_rotation_distance(eventtime)

        return eventtime + self.control_interval

    def _do_safety_retract(self, eventtime):
        """Forced retract when full zone times out."""
        logging.info("buffer: full zone safety retract")
        self.gcode.respond_info(
            "Buffer: full zone timeout - retracting")
        stepper = self.extruder_stepper.stepper
        self._unsync()
        self.state = STATE_RETRACTING
        self.motor_direction = BACK
        try:
            self.force_move.manual_move(
                stepper, -self._manual_chunk_dist,
                self.manual_speed, self.manual_accel)
        except Exception as e:
            logging.warning("buffer: safety retract failed: %s" % e)
        self._safety_zone_start = 0.0
        self._safety_escalated = False
        self._sync()

    # --- Button callbacks ---

    def _feed_button_callback(self, eventtime, state):
        self._feed_button_pressed = bool(state)
        if state:
            # Check for both-button actions
            if self._retract_button_pressed:
                if self.state == STATE_ERROR:
                    self._start_error_clear_hold(eventtime)
                else:
                    self._both_buttons_toggle()
                return
            if self.state == STATE_ERROR:
                return
            self._start_manual_feed(eventtime)
        else:
            # Release
            self._cancel_error_clear_hold()
            if self.state == STATE_MANUAL_FEED:
                self._stop_manual()
                self._button_release_restore()

    def _retract_button_callback(self, eventtime, state):
        self._retract_button_pressed = bool(state)
        if state:
            if self._feed_button_pressed:
                if self.state == STATE_ERROR:
                    self._start_error_clear_hold(eventtime)
                else:
                    self._both_buttons_toggle()
                return
            if self.state == STATE_ERROR:
                return
            self._start_manual_retract(eventtime)
        else:
            self._cancel_error_clear_hold()
            if self.state == STATE_MANUAL_RETRACT:
                self._stop_manual()
                self._button_release_restore()

    def _both_buttons_toggle(self):
        """Both buttons pressed: toggle auto-enable (IDLE <-> STOPPED)."""
        self._stop_manual()
        if self.auto_enabled:
            self.auto_enabled = False
            self._unsync()
            self.state = STATE_IDLE
        else:
            self.auto_enabled = True
            self.state = STATE_STOPPED
            self._sync()
        self.gcode.respond_info(
            "Buffer: %s via buttons"
            % ("enabled" if self.auto_enabled else "disabled"))

    def _button_release_restore(self):
        """Restore state after manual button release."""
        if self.auto_enabled:
            self.state = STATE_STOPPED
            self._sync()
        else:
            self.state = STATE_IDLE

    # --- Error clear via button hold ---

    def _start_error_clear_hold(self, eventtime):
        """Start 2s hold timer to clear error via both buttons."""
        self._error_clear_hold_start = eventtime
        if self._error_clear_timer is None:
            self._error_clear_timer = self.reactor.register_timer(
                self._error_clear_timer_cb, self.reactor.NEVER)
        self.reactor.update_timer(
            self._error_clear_timer,
            eventtime + self.error_clear_hold_time)

    def _cancel_error_clear_hold(self):
        """Cancel error clear hold if a button was released."""
        self._error_clear_hold_start = 0.0
        if self._error_clear_timer is not None:
            self.reactor.update_timer(
                self._error_clear_timer, self.reactor.NEVER)

    def _error_clear_timer_cb(self, eventtime):
        """Timer callback: clear error if both buttons still held."""
        if (self._feed_button_pressed and self._retract_button_pressed
                and self.state == STATE_ERROR):
            self._clear_error()
            self.gcode.respond_info("Buffer: error cleared via buttons")
        self._error_clear_hold_start = 0.0
        return self.reactor.NEVER

    # --- Manual feed/retract ---

    def _start_manual_feed(self, eventtime):
        """Begin manual forward feed (button or command)."""
        self._unsync()
        self.state = STATE_MANUAL_FEED
        self.motor_direction = FORWARD
        self._manual_feed_full_start = 0.0
        self._do_manual_chunk(FORWARD)

    def _start_manual_retract(self, eventtime):
        """Begin manual retract (button or command)."""
        self._unsync()
        self.state = STATE_MANUAL_RETRACT
        self.motor_direction = BACK
        self._do_manual_chunk(BACK)

    def _do_manual_chunk(self, direction):
        """Issue a single manual move chunk."""
        if self.force_move is None:
            return
        stepper = self.extruder_stepper.stepper
        dist = self._manual_chunk_dist
        if direction == BACK:
            dist = -dist
        try:
            self.force_move.manual_move(
                stepper, dist, self.manual_speed, self.manual_accel)
        except Exception as e:
            logging.warning("buffer: manual move failed: %s" % e)

    def _stop_manual(self):
        """Stop any pending manual move state."""
        self.motor_direction = STOP
        self._manual_feed_full_start = 0.0

    # --- Error handling ---

    def _handle_error(self, msg):
        self._unsync()
        self.state = STATE_ERROR
        self.error_msg = msg
        self.motor_direction = STOP
        logging.error("buffer: %s" % msg)
        self.gcode.respond_info("Buffer ERROR: %s" % msg)
        if self.pause_on_runout:
            self._trigger_pause(msg)

    def _clear_error(self):
        """Clear error state."""
        self.state = STATE_STOPPED if self.auto_enabled else STATE_IDLE
        self.error_msg = ""
        self._forward_elapsed = 0.0
        self._safety_zone_start = 0.0
        self._safety_escalated = False
        if self.auto_enabled:
            self._sync()

    def _trigger_pause(self, msg):
        try:
            pause_resume = self.printer.lookup_object("pause_resume")
            if not pause_resume.is_paused:
                self.gcode.respond_info("Buffer: pausing print - %s" % msg)
                self.gcode.run_script("PAUSE")
        except Exception:
            logging.warning("buffer: failed to trigger pause")

    # --- Status ---

    def get_status(self, eventtime):
        return {
            "state": self.state,
            "motor_direction": self.motor_direction,
            "sensor_empty": self.sensor_states["empty"],
            "sensor_middle": self.sensor_states["middle"],
            "sensor_full": self.sensor_states["full"],
            "material_present": self.material_present,
            "enabled": self.auto_enabled,
            "error": self.error_msg,
            "current_zone": self._current_zone,
            "prev_zone": self._prev_zone,
            "rd_multiplier": round(self._rd_multiplier, 4),
            "base_rotation_distance": round(self._base_rd, 4),
            "synced_to": self._synced_to,
            "forward_elapsed": round(self._forward_elapsed, 1),
            "forward_timeout": self.forward_timeout,
            "manual_speed": self.manual_speed,
            "drift_gain": self.drift_gain,
            "safety_gain": self.safety_gain,
            "is_printing": self._is_printing(),
            "manual_feed_full_timeout": self.manual_feed_full_timeout,
        }

    def _is_printing(self):
        if self._print_stats is None:
            return True
        return (self._print_stats.get_status(
            self.reactor.monotonic())["state"] == "printing")

    # --- Gcode commands ---

    def cmd_BUFFER_STATUS(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        msg = (
            "Buffer status:\n"
            "  State: %s\n"
            "  Motor: %s\n"
            "  Sensors: empty=%s middle=%s full=%s\n"
            "  Material present: %s\n"
            "  Auto enabled: %s\n"
            "  Zone: %s (prev: %s)\n"
            "  RD multiplier: %.4f (base: %.4f)\n"
            "  Synced to: %s\n"
            "  Manual speed: %.1f mm/s\n"
            "  Forward elapsed: %.1f / %.0f s\n"
            "  Drift gain: %.3f  Safety gain: %.3f\n"
            "  Printing: %s"
            % (
                status["state"],
                status["motor_direction"],
                status["sensor_empty"],
                status["sensor_middle"],
                status["sensor_full"],
                status["material_present"],
                status["enabled"],
                status["current_zone"],
                status["prev_zone"],
                status["rd_multiplier"],
                status["base_rotation_distance"],
                status["synced_to"] or "none",
                status["manual_speed"],
                status["forward_elapsed"],
                status["forward_timeout"],
                status["drift_gain"],
                status["safety_gain"],
                status["is_printing"],
            )
        )
        if status["error"]:
            msg += "\n  ERROR: %s" % status["error"]
        gcmd.respond_info(msg)

    def cmd_BUFFER_ENABLE(self, gcmd):
        self.auto_enabled = True
        self.state = STATE_STOPPED
        self.error_msg = ""
        self._forward_elapsed = 0.0
        self._safety_zone_start = 0.0
        self._safety_escalated = False
        self._sync()
        gcmd.respond_info("Buffer: automatic control enabled")

    def cmd_BUFFER_DISABLE(self, gcmd):
        self.auto_enabled = False
        self._unsync()
        self.state = STATE_DISABLED
        gcmd.respond_info("Buffer: automatic control disabled")

    def cmd_BUFFER_FEED(self, gcmd):
        if self.state == STATE_ERROR:
            gcmd.respond_info(
                "Buffer: cannot feed while in error state. "
                "Run BUFFER_CLEAR_ERROR first.")
            return
        speed = gcmd.get_float("SPEED", self.manual_speed, above=0.0)
        dist = gcmd.get_float("DIST", 50.0, above=0.0)
        self._unsync()
        self.state = STATE_MANUAL_FEED
        self.motor_direction = FORWARD
        self._manual_feed_full_start = 0.0
        stepper = self.extruder_stepper.stepper
        try:
            self.force_move.manual_move(stepper, dist, speed,
                                        self.manual_accel)
        except Exception as e:
            logging.warning("buffer: BUFFER_FEED failed: %s" % e)
        gcmd.respond_info("Buffer: feeding %.0fmm at %.1f mm/s" % (dist,
                                                                     speed))

    def cmd_BUFFER_RETRACT(self, gcmd):
        if self.state == STATE_ERROR:
            gcmd.respond_info(
                "Buffer: cannot retract while in error state. "
                "Run BUFFER_CLEAR_ERROR first.")
            return
        speed = gcmd.get_float("SPEED", self.manual_speed, above=0.0)
        dist = gcmd.get_float("DIST", 50.0, above=0.0)
        self._unsync()
        self.state = STATE_MANUAL_RETRACT
        self.motor_direction = BACK
        stepper = self.extruder_stepper.stepper
        try:
            self.force_move.manual_move(stepper, -dist, speed,
                                        self.manual_accel)
        except Exception as e:
            logging.warning("buffer: BUFFER_RETRACT failed: %s" % e)
        gcmd.respond_info("Buffer: retracting %.0fmm at %.1f mm/s"
                          % (dist, speed))

    def cmd_BUFFER_STOP(self, gcmd):
        self._stop_manual()
        self._manual_feed_full_start = 0.0
        if self.auto_enabled:
            self.state = STATE_STOPPED
            self._sync()
        else:
            self.state = STATE_IDLE
        gcmd.respond_info("Buffer: stopped")

    def cmd_BUFFER_SET_SPEED(self, gcmd):
        speed = gcmd.get_float("SPEED", above=0.0)
        self.manual_speed = speed
        gcmd.respond_info("Buffer: manual speed set to %.1f mm/s" % speed)

    def cmd_BUFFER_CLEAR_ERROR(self, gcmd):
        if self.state == STATE_ERROR:
            self._clear_error()
            gcmd.respond_info("Buffer: error cleared")
        else:
            gcmd.respond_info("Buffer: no error to clear")


def load_config(config):
    return Buffer(config)
