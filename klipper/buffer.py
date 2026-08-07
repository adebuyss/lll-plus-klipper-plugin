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

# VACTUAL scaling: TMC2208/2225 internal-velocity register expresses
# velocity in units of f_clk_internal / 2^24 microsteps per second.
# With the typical 12 MHz internal clock, one VACTUAL LSB = ~0.715
# microsteps/s.  Therefore register_value = µsteps/s / 0.715 — the
# constant is a divisor.  Verified against the working BufferMotor
# formula from commit de26ddc.
_VACTUAL_USTEP_PER_LSB = 0.715

# VACTUAL recovery polling cadence.  At 50 ms and 40 mm/s the
# worst-case overshoot past the zone-exit trigger is 2 mm, well
# inside the MIDDLE band on the reference LLL Plus.  Doubling the
# reactor-tick cost is trivial.
_RECOVERY_POLL_INTERVAL = 0.05

# Debug-heartbeat cadence (seconds).  One periodic state line so the
# log never goes silent for minutes between sensor edges — the Aug
# 2026 MCU-shutdown forensics had a 433 s blind spot before the
# fault because every log site was edge-triggered.
_HEARTBEAT_INTERVAL = 10.0

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
    ("forward_timeout", 60.0, "minval", 0.0),
    ("apply_dwell", 0.5, "minval", 0.0),
    ("fault_multiplier_low", 0.36, "minval", 0.0),
    ("fault_multiplier_high", 1.65, "minval", 1.0),
    ("fault_escalation_time", 5.0, "above", 0.0),
    ("multiplier_low", 0.9, "minval", 0.0),
    ("multiplier_high", 1.1, "minval", 1.0),
    # recovery_move_distance was used by the sidecar-chunked EMPTY
    # recovery loop.  VACTUAL recovery is continuous, so the chunk
    # size is meaningless; kept here for one release so existing
    # configs load with a warning rather than erroring out.
    ("recovery_move_distance", 5.0, "above", 0.0),
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
        # Instance name: "buffer" for [buffer], "my_buf" for [buffer my_buf]
        parts = self.name.split(None, 1)
        self.short_name = parts[1] if len(parts) > 1 else parts[0]

        # Config — new AFC-style parameters
        self.stepper_name = config.get("stepper")
        # Optional extruder binding.  When set, the buffer only syncs while
        # this extruder is the active one (enables multi-extruder setups).
        # When None, the buffer follows whatever extruder is currently
        # active and re-syncs on tool changes (single-buffer default).
        # Named 'bound_extruder' (not 'extruder') because Mainsail and some
        # Fluidd panels treat any config section whose settings expose an
        # 'extruder' field as part of that extruder's dashboard card and
        # would render buffer state inside the main Extruder panel.
        self.extruder_name = config.get("bound_extruder", None)
        self.drift_gain = config.getfloat("drift_gain", 0.02, minval=0.0,
                                          maxval=0.5)
        self.empty_safety_timeout = config.getfloat("empty_safety_timeout",
                                                    30.0, above=0.0)
        self.full_safety_timeout = config.getfloat("full_safety_timeout",
                                                   10.0, above=0.0)
        self.error_clear_hold_time = config.getfloat("error_clear_hold_time",
                                                     2.0, above=0.0)
        self.manual_speed = config.getfloat("manual_speed", 40.0, above=0.0)
        # EMPTY-zone VACTUAL recovery velocity.  Lower than manual_speed
        # because VACTUAL is an instant velocity step at the chip — there
        # is no trapezoid acceleration ramp like force_move.manual_move
        # has, so the TMC has to develop torque from rest in a single
        # microstep period.  On the LLL Plus reference hardware
        # (stealthchop, run_current=0.3) the empirical safe ceiling is
        # ~30 mm/s; 10 mm/s leaves a 3x margin and is still faster than
        # typical extruder draw (5-10 mm/s on common print profiles),
        # so EMPTY recovery still has net positive fill rate.  Bump
        # this if you have higher run_current, spreadcycle, or
        # consistently high-flow prints that outrun the default.
        self.recovery_speed = config.getfloat("recovery_speed", 10.0,
                                              above=0.0)
        # Print-start normalization drain speed.  Filament loading
        # deliberately ends with the arm at FULL (continuous feed
        # auto-stops on sustained full), so FULL at print start is
        # the expected state, not a fault.  When a print starts (or
        # BUFFER_ENABLE runs mid-print / on RESUME) with the arm at
        # FULL, drain at this speed instead of the slow 1 mm/s used
        # for mid-print drift over-fill.  Same VACTUAL stall-ceiling
        # considerations as recovery_speed apply.
        self.start_drain_speed = config.getfloat("start_drain_speed",
                                                 5.0, above=0.0)
        self.manual_accel = config.getfloat("manual_accel", 1500.0, above=0.0)
        self.manual_move_distance = config.getfloat(
            "manual_move_distance", 10.0, above=0.0)
        self.pause_on_runout = config.getboolean("pause_on_runout", True)
        # Whether ERROR-state entry (sensor conflict, recovery/safety
        # timeouts, VACTUAL failures) pauses the print.  Historically
        # pause_on_runout gated this too, despite its name; the
        # default chains to it so existing configs behave unchanged.
        # Must be parsed AFTER pause_on_runout.
        self.pause_on_error = config.getboolean("pause_on_error",
                                                self.pause_on_runout)
        self.debug = config.getboolean("debug", False)
        self.control_interval = config.getfloat("control_interval", 0.5,
                                                above=0.05)
        self.initial_fill_timeout = config.getfloat("initial_fill_timeout",
                                                    10.0, above=0.0)
        # Per-attempt cap on VACTUAL recovery.  If recovery has not
        # pulled the buffer out of the extreme zone within this many
        # seconds, the recovery poller escalates to _handle_error.
        # empty_safety_timeout is the cumulative cap for cases where
        # recovery never starts (e.g. EMPTY entered while not printing).
        self.extreme_recovery_timeout = config.getfloat(
            "extreme_recovery_timeout", 10.0, above=0.0)
        self.manual_feed_full_timeout = config.getfloat(
            "manual_feed_full_timeout", 3.0, above=0.0)
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
        self._last_active_extruder = None  # tracked for tool-change detection

        # State
        self.state = STATE_DISABLED
        self.auto_enabled = False
        # Default sensor_states to True (all sensors "triggered").
        # Rationale: Klipper's buttons module fires callbacks only on
        # state CHANGES from an internal default of 0.  For our `^!`
        # inverted hall pins, a sensor that is actively triggered at
        # boot reads raw HIGH → post-invert 0 → matches the internal
        # default and does NOT fire an initial callback.  Inactive
        # sensors read raw LOW → post-invert 1 → differ from default
        # and DO fire an initial callback with state=1 → `not bool(1)`
        # = False.  Starting True means inactive sensors flip to False
        # via the initial report, and active-at-boot sensors correctly
        # stay True without needing a callback.
        self.sensor_states = {"empty": True, "middle": True, "full": True}
        self.material_present = False
        self.error_msg = ""
        self.motor_direction = STOP
        self._current_zone = None
        self._prev_zone = None
        # Guard: suppress insertion/fill logic until the first round of
        # pin-state reports has been consumed.  Klipper's buttons module
        # fires callbacks for the initial pin state shortly after
        # klippy:ready — we need to absorb those without treating them
        # as physical events (e.g. filament inserted).
        self._initial_state_received = False
        # Track whether any hall-sensor callback has fired.  sensor_states
        # defaults to all-True (see above) but remains unreliable until at
        # least one inactive-at-boot sensor has reported and flipped to
        # False.  _update_rotation_distance defers while this is False to
        # avoid spurious sensor-conflict errors during the boot window.
        self._any_sensor_reported = False

        # Safety timeout tracking
        self._safety_zone_start = 0.0  # when we entered EMPTY or FULL

        # Initial fill
        self._initial_fill_until = 0.0

        # Button state
        self._feed_button_pressed = False
        self._retract_button_pressed = False
        self._manual_feed_full_start = 0.0
        self._error_clear_hold_start = 0.0
        self._error_clear_timer = None

        # Manual move tracking
        self._manual_chunk_dist = self.manual_move_distance
        self._continuous_feed_direction = STOP
        self._continuous_feed_speed = 0.0
        self._retract_until_clear = False

        # Print state tracking
        self._print_stats = None

        # Extreme-zone recovery via TMC VACTUAL.  When non-None, the
        # TMC is in chip-side velocity mode driving the motor at a
        # constant rate; the stepper is still nominally trapq-synced
        # but the TMC ignores STEP/DIR while VACTUAL != 0.
        self._extreme_recovery_active = None
        self._recovery_started_at = 0.0
        # Pessimistic latch: "the chip may currently be driving the
        # motor from VACTUAL."  Deliberately NOT derived from
        # _extreme_recovery_active — the two disagree in three windows
        # and every one of them ends with a running motor:
        #   1. _enter_*_recovery writes VACTUAL *before* setting
        #      _extreme_recovery_active, and bails out via _handle_error
        #      if the write is unconfirmed.  mcu_uart.reg_write is
        #      fire-and-forget at the wire level, so an unconfirmed
        #      write means "the IFCNT read-back failed", NOT "the chip
        #      never got it".
        #   2. _stop_recovery_vactual writes a NONZERO latch nudge and
        #      relies on a deferred callback ~50 ms later to clear it,
        #      while its callers clear _extreme_recovery_active
        #      immediately.
        #   3. On klippy:disconnect the reactor is tearing down, so that
        #      deferred clear may never run at all.
        # Set before any nonzero write, cleared only by a CONFIRMED
        # zero write, so the error always points at "might be running".
        self._vactual_maybe_running = False
        # One-shot per failure episode: an enable-pin driver kill has
        # been requested (deferred) or executed.  A single dead-bus
        # episode otherwise stacks doomed UART transactions — entry
        # failure -> _handle_error -> _unsync -> _stop_recovery_vactual
        # would retry the bus that just failed, at ~5 s of blocked
        # greenlet per attempt (measured: serial response timeout on a
        # dead MCU), and request the fallback again each time.  Once
        # the kill is requested the motor cannot move (EN drops), so
        # further UART attempts buy nothing.  Re-armed by the paths
        # that bring the buffer back into service (BUFFER_ENABLE,
        # error clear), which must first prove the bus works again.
        self._driver_disable_requested = False

        # Drift-correction rate-limit + zone hysteresis state.
        # _apply_multiplier rate-limits to 2 Hz during printing to
        # avoid stacked no-flush set_rotation_distance calls when a
        # sensor bounces.  _update_rotation_distance requires a zone
        # to be stable for 200 ms before committing.
        self._last_rd_apply_at = 0.0
        self._pending_multiplier = None
        self._proposed_zone = None
        self._proposed_zone_at = 0.0

        # Transient sensor-conflict deferral: timestamp of the first
        # tick where empty+full were both true.  Cleared on the next
        # clean zone read.  Promoted to a hard error from the control
        # timer if it persists past control_interval (i.e. a real
        # wiring fault, not just out-of-order callbacks within one MCU
        # report).
        self._conflict_since = 0.0

        # Per-key debug-log throttle: key -> (last_eventtime, suppressed_count)
        self._dlog_throttle = {}
        # Debug heartbeat pacing (see _control_timer_cb).
        self._last_heartbeat_at = 0.0

        # Timer handles
        self._control_timer = None
        # VACTUAL recovery poller: 50 ms cadence, single registered
        # timer reused across recovery sessions via update_timer.
        self._recovery_check_timer = None
        # Per-site chunk-pacing timers.  Each chunk schedules the next
        # attempt at eventtime + chunk_duration so a brief button tap
        # produces ONE chunk instead of bursting 3 back-to-back into
        # the MCU queue (which would all execute regardless of when
        # the user releases the button).
        self._fill_timer = None
        self._continuous_timer = None
        self._retract_clear_timer = None
        # Completion timer for one-shot timed moves: fixed-distance
        # BUFFER_FEED/RETRACT and the safety retract.  Fires at
        # move-end (chunk_move_duration) to return the state machine
        # to STOPPED/IDLE — without it those states were terminal
        # until a manual BUFFER_STOP.
        self._move_done_timer = None

        # Register events.  klippy:disconnect is critical: VACTUAL
        # persists across host disconnect and the TMC will keep
        # stepping the motor until power-off or until the enable pin
        # drops.  Reuse _handle_shutdown so the same cleanup runs on
        # both clean shutdown and connection loss.
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        self.printer.register_event_handler("klippy:disconnect",
                                            self._handle_shutdown)
        # Kalico fires this on every virtual_sdcard print start and
        # resume (mainline Klipper never emits it — harmless there).
        # Drives print-start FULL normalization; see
        # _maybe_start_print_normalization.
        self.printer.register_event_handler("print_stats:start_printing",
                                            self._handle_print_start)

        # Register gcode commands via mux so multiple [buffer ...] sections
        # can coexist and be selected with BUFFER=name.  For the bare
        # [buffer] section we also register a value=None default so that
        # existing macros calling BUFFER_* without BUFFER=... keep working.
        gcode_commands = [
            ("BUFFER_STATUS", self.cmd_BUFFER_STATUS,
             "Report buffer status (BUFFER=name to target a specific buffer)"),
            ("BUFFER_ENABLE", self.cmd_BUFFER_ENABLE,
             "Enable automatic buffer control"),
            ("BUFFER_DISABLE", self.cmd_BUFFER_DISABLE,
             "Disable buffer control and stop motor"),
            ("BUFFER_FEED", self.cmd_BUFFER_FEED,
             "Manually feed filament forward"),
            ("BUFFER_RETRACT", self.cmd_BUFFER_RETRACT,
             "Manually retract filament"),
            ("BUFFER_RETRACT_UNTIL_CLEAR",
             self.cmd_BUFFER_RETRACT_UNTIL_CLEAR,
             "Retract until filament presence switch clears"),
            ("BUFFER_STOP", self.cmd_BUFFER_STOP,
             "Stop motor and return to auto mode"),
            ("BUFFER_SET_SPEED", self.cmd_BUFFER_SET_SPEED,
             "Set manual feed/retract speed in mm/s"),
            ("BUFFER_CLEAR_ERROR", self.cmd_BUFFER_CLEAR_ERROR,
             "Clear buffer error state"),
        ]
        for cmd, handler, desc in gcode_commands:
            self.gcode.register_mux_command(
                cmd, "BUFFER", self.short_name, handler, desc=desc)
            if self.short_name == "buffer":
                self.gcode.register_mux_command(
                    cmd, "BUFFER", None, handler, desc=desc)

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
        # Look up the TMC driver for direct VACTUAL register writes.
        # Recovery uses chip-side velocity mode (TMC ignores STEP/DIR
        # while VACTUAL != 0) so we can drive the motor independently
        # of Klipper's motion pipeline.  Try common variants — the
        # plugin targets TMC2208/2225, but TMC2209 shares the same
        # field layout via inheritance.
        self._mcu_tmc = None
        for prefix in ("tmc2208", "tmc2225", "tmc2209"):
            try:
                tmc_obj = self.printer.lookup_object(
                    "%s extruder_stepper %s"
                    % (prefix, self.stepper_name))
                self._mcu_tmc = tmc_obj.mcu_tmc
                break
            except Exception:
                continue
        if self._mcu_tmc is None:
            logging.warning(
                "buffer[%s]: TMC driver not found — VACTUAL recovery "
                "disabled" % self.short_name)
        # Capture baseline rotation_distance
        self._base_rd = (
            self.extruder_stepper.stepper.get_rotation_distance()[0])
        self._rd_multiplier = 1.0
        # Cache steps_per_mm for VACTUAL register conversion.
        # get_step_dist() returns mm per microstep, folding in the
        # user's rotation_distance, microsteps, and gear_ratio exactly
        # as Klipper computed them at config time.  No literals — the
        # cache stays consistent with set_rotation_distance because
        # both live on the same stepper object.
        try:
            self._steps_per_mm = (
                1.0 / self.extruder_stepper.stepper.get_step_dist())
        except Exception as e:
            logging.warning(
                "buffer[%s]: get_step_dist() unavailable (%s); "
                "VACTUAL conversion disabled" % (self.short_name, e))
            self._steps_per_mm = None
        # Boot-time VACTUAL hygiene.  The TMC is powered from the motor
        # rail, not the MCU, so a nonzero VACTUAL from a prior session
        # survives klippy restarts and even FIRMWARE_RESTART's MCU
        # reset — and there is no end-of-session path guaranteed to
        # clear it: on an MCU shutdown every UART write fails (the
        # firmware's enable-pin drop is what stops the motor), and on
        # klippy:disconnect the MCU's own disconnect handler closes the
        # serial before ours runs.  A fresh process therefore cannot
        # trust the chip.  Mark it unknown and run the full stop
        # sequence (zero + direction-latch nudge); a confirmed zero
        # clears the latch, a failure leaves it set and kills the
        # driver via the enable pin.
        if self._mcu_tmc is not None:
            self._vactual_maybe_running = True
            self._stop_recovery_vactual()
        # Undo the auto-sync that [extruder_stepper]'s handle_connect
        # performed at klippy:connect.  The buffer stepper should not
        # follow the extruder until the user enables the buffer.
        try:
            self.extruder_stepper.sync_to_extruder(None)
        except Exception:
            pass
        # Record the currently active extruder so the control timer can
        # detect tool changes via polling (see _control_timer_cb).
        try:
            self._last_active_extruder = (
                self.toolhead.get_extruder().get_name())
        except Exception:
            self._last_active_extruder = None
        # Transition to IDLE
        self.state = STATE_IDLE
        # Look up print_stats
        try:
            self._print_stats = self.printer.lookup_object("print_stats")
        except Exception:
            logging.info("buffer[%s]: print_stats not available"
                         % self.short_name)
        # Emit deprecation warnings
        if self._deprecated_warnings:
            msg = ("Buffer[%s]: deprecated config params ignored: %s. "
                   "These will be removed in a future release."
                   % (self.short_name,
                      ", ".join(self._deprecated_warnings)))
            logging.warning("buffer[%s]: %s"
                            % (self.short_name, msg))
            self.gcode.respond_info(msg)
        # Warn if multiple buffers omit the 'extruder' binding.  Two
        # unbound buffers both follow the active extruder, which is
        # usually a misconfiguration on multi-extruder printers.
        try:
            unbound = []
            for obj_name, obj in self.printer.lookup_objects("buffer"):
                if getattr(obj, "extruder_name", "_missing_") is None:
                    unbound.append(getattr(obj, "short_name", obj_name))
            if len(unbound) > 1:
                logging.warning(
                    "buffer[%s]: multiple buffers without 'extruder' "
                    "param will all follow the active extruder: %s"
                    % (self.short_name, ", ".join(unbound)))
        except Exception:
            pass
        # Start control timer
        self._control_timer = self.reactor.register_timer(
            self._control_timer_cb, self.reactor.monotonic() + 1.0)
        logging.info("buffer[%s]: ready (base_rd=%.4f)"
                     % (self.short_name, self._base_rd))

    def _handle_shutdown(self):
        # VACTUAL persists across host disconnect; the chip will keep
        # stepping the motor unless we explicitly stop it.  Try the
        # register write first, then drop the enable pin as a fallback
        # — disabling the driver halts the TMC internal step generator.
        #
        # Both steps stay UNCONDITIONAL here, deliberately: this runs on
        # klippy:disconnect as well as klippy:shutdown, and on
        # disconnect the reactor is tearing down, so any deferred
        # _clear_vactual_latch_nudge may never fire.  Do not gate either
        # of these on _vactual_maybe_running — shutdown is exactly where
        # the belt-and-braces is worth its cost.
        try:
            self._write_vactual(0)
        except Exception:
            pass
        try:
            self.gcode.run_script(
                "SET_STEPPER_ENABLE STEPPER=%s ENABLE=0"
                % self.stepper_name)
        except Exception:
            pass
        self._unsync("shutdown")
        self.state = STATE_DISABLED
        self.auto_enabled = False
        logging.info("buffer[%s]: shutdown" % self.short_name)

    # --- VACTUAL helpers ---

    def _mm_per_s_to_vactual(self, mm_per_s):
        """Convert mm/s of filament motion to a signed VACTUAL register
        value.  Reads steps_per_mm from the live stepper (cached in
        _handle_ready) so changes to rotation_distance / microsteps /
        gear_ratio in the user's config take effect automatically.

        Polarity: on the reference Mellow LLL Plus wiring, the TMC's
        internal VACTUAL direction is INVERTED relative to STEP/DIR
        (verified on hardware — positive STEP/DIR pulses drive
        filament forward, but positive VACTUAL drives reverse).
        Negate the result so callers can treat the input mm/s with
        the same sign convention as manual_move (+forward / -reverse).
        """
        if self._steps_per_mm is None:
            return 0
        return -int(mm_per_s * self._steps_per_mm / _VACTUAL_USTEP_PER_LSB)

    def _write_vactual(self, register_value):
        """Write the TMC VACTUAL register directly.  This is the
        critical part of the refactor: SET_TMC_FIELD via gcode dispatch
        schedules the register write at toolhead.get_last_move_time()
        — which during a print is the lookahead tail, ~1-2 s ahead of
        MCU execution.  mcu_tmc.set_register with no print_time
        argument applies immediately, which is what recovery needs.
        Mirrors the working pattern from BufferMotor (commit de26ddc).
        """
        if self._mcu_tmc is None:
            return False
        # Latch BEFORE the write, not after.  Klipper's
        # MCU_TMC_uart.set_register only raises once its IFCNT
        # read-back retries are exhausted; the underlying
        # mcu_uart.reg_write is one-way and never waits for a reply.
        # So a failed nonzero write means "could not confirm", and the
        # chip may be spinning.  Latching first keeps every downstream
        # cleanup path armed in exactly that case.
        if register_value:
            self._vactual_maybe_running = True
        try:
            self._mcu_tmc.set_register("VACTUAL", register_value)
            if not register_value:
                # Only a CONFIRMED zero write releases the latch.
                self._vactual_maybe_running = False
            # Success log (debug-gated): 590 recoveries in the Aug
            # 2026 incident log left no record of what was written.
            self._dlog(None, "VACTUAL write %d ok", register_value)
            return True
        except Exception as e:
            logging.warning("buffer[%s]: VACTUAL write failed: %s"
                            % (self.short_name, e))
            return False

    def _disable_stepper_driver(self, reason):
        """Last-resort motor stop that does not touch the TMC UART.

        Every VACTUAL stop path runs over the same bit-banged UART that
        just failed, so retrying it is the one thing guaranteed not to
        help — and each retry is expensive (set_register retries 5x,
        and every IFCNT read inside retries 5x on top of that, with a
        0.5 s response timeout per attempt).  The enable pin is a plain
        GPIO on the same MCU: dropping it disables the driver output
        stage, which halts the chip's internal step generator whatever
        VACTUAL happens to contain.

        The motor goes limp rather than held.  For a filament buffer
        that is strictly better than a runaway, and the buffer is
        heading into an error state anyway on every path that calls
        this.

        The run_script itself is DEFERRED to a reactor callback rather
        than issued inline.  This helper is reachable from gcode
        command handlers — BUFFER_DISABLE / BUFFER_FEED / BUFFER_RETRACT
        / BUFFER_RETRACT_UNTIL_CLEAR all reach it via _unsync ->
        _stop_recovery_vactual — and Kalico's gcode mutex is not
        reentrant: reactor.ReactorMutex.__enter__ parks the calling
        greenlet on reactor.pause(NEVER) when the lock is already held,
        so an inline gcode.run_script from inside a command handler
        would hang klippy permanently.  A reactor callback runs on a
        later iteration, once the handler has returned and released the
        mutex.  The extra latency is one reactor iteration, which is
        irrelevant next to a motor that is already running away.

        _handle_shutdown deliberately does NOT route through here: it
        runs on klippy:disconnect where the reactor may never drain
        another callback, so it issues its own direct run_script.

        One-shot per failure episode (_driver_disable_requested): the
        first requester wins; repeats are dropped so the error unwind
        does not spam the console or queue duplicate scripts.
        """
        if self._driver_disable_requested:
            return
        self._driver_disable_requested = True
        logging.warning(
            "buffer[%s]: disabling driver via enable pin — %s"
            % (self.short_name, reason))
        self.gcode.respond_info(
            "Buffer[%s]: TMC UART unresponsive (%s); disabling driver to "
            "stop the motor. Re-enable with BUFFER_ENABLE once the "
            "connection is healthy." % (self.short_name, reason))
        self.reactor.register_callback(self._do_disable_stepper_driver)

    def _do_disable_stepper_driver(self, eventtime):
        try:
            self.gcode.run_script(
                "SET_STEPPER_ENABLE STEPPER=%s ENABLE=0"
                % self.stepper_name)
        except Exception as e:
            logging.error(
                "buffer[%s]: enable-pin fallback FAILED (%s) — motor may "
                "still be running" % (self.short_name, e))

    def _stop_recovery_vactual(self):
        """Stop VACTUAL recovery and restore correct STEP/DIR
        interpretation via a brief directional nudge.  Returns True
        when the stop (VACTUAL=0) was confirmed on the chip, False
        when it could not be — callers that intend to re-arm the
        driver (BUFFER_ENABLE, error clear) must treat False as
        "do not proceed".

        The TMC2208/2225 latches the sign of the last nonzero VACTUAL
        write and applies it to subsequent STEP pulses regardless of
        the DIR pin.  EMPTY recovery writes a negative VACTUAL value
        (forward-filament intent, negated by _mm_per_s_to_vactual's
        wiring inversion); a bare VACTUAL=0 would leave the latch
        "negative" and the synced gear stepper would step backward
        during the extruder's next forward motion (observed: buffer
        drains back to EMPTY in ~3 s, recovery refires).  Manual
        rescue with SET_TMC_FIELD VACTUAL=100 then VACTUAL=0
        restores correct direction, which this method automates:

        1. Write 0 to stop recovery immediately.
        2. Write a small positive raw register value to set the
           chip's internal direction latch to match the sign of
           normal forward STEP/DIR motion under this wiring.
        3. Schedule a final 0 ~50 ms later to return to step/dir
           mode with the corrected latch.  Physical motion at
           0.1 mm/s held 50 ms is well under one microstep.
        """
        if self._driver_disable_requested:
            # A previous write this episode already proved the bus dead
            # and the driver kill is pending or done — the motor cannot
            # move with EN dropped, so more UART only stalls the
            # reactor greenlet (~5 s per doomed attempt).  The latch
            # stays set; whichever path re-enables must clear it first.
            return False
        if not self._write_vactual(0):
            # The stop write is the one that must not fail: the chip is
            # driving the motor right now and the UART just refused.
            # Retrying over the same bus is pointless, and issuing the
            # nudge would only queue two more doomed transactions.
            # Kill the driver instead and leave the latch set so any
            # later cleanup path still tries to zero VACTUAL.
            self._disable_stepper_driver("VACTUAL stop write failed")
            return False
        # Negate _mm_per_s_to_vactual's caller-facing "+forward"
        # convention to produce a POSITIVE raw register value —
        # opposite sign from EMPTY recovery's push, matching the
        # manual rescue.
        nudge = -self._mm_per_s_to_vactual(0.1)
        self._write_vactual(nudge)
        self.reactor.register_callback(
            self._clear_vactual_latch_nudge,
            self.reactor.monotonic() + 0.05)
        return True

    def _clear_vactual_latch_nudge(self, eventtime):
        # Don't clobber a fresh recovery push if recovery has
        # restarted within the 50 ms nudge window.
        if self._extreme_recovery_active is not None:
            return
        if not self._write_vactual(0):
            # The nudge is still live (0.1 mm/s creep, STEP/DIR still
            # overridden) and we cannot clear it.  Same reasoning as
            # the stop path: stop trusting the UART, drop the driver.
            self._disable_stepper_driver(
                "VACTUAL latch-nudge clear failed")

    def _chunk_move_duration(self, dist, speed, accel):
        """Real-time duration of a single force_move.manual_move
        chunk.  Used to pace the chained chunk loops: scheduling the
        next callback at eventtime + duration ensures one chunk
        completes before the next is queued, so a brief button tap
        produces one chunk instead of a back-to-back burst that's
        already committed to the MCU before the release callback
        fires.  Matches the trapezoid math in force_move.calc_move_time.
        """
        d = abs(dist)
        if d <= 0.0 or speed <= 0.0:
            return 0.0
        if accel <= 0.0:
            return d / speed
        # Klipper's trapezoid: if max_cruise_v² < speed², we never
        # reach the requested speed.
        max_cruise_v2 = d * accel
        if max_cruise_v2 < speed * speed:
            cruise_v = max_cruise_v2 ** 0.5
        else:
            cruise_v = speed
        accel_t = cruise_v / accel
        # Distance covered during accel+decel: cruise_v² / accel.
        cruise_t = (d - cruise_v * cruise_v / accel) / cruise_v
        if cruise_t < 0.0:
            cruise_t = 0.0
        return accel_t + cruise_t + accel_t

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
            self._any_sensor_reported = True
            if self.auto_enabled and self.state not in (
                    STATE_MANUAL_FEED, STATE_MANUAL_RETRACT,
                    STATE_ERROR, STATE_DISABLED):
                self._update_rotation_distance(eventtime)
        return callback

    def _cancel_fill(self):
        """Cancel any in-progress initial fill loop."""
        self._initial_fill_until = 0.0
        self._cancel_fill_timer()

    def _material_callback(self, eventtime, state):
        was_present = self.material_present
        self.material_present = bool(state)
        if not self._initial_state_received:
            # First report from the MCU after startup — record the state
            # but do NOT treat it as a physical insertion or removal.
            self._initial_state_received = True
            if self.material_present:
                logging.info(
                    "buffer[%s]: filament already present at startup"
                    % self.short_name)
            return
        if not self.material_present and self.auto_enabled:
            self._cancel_fill()
            self._unsync("runout")
            self.state = STATE_IDLE
            if self.pause_on_runout:
                self._trigger_pause("buffer: filament runout detected")
        elif self.material_present and not was_present:
            if self.state in (STATE_DISABLED, STATE_ERROR):
                logging.info(
                    "buffer[%s]: filament detected but buffer is %s; "
                    "use BUFFER_ENABLE or BUFFER_CLEAR_ERROR"
                    % (self.short_name, self.state))
                return
            self.auto_enabled = True
            self.state = STATE_STOPPED
            self.error_msg = ""
            self._initial_fill_until = eventtime + self.initial_fill_timeout
            logging.info("buffer[%s]: filament detected, starting fill"
                         % self.short_name)
            self.gcode.respond_info(
                "Buffer[%s]: filament detected, starting fill"
                % self.short_name)
            self._do_initial_fill(eventtime)

    # --- Extruder sync/unsync ---

    def _sync(self, reason=None):
        """Sync the buffer stepper to the active extruder's trapq.

        If this buffer is bound to a specific extruder via the 'extruder'
        config param, only sync when that extruder is currently active.
        Otherwise sync to whatever extruder is active.

        reason is a short slug threaded into the debug log — with a
        dozen call sites, a bare "synced" line is nearly useless when
        reconstructing a failure from klippy.log.
        """
        if self._synced_to is not None:
            return
        try:
            extruder = self.toolhead.get_extruder()
            extruder_name = extruder.get_name()
            # If bound to a specific extruder, only sync when it's active.
            if (self.extruder_name is not None
                    and extruder_name != self.extruder_name):
                self._dlog(None, "skip sync (active=%s, bound=%s)",
                           extruder_name, self.extruder_name)
                return
            self.extruder_stepper.sync_to_extruder(extruder_name)
            self._synced_to = extruder_name
            self._apply_multiplier(self._rd_multiplier)
            if self.state == STATE_STOPPED:
                self.state = STATE_FEEDING
                self.motor_direction = FORWARD
            self._dlog(None, "synced to %s (%s)", extruder_name,
                       reason or "-")
        except Exception as e:
            logging.warning("buffer[%s]: sync failed: %s"
                            % (self.short_name, e))

    def _unsync(self, reason=None):
        """Unsync the buffer stepper from the extruder.  reason is a
        short slug for the debug log (see _sync).

        Restores the stepper's rotation_distance to the baseline so
        subsequent manual moves (BUFFER_FEED, BUFFER_RETRACT, safety
        retract, fill chunks) use the correct mm->step conversion
        regardless of which zone the buffer was in when we unsynced.
        Without this, force_move.manual_move would use the last applied
        zone multiplier and move the wrong amount of filament.
        """
        # VACTUAL cleanup invariant: every code path that ends a sync
        # session must clear the chip-side velocity register too.  The
        # TMC does NOT clear VACTUAL on its own; without this, BUFFER_
        # DISABLE / runout / error / tool-change while VACTUAL is
        # active would leave the motor running indefinitely.
        #
        # Gated on _vactual_maybe_running, NOT on
        # _extreme_recovery_active (see the latch's definition in
        # __init__ for why the latter is unsafe here).  The latch is
        # False only when no nonzero VACTUAL has been written since the
        # last confirmed zero, so skipping is provably safe — and it
        # skips on every _unsync of a print that never entered
        # recovery, which is the overwhelming majority of them.  Each
        # skipped cleanup avoids three set_register round trips on a
        # bit-banged 40 kbaud UART.
        if self._vactual_maybe_running:
            self._stop_recovery_vactual()
        self._extreme_recovery_active = None
        self._recovery_started_at = 0.0
        if self._recovery_check_timer is not None:
            self.reactor.update_timer(
                self._recovery_check_timer, self.reactor.NEVER)
        if self._synced_to is None:
            return
        try:
            # ExtruderStepper.sync_to_extruder(None) flushes step
            # generation internally before unbinding the trapq, so
            # we don't need a separate flush here — after this call
            # the stepper has no trapq and is no longer generating
            # steps, so changing step_dist is safe.
            self.extruder_stepper.sync_to_extruder(None)
            self.extruder_stepper.stepper.set_rotation_distance(self._base_rd)
        except Exception as e:
            logging.warning("buffer[%s]: unsync failed: %s"
                            % (self.short_name, e))
        self._synced_to = None
        self._rd_multiplier = 1.0
        self.motor_direction = STOP
        self._safety_zone_start = 0.0
        self._dlog(None, "unsynced (%s)", reason or "-")

    def _handle_extruder_change(self, new_extruder_name):
        """React to the active extruder changing.

        For bound buffers: sync iff the new extruder matches our binding,
        otherwise unsync.  For unbound buffers: re-sync to whatever is
        now active.  No-op while disabled, errored, or auto-disabled.
        """
        if self.state in (STATE_DISABLED, STATE_ERROR):
            return
        if not self.auto_enabled:
            return
        if self.extruder_name is not None:
            if new_extruder_name == self.extruder_name:
                if self._synced_to is None:
                    self._sync("tool-change")
            else:
                if self._synced_to is not None:
                    self._unsync("tool-change")
                    # Return to STOPPED so _sync() can promote back to
                    # FEEDING when our extruder is active again.
                    if self.state == STATE_FEEDING:
                        self.state = STATE_STOPPED
        else:
            # Unbound — follow whatever is active.
            if self._synced_to is not None:
                self._unsync("tool-change")
            self._sync("tool-change")

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
        """Map a non-extreme sensor zone to a rotation_distance multiplier.

        multiplier > 1.0 -> smaller rd -> more steps/mm -> deliver MORE
        multiplier < 1.0 -> larger rd  -> fewer steps/mm -> deliver LESS
        Formula: rd_new = base_rd / multiplier (matches AFC convention)

        The extreme zones (ZONE_FULL, ZONE_EMPTY) intentionally return
        1.0 here because they are handled by the unsync-and-recover
        path in _update_rotation_distance — at the extremes we stop
        chasing with rotation_distance entirely.
        """
        if zone == ZONE_EMPTY_MIDDLE:
            return 1.0 + self.drift_gain
        if zone == ZONE_FULL_MIDDLE:
            return 1.0 - self.drift_gain
        return 1.0

    def _dlog(self, eventtime, fmt, *args):
        """Debug log with reactor-eventtime prefix for cross-correlation.

        Embeds the reactor monotonic eventtime so klippy.log lines line
        up with toolhead/MCU events when chasing mid-print regressions.
        Pass None when eventtime is not in scope; the helper falls back
        to reactor.monotonic() (cheap but a syscall, so prefer to pass
        the existing eventtime on hot paths).
        """
        if not self.debug:
            return
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        logging.info("buffer[%s] t=%.3f debug: " + fmt,
                     self.short_name, eventtime, *args)

    def _dlog_throttled(self, eventtime, key, window, fmt, *args):
        """Throttled debug log: collapse repeated same-key entries inside
        `window` seconds.  On the first call after the gap, emit the
        line plus a "(N suppressed)" summary if any were dropped.

        Used to tame rd_mult / zone-transition chatter when filament
        oscillates at a sensor boundary; bouncing 30+ times per second
        between two adjacent zones would otherwise drown other Klipper
        log messages.  Each distinct key throttles independently so a
        genuinely new edge breaks through immediately.
        """
        if not self.debug:
            return
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        last = self._dlog_throttle.get(key)
        if last is not None:
            last_t, count = last
            if eventtime - last_t < window:
                self._dlog_throttle[key] = (last_t, count + 1)
                return
            if count > 0:
                logging.info(
                    "buffer[%s] t=%.3f debug: (suppressed %d %s repeats)",
                    self.short_name, eventtime, count, key)
        self._dlog_throttle[key] = (eventtime, 0)
        logging.info("buffer[%s] t=%.3f debug: " + fmt,
                     self.short_name, eventtime, *args)

    # --- Extreme-zone recovery (VACTUAL) ---
    #
    # Recovery drives the stepper independently of Klipper's motion
    # pipeline by writing the TMC VACTUAL register.  While VACTUAL is
    # non-zero the TMC ignores STEP/DIR pins and generates steps
    # internally — so the buffer stepper can stay nominally synced to
    # the extruder's trapq, the print continues without hitch, and
    # we don't have to worry about stepcompress invariants.
    #
    # EMPTY recovery: forward VACTUAL at recovery_speed (NOT manual_speed
    #                 — VACTUAL is an instant velocity step, no trapezoid
    #                 ramp, so the safe ceiling is much lower than the
    #                 trapezoid-accelerated manual_speed).
    # FULL recovery:  reverse VACTUAL at 1 mm/s (slow drain — fast
    # enough to recover within timeout, slow enough not to fight a
    # forward-feeding extruder).

    def _enter_empty_recovery(self, eventtime):
        if self._extreme_recovery_active is not None:
            return
        register = self._mm_per_s_to_vactual(self.recovery_speed)
        if not self._write_vactual(register):
            # Unconfirmed write: the chip may already be feeding at
            # recovery_speed.  Stop it by the one route that does not
            # depend on the bus that just failed, then escalate.  The
            # latch gate skips the fallback when no write can have
            # left at all (_mcu_tmc is None).
            if self._vactual_maybe_running:
                self._disable_stepper_driver(
                    "EMPTY recovery VACTUAL write failed")
            self._handle_error("EMPTY recovery: VACTUAL write failed")
            return
        self._extreme_recovery_active = ZONE_EMPTY
        self._recovery_started_at = eventtime
        self._dlog(eventtime,
                   "recovery enter EMPTY vactual=%d (%.1f mm/s)",
                   register, self.recovery_speed)
        self._arm_recovery_timer(eventtime + _RECOVERY_POLL_INTERVAL)

    def _enter_full_recovery(self, eventtime, drain_speed=1.0):
        """Reverse-VACTUAL drain out of the FULL band.  The default
        1 mm/s is the mid-print drift-over-fill drain (slow enough
        not to fight a forward-feeding extruder); print-start
        normalization passes start_drain_speed instead."""
        if self._extreme_recovery_active is not None:
            return
        register = -self._mm_per_s_to_vactual(drain_speed)
        if not self._write_vactual(register):
            # Unconfirmed write: the chip may already be draining in
            # reverse.  Same reasoning as _enter_empty_recovery.
            if self._vactual_maybe_running:
                self._disable_stepper_driver(
                    "FULL recovery VACTUAL write failed")
            self._handle_error("FULL recovery: VACTUAL write failed")
            return
        self._extreme_recovery_active = ZONE_FULL
        self._recovery_started_at = eventtime
        self._dlog(eventtime,
                   "recovery enter FULL vactual=%d (%.1f mm/s reverse)",
                   register, drain_speed)
        self._arm_recovery_timer(eventtime + 0.5)

    def _maybe_start_print_normalization(self, eventtime):
        """One-shot FULL drain at print start.  Loading parks the arm
        at FULL by design, so a print beginning with a full arm is the
        common case, not drift — drain it at start_drain_speed rather
        than the mid-print 1 mm/s.  Stateless: all gates re-checked at
        call time, so the three triggers (print_stats:start_printing,
        BUFFER_ENABLE while printing, auto-re-sync while printing) can
        fire in any order and duplicates are no-ops via the
        recovery-active gate.  The per-attempt extreme_recovery_timeout
        caps the drain (10 s at 5 mm/s = 50 mm); raise it for longer
        buffer arms."""
        if not self.auto_enabled:
            return
        if self._synced_to is None:
            return
        if self._extreme_recovery_active is not None:
            return
        if not self.material_present:
            return
        if not self._is_printing():
            return
        if self._compute_zone() != ZONE_FULL:
            return
        self._dlog(eventtime,
                   "print-start normalization: FULL at start, draining "
                   "at %.1f mm/s", self.start_drain_speed)
        self._enter_full_recovery(eventtime, self.start_drain_speed)

    def _handle_print_start(self):
        """print_stats:start_printing handler (Kalico-only event; on
        mainline Klipper it never fires and the BUFFER_ENABLE /
        auto-re-sync hooks carry the feature).  Fired from
        virtual_sdcard's work_handler on every print start AND every
        resume, before the file's first line runs.  Deferred to a
        reactor callback: event handlers run inside the work_handler
        dispatch and must stay cheap."""
        self.reactor.register_callback(
            self._maybe_start_print_normalization)

    def _arm_recovery_timer(self, wake):
        if self._recovery_check_timer is None:
            self._recovery_check_timer = self.reactor.register_timer(
                self._check_recovery_done, wake)
        else:
            self.reactor.update_timer(
                self._recovery_check_timer, wake)

    def _check_recovery_done(self, eventtime):
        """Poll recovery state every _RECOVERY_POLL_INTERVAL.  Exits
        on zone return, material runout, or per-attempt timeout."""
        if self._extreme_recovery_active is None:
            return self.reactor.NEVER
        active = self._extreme_recovery_active
        if not self.material_present:
            self._stop_recovery_vactual()
            self._exit_extreme_recovery(eventtime, "material-gone")
            return self.reactor.NEVER
        if (eventtime - self._recovery_started_at
                > self.extreme_recovery_timeout):
            self._stop_recovery_vactual()
            self._extreme_recovery_active = None
            self._handle_error(
                "%s recovery exceeded %.0fs"
                % (active, self.extreme_recovery_timeout))
            return self.reactor.NEVER
        zone = self._compute_zone()
        if zone is not None:
            if (active == ZONE_EMPTY
                    and zone not in (ZONE_EMPTY, ZONE_EMPTY_MIDDLE)):
                self._stop_recovery_vactual()
                self._exit_extreme_recovery(eventtime, "reached-middle")
                return self.reactor.NEVER
            if (active == ZONE_FULL
                    and zone not in (ZONE_FULL, ZONE_FULL_MIDDLE)):
                self._stop_recovery_vactual()
                self._exit_extreme_recovery(eventtime, "reached-middle")
                return self.reactor.NEVER
        return eventtime + _RECOVERY_POLL_INTERVAL

    def _exit_extreme_recovery(self, eventtime, reason):
        """Clear recovery state.  The stepper was never unsynced
        during VACTUAL recovery, so there is no resync to perform."""
        if self._extreme_recovery_active is None:
            return
        prev_zone = self._extreme_recovery_active
        elapsed = (eventtime - self._recovery_started_at
                   if self._recovery_started_at > 0.0 else 0.0)
        self._extreme_recovery_active = None
        self._recovery_started_at = 0.0
        self._dlog(eventtime,
                   "recovery exit reason=%s prev=%s zone=%s elapsed=%.2fs",
                   reason, prev_zone, self._current_zone, elapsed)

    def _apply_multiplier(self, multiplier):
        """Set the buffer stepper's rotation_distance based on multiplier.

        Deliberately does NOT call flush_step_generation() — this is
        the AFC pattern, NOT canonical Klipper.  At small multiplier
        deltas (drift_gain <= 0.02 ⇒ <= 2% step_dist change) over a
        buffer stepper's shallow queued depth (~5-20 ms), the resulting
        sub-step timing glitch is mechanically invisible.  The
        canonical flush, by contrast, drains the entire main-toolhead
        lookahead (1-10 ms pipeline drain per zone transition) and
        shows up as visible XYZ stalls on long fast moves.  AFC makes
        the same trade-off.

        Rate-limited to 2 Hz during printing: sensor bouncing at a
        zone boundary can otherwise stack up no-flush set_rotation_
        distance calls every few ms, each of which still has some
        cost.  The deferred update lands on the next control timer
        tick via _pending_multiplier.
        """
        if multiplier <= 0.0:
            multiplier = 0.01
        if abs(multiplier - self._rd_multiplier) < 1e-9:
            self._pending_multiplier = None
            return
        now = self.reactor.monotonic()
        if (self._is_printing()
                and now - self._last_rd_apply_at < 0.5):
            self._pending_multiplier = multiplier
            return
        self._rd_multiplier = multiplier
        self._pending_multiplier = None
        self._last_rd_apply_at = now
        new_rd = self._base_rd / multiplier
        self.extruder_stepper.stepper.set_rotation_distance(new_rd)
        self._dlog_throttled(
            None, "rd_mult zone=%s" % self._current_zone, 0.25,
            "rd_mult=%.4f rd=%.4f zone=%s",
            multiplier, new_rd, self._current_zone)

    def _update_rotation_distance(self, eventtime):
        """Evaluate sensors and update rotation_distance multiplier."""
        # Defer until at least one hall-sensor callback has fired.
        # sensor_states defaults to all-True at boot so that active-at-
        # boot sensors (which don't fire initial callbacks under Klipper's
        # buttons module change-detection logic) are correctly reflected.
        # During the short window before the first callback, the default
        # would spuriously look like a sensor conflict.
        if not self._any_sensor_reported:
            return
        zone = self._compute_zone()
        if zone is None:
            # Transient empty+full conflict: Klipper's buttons module
            # dispatches edge callbacks serially across one MCU report,
            # so a fast EMPTY->FULL transit can arrive as
            # "full=True (set first) before empty=False (set next)" —
            # _compute_zone sees both true and returns None.  Treat as
            # a deferral; the next callback resolves it.  Only escalate
            # if the conflict persists past control_interval (the
            # control timer promotes it to _handle_error).
            if self._conflict_since <= 0.0:
                self._conflict_since = eventtime
            self._dlog(eventtime,
                       "transient sensor conflict — deferring")
            return
        self._conflict_since = 0.0

        # Track zone transitions
        entered = zone != self._current_zone
        if entered:
            # Throttled — adjacent-zone bouncing at a sensor boundary
            # otherwise produces 40+ log lines per second.  The key
            # encodes both endpoints so a true new edge (e.g.
            # full_middle -> full) breaks through immediately.
            self._dlog_throttled(
                eventtime,
                "zone %s->%s" % (self._current_zone, zone), 0.25,
                "zone %s -> %s", self._current_zone, zone)
            self._prev_zone = self._current_zone
        self._current_zone = zone

        # Ensure we're synced when auto-enabled.  Skip during extreme
        # recovery — the recovery owns the unsynced state and will
        # resync via _exit_extreme_recovery when the zone returns.
        # Material gate: after a runout _material_callback unsyncs but
        # leaves auto_enabled=True in IDLE; filament is still moving
        # during the runout, so without this gate the very next hall
        # edge would re-sync a buffer that has nothing to feed.
        if (self._synced_to is None
                and self._initial_fill_until <= 0.0
                and self._extreme_recovery_active is None
                and self.material_present):
            self._sync("auto-resync")
            if self._synced_to is not None:
                # Re-synced while printing with the arm at FULL —
                # normalize at start_drain_speed before the generic
                # 1 mm/s entry below can claim it.  When this fires,
                # the mid-recovery branch returns for the rest of
                # this tick; the per-attempt extreme_recovery_timeout
                # is the governing cap (cumulative arming is skipped
                # via in_active_recovery).
                self._maybe_start_print_normalization(eventtime)

        # Safety timeout tracking.  Edge-based arming on zone entry
        # preserves the be70969 intent: stale state from before printing
        # must not accrue toward the timeout.  As a fallback, also arm
        # from current state when we're already in a safety zone *and*
        # printing — this catches the case where the zone-entry edge
        # fired before _print_stats flipped to "printing" (e.g. during
        # the prime line) and was then cleared by the not-printing
        # branch of _control_timer_cb.  The fresh timestamp is critical:
        # the timer counts only post-print-start time, exactly as
        # be70969 intended.
        if zone in (ZONE_EMPTY, ZONE_FULL):
            in_active_recovery = self._extreme_recovery_active == zone
            if entered and not in_active_recovery:
                # Fresh entry to a safety zone — start the cumulative
                # clock.  Skip during active same-zone recovery so a
                # bounce out and back into the zone (e.g. empty <->
                # empty_middle while chunks pump) doesn't reset the
                # cap that's already counting from recovery_started_at.
                self._safety_zone_start = eventtime
            elif (self._safety_zone_start <= 0.0
                    and self._is_printing()
                    and not in_active_recovery):
                self._safety_zone_start = eventtime
        else:
            # Only clear when no recovery is in flight — empty <->
            # empty_middle bouncing during recovery should not reset
            # the cumulative cap (the recovery's per-attempt cap is
            # measured from _recovery_started_at independently).
            if self._extreme_recovery_active is None:
                self._safety_zone_start = 0.0

        # Clear initial fill once filament reaches middle or beyond
        if (self._initial_fill_until > 0.0
                and zone not in (ZONE_EMPTY, ZONE_EMPTY_MIDDLE)):
            self._initial_fill_until = 0.0
            self._cancel_fill_timer()
            self._sync("fill-complete")

        # Mid-recovery handling: detect exit conditions.  The recovery
        # check timer also polls for these; this branch covers the
        # sensor-callback path so we react immediately on the edge
        # rather than waiting for the next 50 ms poll.
        if self._extreme_recovery_active is not None:
            active_zone = self._extreme_recovery_active
            if active_zone == ZONE_EMPTY:
                left_band = zone not in (ZONE_EMPTY, ZONE_EMPTY_MIDDLE)
            else:  # ZONE_FULL
                left_band = zone not in (ZONE_FULL, ZONE_FULL_MIDDLE)
            if left_band:
                self._stop_recovery_vactual()
                self._exit_extreme_recovery(eventtime, "reached-middle")
            if self._extreme_recovery_active is not None:
                # Still recovering — leave _rd_multiplier alone.
                return

        # Extreme zones go through VACTUAL recovery, not multiplier.
        # Only enter recovery while actually printing — outside a print
        # the user may be loading/unloading or manually drawing
        # filament, and surprise active motion (EMPTY) or auto-drain
        # (FULL) is unwanted.  This matches the safety-timeout policy
        # which also only ticks while printing.
        #
        # The synced gate below is deliberate: recovery assumes the
        # sync-follow context.  Since the control timer now calls
        # _update_rotation_distance unconditionally, the auto-sync
        # above restores sync within one control_interval of any
        # unsync (material permitting), so recovery can enter on the
        # very tick that re-syncs — worst-case latency is one tick.
        if (zone in (ZONE_FULL, ZONE_EMPTY)
                and self._synced_to is not None
                and self._is_printing()):
            if zone == ZONE_EMPTY:
                self._enter_empty_recovery(eventtime)
            else:
                self._enter_full_recovery(eventtime)
            return

        # Non-extreme zones: apply the drift-gain multiplier as usual,
        # with 200 ms zone hysteresis to suppress sensor-bounce thrash.
        # Bake the hysteresis here (rather than in _apply_multiplier)
        # so that the multiplier itself is computed from a stable zone.
        if zone != self._proposed_zone:
            self._proposed_zone = zone
            self._proposed_zone_at = eventtime
            return
        if eventtime - self._proposed_zone_at < 0.2:
            return
        multiplier = self._zone_to_multiplier(zone)
        if self._synced_to is not None:
            self._apply_multiplier(multiplier)

    # --- Initial fill ---

    def _do_initial_fill(self, eventtime):
        """Start chunked forward feed to fill the buffer tube.

        Issues small move chunks via force_move, checking sensor state
        between each chunk.  The reactor processes sensor callbacks
        between chunks, so the fill aborts promptly when the middle
        sensor (or beyond) triggers.
        """
        if self.force_move is None:
            return
        self._unsync("initial-fill")
        self.state = STATE_FEEDING
        self.motor_direction = FORWARD
        next_wake = self._do_fill_chunk(eventtime)
        if next_wake == self.reactor.NEVER:
            return
        if self._fill_timer is None:
            self._fill_timer = self.reactor.register_timer(
                self._do_fill_chunk, next_wake)
        else:
            self.reactor.update_timer(self._fill_timer, next_wake)

    def _do_fill_chunk(self, eventtime):
        """Issue one fill chunk and return the next waketime, or
        NEVER if the loop should terminate.  Pre-sidecar pattern
        (commit 5d68d34): force_move.manual_move synchronously
        enqueues onto the main toolhead and pays the dwell cost.
        Outside a print (which is when this runs) the dwell is
        invisible.

        Serves as both the inline entry from _do_initial_fill and the
        registered timer callback.  See _do_continuous_chunk for why
        we return the next waketime instead of calling update_timer.
        """
        # Abort: timeout expired
        if (self._initial_fill_until > 0.0
                and eventtime >= self._initial_fill_until):
            self._initial_fill_until = 0.0
            self.motor_direction = STOP
            logging.info("buffer[%s]: initial fill timeout"
                         % self.short_name)
            self.gcode.respond_info(
                "Buffer: initial fill timed out after %.0fs"
                % self.initial_fill_timeout)
            self._sync("fill-timeout")
            return self.reactor.NEVER
        # Abort: filament reached middle sensor or beyond
        zone = self._compute_zone()
        if zone is not None and zone not in (ZONE_EMPTY, ZONE_EMPTY_MIDDLE):
            self._initial_fill_until = 0.0
            logging.info("buffer[%s]: initial fill complete (zone=%s)"
                         % (self.short_name, zone))
            self._sync("fill-complete")
            return self.reactor.NEVER
        # Abort: fill was cancelled (disable, runout, error)
        if self._initial_fill_until <= 0.0:
            return self.reactor.NEVER
        # Issue one chunk
        try:
            self.force_move.manual_move(
                self.extruder_stepper.stepper,
                self._manual_chunk_dist,
                self.manual_speed, self.manual_accel)
        except Exception as e:
            logging.warning("buffer[%s]: fill chunk failed: %s"
                            % (self.short_name, e))
            self._initial_fill_until = 0.0
            self._sync("fill-error")
            return self.reactor.NEVER
        return eventtime + self._chunk_move_duration(
            self._manual_chunk_dist, self.manual_speed, self.manual_accel)

    def _cancel_fill_timer(self):
        if self._fill_timer is not None:
            self.reactor.update_timer(
                self._fill_timer, self.reactor.NEVER)

    # --- Control timer ---

    def _control_timer_cb(self, eventtime):
        # Debug heartbeat — BEFORE the manual-state early returns so
        # it reports in every state.  margin is how far the queued
        # motion runs ahead of the buffer stepper's MCU clock; a
        # margin trending toward zero under load is the precursor of
        # a "Rescheduled timer in the past" shutdown.  Uses
        # toolhead.get_status (never get_last_move_time — that
        # flushes the lookahead) and the buffer stepper's own MCU,
        # which may be a secondary board.
        if (self.debug
                and eventtime - self._last_heartbeat_at
                >= _HEARTBEAT_INTERVAL):
            self._last_heartbeat_at = eventtime
            margin = -1.0
            try:
                mcu = self.extruder_stepper.stepper.get_mcu()
                margin = (self.toolhead.get_status(eventtime)
                          ["print_time"]
                          - mcu.estimated_print_time(eventtime))
            except Exception:
                pass
            self._dlog(eventtime,
                       "heartbeat zone=%s state=%s synced=%s "
                       "recovery=%s latch=%s material=%s margin=%.3f",
                       self._current_zone, self.state, self._synced_to,
                       self._extreme_recovery_active,
                       self._vactual_maybe_running,
                       self.material_present, margin)
        # Poll active extruder to detect tool changes.  The toolhead's
        # active extruder can change via ACTIVATE_EXTRUDER / T0/T1 macros
        # without firing a dedicated event, so we sample it each tick.
        if self.toolhead is not None:
            try:
                current_extruder = self.toolhead.get_extruder().get_name()
            except Exception:
                current_extruder = self._last_active_extruder
            if current_extruder != self._last_active_extruder:
                self._handle_extruder_change(current_extruder)
                self._last_active_extruder = current_extruder
        # Manual feed auto-stop on sustained full sensor.  Skip for
        # button-held feed (user's explicit intent — release to stop).
        # Preserves auto_enabled so the buffer returns to normal control
        # instead of silently disabling itself.
        if self.state == STATE_MANUAL_FEED:
            if (self.sensor_states["full"]
                    and not self._feed_button_pressed):
                if self._manual_feed_full_start == 0.0:
                    self._manual_feed_full_start = eventtime
                elif (eventtime - self._manual_feed_full_start
                      >= self.manual_feed_full_timeout):
                    self._stop_manual()
                    if self.auto_enabled:
                        self.state = STATE_STOPPED
                        self._sync("manual-feed-stop")
                    else:
                        self.state = STATE_IDLE
                    self._manual_feed_full_start = 0.0
                    self.gcode.respond_info(
                        "Buffer[%s]: manual feed stopped - full zone "
                        "sustained for %.0fs"
                        % (self.short_name,
                           self.manual_feed_full_timeout))
            else:
                self._manual_feed_full_start = 0.0
            return eventtime + self.control_interval
        # Manual retract auto-stop on empty sensor.  Skip for:
        #   * retract-until-clear (stops on material_present instead)
        #   * button-held retract (user's explicit intent — they can
        #     release the button to stop)
        # Also: this is a soft stop (back to STOPPED/IDLE) that preserves
        # auto_enabled.  The prior behavior of clearing auto_enabled meant
        # a button-held retract that brushed the empty sensor would
        # silently disable the buffer, requiring a manual BUFFER_ENABLE.
        if self.state == STATE_MANUAL_RETRACT:
            if (self.sensor_states["empty"]
                    and not self._retract_until_clear
                    and not self._retract_button_pressed):
                self._stop_manual()
                if self.auto_enabled:
                    self.state = STATE_STOPPED
                    self._sync("manual-retract-stop")
                else:
                    self.state = STATE_IDLE
                self.gcode.respond_info(
                    "Buffer[%s]: manual retract stopped - empty sensor"
                    % self.short_name)
            return eventtime + self.control_interval

        if not self.auto_enabled:
            return eventtime + self.control_interval

        if self.state in (STATE_MANUAL_RETRACT,
                          STATE_ERROR, STATE_DISABLED):
            return eventtime + self.control_interval

        # Persistent sensor conflict escalation.  _update_rotation_distance
        # defers a transient empty+full conflict (out-of-order callbacks
        # in one MCU report); if the conflict survives past
        # control_interval it's a real wiring fault and must error.
        if (self._conflict_since > 0.0
                and eventtime - self._conflict_since
                >= self.control_interval):
            self._conflict_since = 0.0
            self._handle_error(
                "Sensor conflict: empty and full both triggered")
            return eventtime + self.control_interval

        # Safety timeout check — only tick while printing.  When the
        # extruder is idle, the multiplier has no moves to act on, so
        # being in a safety zone is static, not worsening.  Clear the
        # start time so it can only be set by a zone-entry transition
        # that occurs during active printing (_update_rotation_distance
        # sets _safety_zone_start on 'entered').
        zone = self._compute_zone()
        if zone is not None and self._safety_zone_start > 0.0:
            if not self._is_printing():
                self._safety_zone_start = 0.0
            else:
                elapsed = eventtime - self._safety_zone_start
                if (zone == ZONE_EMPTY
                        and elapsed >= self.empty_safety_timeout):
                    self._handle_error(
                        "Buffer stuck in empty zone for %.0fs"
                        % self.empty_safety_timeout)
                    return eventtime + self.control_interval
                if (zone == ZONE_FULL
                        and elapsed >= self.full_safety_timeout):
                    self._do_safety_retract(eventtime)
                    return eventtime + self.control_interval

        # Periodic re-evaluation of rotation_distance.  Called
        # UNCONDITIONALLY — the auto-sync inside it is the only path
        # that restores sync after a safety retract or manual unsync,
        # and gating this call on being synced (the old behavior) made
        # re-sync depend entirely on a lucky sensor edge.  Upstream
        # guards in this timer already filter MANUAL_* (early
        # returns), not-auto-enabled, and ERROR/DISABLED, so the
        # reachable states here are IDLE, FEEDING, STOPPED and
        # RETRACTING; the auto-sync itself is additionally
        # material-gated.
        self._update_rotation_distance(eventtime)

        # Drain any rate-limit-deferred multiplier update.  Without
        # this, a multiplier change that was rate-limited away during
        # a print would linger until the next sensor edge.
        if (self._pending_multiplier is not None
                and eventtime - self._last_rd_apply_at >= 0.5):
            self._apply_multiplier(self._pending_multiplier)

        return eventtime + self.control_interval

    def _do_safety_retract(self, eventtime):
        """Forced retract when full zone times out.

        Unsyncs, issues a retract via force_move.manual_move, and arms
        the shared completion timer: when the move's duration elapses,
        _timed_move_done_cb restores RETRACTING -> STOPPED, and the
        control timer's unconditional _update_rotation_distance call
        then re-syncs (STOPPED -> FEEDING) within one
        control_interval.  This pays a brief toolhead dwell during a
        print — acceptable for a rare fault-recovery path.
        """
        logging.info("buffer[%s]: full zone safety retract"
                     % self.short_name)
        self.gcode.respond_info(
            "Buffer: full zone timeout - retracting")
        stepper = self.extruder_stepper.stepper
        self._unsync("safety-retract")
        self.state = STATE_RETRACTING
        self.motor_direction = BACK
        # Armed before the move so a raising manual_move still exits
        # RETRACTING instead of leaving it terminal.
        self._arm_move_done_timer(
            eventtime + self._chunk_move_duration(
                self._manual_chunk_dist, self.manual_speed,
                self.manual_accel))
        try:
            self.force_move.manual_move(
                stepper, -self._manual_chunk_dist, self.manual_speed,
                self.manual_accel)
        except Exception as e:
            logging.warning("buffer[%s]: safety retract failed: %s"
                            % (self.short_name, e))
            return
        self._safety_zone_start = 0.0

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
            self._start_continuous_feed(FORWARD, self.manual_speed)
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
            self._start_continuous_feed(BACK, self.manual_speed)
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
            self._unsync("buttons-disable")
            self.state = STATE_IDLE
        else:
            self.auto_enabled = True
            self.state = STATE_STOPPED
            self._sync("buttons-enable")
        self.gcode.respond_info(
            "Buffer: %s via buttons"
            % ("enabled" if self.auto_enabled else "disabled"))

    def _button_release_restore(self):
        """Restore state after manual button release."""
        if self.auto_enabled:
            self.state = STATE_STOPPED
            self._sync("button-release")
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
            if self._clear_error():
                self.gcode.respond_info(
                    "Buffer: error cleared via buttons")
            else:
                self.gcode.respond_info(
                    "Buffer: error NOT cleared - %s" % self.error_msg)
        self._error_clear_hold_start = 0.0
        return self.reactor.NEVER

    # --- Manual feed/retract ---

    def _start_continuous_feed(self, direction, speed):
        """Feed/retract continuously via chunked moves until stopped.

        Stopped by: full sensor auto-stop (feed), empty sensor auto-stop
        (retract), BUFFER_STOP, or button release.
        """
        self._cancel_fill()
        # A pending fixed-move completion shares the MANUAL_* states,
        # so the state guard alone would NOT protect this session —
        # a stale fire would kill a button-held feed mid-hold.
        self._cancel_move_done_timer()
        self._unsync("manual-continuous")
        if direction == FORWARD:
            self.state = STATE_MANUAL_FEED
        else:
            self.state = STATE_MANUAL_RETRACT
        self.motor_direction = direction
        self._manual_feed_full_start = 0.0
        self._continuous_feed_direction = direction
        self._continuous_feed_speed = speed
        eventtime = self.reactor.monotonic()
        next_wake = self._do_continuous_chunk(eventtime)
        if next_wake == self.reactor.NEVER:
            return
        if self._continuous_timer is None:
            self._continuous_timer = self.reactor.register_timer(
                self._do_continuous_chunk, next_wake)
        else:
            self.reactor.update_timer(
                self._continuous_timer, next_wake)

    def _do_continuous_chunk(self, eventtime):
        """Issue one chunk of a continuous feed and return the next
        waketime, or NEVER if the loop should terminate.

        Serves as both the inline entry from _start_continuous_feed and
        the registered timer callback.  Returning the next waketime
        (rather than calling update_timer + returning NEVER) is the
        only pattern that re-arms the timer in production Klipper:
        update_timer is a no-op while a timer's callback is running,
        and _check_timers unconditionally assigns the return value to
        the timer's waketime.
        """
        # Abort if state changed (stop, disable, error, button release)
        if self.state not in (STATE_MANUAL_FEED, STATE_MANUAL_RETRACT):
            return self.reactor.NEVER
        if self.force_move is None:
            return self.reactor.NEVER
        stepper = self.extruder_stepper.stepper
        dist = self._manual_chunk_dist
        if self._continuous_feed_direction == BACK:
            dist = -dist
        try:
            self.force_move.manual_move(
                stepper, dist, self._continuous_feed_speed,
                self.manual_accel)
        except Exception as e:
            logging.warning("buffer[%s]: continuous feed chunk failed: %s"
                            % (self.short_name, e))
            return self.reactor.NEVER
        return eventtime + self._chunk_move_duration(
            dist, self._continuous_feed_speed, self.manual_accel)

    def _cancel_continuous_timer(self):
        if self._continuous_timer is not None:
            self.reactor.update_timer(
                self._continuous_timer, self.reactor.NEVER)

    def _stop_manual(self):
        """Stop any pending manual move state.  Cancels the timers
        explicitly to prevent the next scheduled chunk (or a pending
        fixed-move completion) from firing with stale state — the
        state guards in the callbacks are belt-and-suspenders on top
        of this."""
        self.motor_direction = STOP
        self._manual_feed_full_start = 0.0
        self._cancel_continuous_timer()
        self._cancel_move_done_timer()

    # --- Timed-move completion ---

    def _arm_move_done_timer(self, wake):
        if self._move_done_timer is None:
            self._move_done_timer = self.reactor.register_timer(
                self._timed_move_done_cb, wake)
        else:
            self.reactor.update_timer(self._move_done_timer, wake)

    def _cancel_move_done_timer(self):
        if self._move_done_timer is not None:
            self.reactor.update_timer(
                self._move_done_timer, self.reactor.NEVER)

    def _timed_move_done_cb(self, eventtime):
        """One-shot completion for fixed-distance manual moves and the
        safety retract.  Restores the state machine when the move's
        real-time duration has elapsed; any takeover that changed
        state in the meantime (error, disable, runout, a fresh
        continuous feed) makes this a no-op via the state guard.

        When auto-enabled and still unsynced, deliberately do NOT call
        _sync() here — the control timer's unconditional
        _update_rotation_distance call re-syncs within one
        control_interval, with the material gate applied.  When a
        control tick already re-synced mid-move (possible when
        control_interval < move duration), promote directly to
        FEEDING: _sync() only promotes on the STOPPED->synced edge,
        so without this the buffer would sit synced-but-STOPPED
        indefinitely."""
        if self.state not in (STATE_MANUAL_FEED, STATE_MANUAL_RETRACT,
                              STATE_RETRACTING):
            return self.reactor.NEVER
        self.motor_direction = STOP
        if not self.auto_enabled:
            self.state = STATE_IDLE
            return self.reactor.NEVER
        self.state = STATE_STOPPED
        if self._synced_to is not None:
            self.state = STATE_FEEDING
            self.motor_direction = FORWARD
        return self.reactor.NEVER

    # --- Error handling ---

    def _handle_error(self, msg):
        self._cancel_fill()
        self._unsync("error")
        self.state = STATE_ERROR
        self.error_msg = msg
        self.motor_direction = STOP
        logging.error("buffer[%s]: %s" % (self.short_name, msg))
        self.gcode.respond_info("Buffer ERROR: %s" % msg)
        if self.pause_on_error:
            self._trigger_pause(msg)

    def _clear_error(self):
        """Clear error state.  Returns False — and stays in ERROR —
        when the chip-side VACTUAL state cannot be confirmed cleared,
        for the same reason cmd_BUFFER_ENABLE refuses: leaving ERROR
        re-arms sync and the driver, and a stale nonzero VACTUAL would
        resume motion the moment the driver re-enables."""
        self._driver_disable_requested = False
        if (self._vactual_maybe_running
                and not self._stop_recovery_vactual()):
            self.error_msg = ("VACTUAL could not be cleared; "
                              "TMC UART unresponsive")
            return False
        self.state = STATE_STOPPED if self.auto_enabled else STATE_IDLE
        self.error_msg = ""
        self._safety_zone_start = 0.0
        if self.auto_enabled:
            self._sync("error-cleared")
        return True

    def _trigger_pause(self, msg):
        try:
            pause_resume = self.printer.lookup_object("pause_resume")
            if not pause_resume.is_paused:
                self.gcode.respond_info("Buffer: pausing print - %s" % msg)
                self.gcode.run_script("PAUSE")
        except Exception:
            logging.warning("buffer[%s]: failed to trigger pause"
                            % self.short_name)

    # --- Status ---

    def get_status(self, eventtime):
        return {
            "name": self.short_name,
            "bound_extruder": self.extruder_name,
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
            "manual_speed": self.manual_speed,
            "recovery_speed": self.recovery_speed,
            "start_drain_speed": self.start_drain_speed,
            "manual_move_distance": self.manual_move_distance,
            "drift_gain": self.drift_gain,
            "extreme_recovery_active": self._extreme_recovery_active,
            "vactual_maybe_running": self._vactual_maybe_running,
            "driver_disable_requested": self._driver_disable_requested,
            "is_printing": self._is_printing(),
            "manual_feed_full_timeout": self.manual_feed_full_timeout,
            "pause_on_runout": self.pause_on_runout,
            "pause_on_error": self.pause_on_error,
        }

    def _is_printing(self):
        # Safe default: without print_stats (no [virtual_sdcard]) we
        # cannot know a print is running, so treat the machine as not
        # printing — no surprise recovery motion, normalization, or
        # safety-timeout escalation on such setups.  Drift-gain
        # feedback is unaffected (it doesn't gate on printing).
        if self._print_stats is None:
            return False
        return (self._print_stats.get_status(
            self.reactor.monotonic())["state"] == "printing")

    # --- Gcode commands ---

    def cmd_BUFFER_STATUS(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        msg = (
            "Buffer status [%s]:\n"
            "  Extruder binding: %s\n"
            "  State: %s\n"
            "  Motor: %s\n"
            "  Sensors: empty=%s middle=%s full=%s\n"
            "  Material present: %s\n"
            "  Auto enabled: %s\n"
            "  Zone: %s (prev: %s)\n"
            "  RD multiplier: %.4f (base: %.4f)\n"
            "  Synced to: %s\n"
            "  Manual speed: %.1f mm/s  chunk: %.1f mm\n"
            "  Drift gain: %.3f\n"
            "  Recovery active: %s\n"
            "  Printing: %s"
            % (
                status["name"],
                status["bound_extruder"] or "(any active)",
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
                status["manual_move_distance"],
                status["drift_gain"],
                status["extreme_recovery_active"] or "none",
                status["is_printing"],
            )
        )
        if status["error"]:
            msg += "\n  ERROR: %s" % status["error"]
        gcmd.respond_info(msg)

    def cmd_BUFFER_ENABLE(self, gcmd):
        # No-op while extreme-zone recovery is driving the motor.
        # Recovery implies enabled-and-synced already, and the latch
        # consumption below would otherwise write the stop sequence —
        # silently killing the recovery push while the poller stays
        # armed, which manifests as a spurious "recovery exceeded"
        # error a few seconds later.
        if self._extreme_recovery_active is not None:
            gcmd.respond_info(
                "Buffer: already enabled (extreme-zone recovery in "
                "progress)")
            return
        # Consume the VACTUAL latch before arming anything.  If a prior
        # cleanup failed (or a prior session died mid-recovery), the
        # chip still holds a nonzero VACTUAL and will resume motion the
        # moment the driver re-enables — refuse until a confirmed zero
        # lands.  Re-arm the one-shot fallback first so this attempt
        # gets its own fresh try at the bus.
        self._driver_disable_requested = False
        if (self._vactual_maybe_running
                and not self._stop_recovery_vactual()):
            gcmd.respond_info(
                "Buffer: NOT enabled - VACTUAL could not be cleared "
                "(TMC UART unresponsive). Fix the connection and retry "
                "BUFFER_ENABLE.")
            return
        self.auto_enabled = True
        self.state = STATE_STOPPED
        self.error_msg = ""
        self._safety_zone_start = 0.0
        self._sync("enable")
        # The recommended START_PRINT flow disables the buffer during
        # heat/level and re-enables right before the prime line — for
        # file prints print_stats is already "printing" there, so this
        # hook (not the start event) is the one that normalizes a
        # loaded-full arm in practice.
        if self._synced_to is not None:
            self._maybe_start_print_normalization(
                self.reactor.monotonic())
        gcmd.respond_info("Buffer: automatic control enabled")

    def cmd_BUFFER_DISABLE(self, gcmd):
        self.auto_enabled = False
        self._cancel_fill()
        self._unsync("disable")
        self.state = STATE_DISABLED
        gcmd.respond_info("Buffer: automatic control disabled")

    def cmd_BUFFER_FEED(self, gcmd):
        if self.state == STATE_ERROR:
            gcmd.respond_info(
                "Buffer: cannot feed while in error state. "
                "Run BUFFER_CLEAR_ERROR first.")
            return
        speed = gcmd.get_float("SPEED", self.manual_speed, above=0.0)
        dist = gcmd.get_float("DIST", 0.0, minval=0.0)
        if dist > 0.0:
            # Fixed distance feed
            self._cancel_fill()
            self._unsync("manual-feed")
            self.state = STATE_MANUAL_FEED
            self.motor_direction = FORWARD
            self._manual_feed_full_start = 0.0
            try:
                self.force_move.manual_move(
                    self.extruder_stepper.stepper,
                    dist, speed, self.manual_accel)
            except Exception as e:
                logging.warning("buffer[%s]: BUFFER_FEED failed: %s"
                                % (self.short_name, e))
            # Armed even when manual_move raised — the completion
            # restores STOPPED/IDLE either way instead of stranding
            # the state machine in MANUAL_FEED forever.
            self._arm_move_done_timer(
                self.reactor.monotonic()
                + self._chunk_move_duration(dist, speed,
                                            self.manual_accel))
            gcmd.respond_info(
                "Buffer: feeding %.0fmm at %.1f mm/s" % (dist, speed))
        else:
            # Continuous feed until full sensor auto-stop
            self._start_continuous_feed(FORWARD, speed)
            gcmd.respond_info(
                "Buffer: feeding at %.1f mm/s until full sensor" % speed)

    def cmd_BUFFER_RETRACT(self, gcmd):
        if self.state == STATE_ERROR:
            gcmd.respond_info(
                "Buffer: cannot retract while in error state. "
                "Run BUFFER_CLEAR_ERROR first.")
            return
        speed = gcmd.get_float("SPEED", self.manual_speed, above=0.0)
        dist = gcmd.get_float("DIST", 0.0, minval=0.0)
        if dist > 0.0:
            # Fixed distance retract
            self._cancel_fill()
            self._unsync("manual-retract")
            self.state = STATE_MANUAL_RETRACT
            self.motor_direction = BACK
            try:
                self.force_move.manual_move(
                    self.extruder_stepper.stepper,
                    -dist, speed, self.manual_accel)
            except Exception as e:
                logging.warning("buffer[%s]: BUFFER_RETRACT failed: %s"
                                % (self.short_name, e))
            # See cmd_BUFFER_FEED: completion restores STOPPED/IDLE.
            self._arm_move_done_timer(
                self.reactor.monotonic()
                + self._chunk_move_duration(dist, speed,
                                            self.manual_accel))
            gcmd.respond_info(
                "Buffer: retracting %.0fmm at %.1f mm/s" % (dist, speed))
        else:
            # Continuous retract until empty sensor auto-stop
            self._start_continuous_feed(BACK, speed)
            gcmd.respond_info(
                "Buffer: retracting at %.1f mm/s until empty sensor"
                % speed)

    def cmd_BUFFER_RETRACT_UNTIL_CLEAR(self, gcmd):
        """Retract continuously until the filament presence switch clears."""
        if self.state == STATE_ERROR:
            gcmd.respond_info(
                "Buffer: cannot retract while in error state. "
                "Run BUFFER_CLEAR_ERROR first.")
            return
        speed = gcmd.get_float("SPEED", self.manual_speed, above=0.0)
        self._cancel_fill()
        # Same shared-state hazard as _start_continuous_feed: this
        # loop lives in MANUAL_RETRACT too.
        self._cancel_move_done_timer()
        self._unsync("retract-until-clear")
        self.state = STATE_MANUAL_RETRACT
        self.motor_direction = BACK
        self._continuous_feed_direction = BACK
        self._continuous_feed_speed = speed
        self._retract_until_clear = True
        eventtime = self.reactor.monotonic()
        next_wake = self._do_retract_until_clear_chunk(eventtime)
        if next_wake != self.reactor.NEVER:
            if self._retract_clear_timer is None:
                self._retract_clear_timer = self.reactor.register_timer(
                    self._do_retract_until_clear_chunk, next_wake)
            else:
                self.reactor.update_timer(
                    self._retract_clear_timer, next_wake)
        gcmd.respond_info(
            "Buffer: retracting at %.1f mm/s until filament clears" % speed)

    def _do_retract_until_clear_chunk(self, eventtime):
        """Issue one retract chunk and return the next waketime, or
        NEVER if the loop should terminate.  Serves as both the inline
        entry from cmd_BUFFER_RETRACT_UNTIL_CLEAR and the registered
        timer callback.  See _do_continuous_chunk for why we return
        the next waketime instead of calling update_timer."""
        if not self._retract_until_clear:
            return self.reactor.NEVER
        if self.state != STATE_MANUAL_RETRACT:
            self._retract_until_clear = False
            return self.reactor.NEVER
        if not self.material_present:
            # Filament has cleared the switch
            self._retract_until_clear = False
            self._stop_manual()
            self.state = STATE_IDLE
            self.auto_enabled = False
            self.gcode.respond_info(
                "Buffer: retract complete - filament cleared")
            return self.reactor.NEVER
        if self.force_move is None:
            self._retract_until_clear = False
            return self.reactor.NEVER
        try:
            self.force_move.manual_move(
                self.extruder_stepper.stepper,
                -self._manual_chunk_dist, self._continuous_feed_speed,
                self.manual_accel)
        except Exception as e:
            logging.warning(
                "buffer[%s]: retract_until_clear chunk failed: %s"
                % (self.short_name, e))
            self._retract_until_clear = False
            return self.reactor.NEVER
        return eventtime + self._chunk_move_duration(
            self._manual_chunk_dist, self._continuous_feed_speed,
            self.manual_accel)

    def _cancel_retract_clear_timer(self):
        if self._retract_clear_timer is not None:
            self.reactor.update_timer(
                self._retract_clear_timer, self.reactor.NEVER)

    def cmd_BUFFER_STOP(self, gcmd):
        self._cancel_fill()
        self._retract_until_clear = False
        self._cancel_retract_clear_timer()
        self._stop_manual()
        if self.auto_enabled:
            self.state = STATE_STOPPED
            self._sync("stop")
        else:
            self.state = STATE_IDLE
        gcmd.respond_info("Buffer: stopped")

    def cmd_BUFFER_SET_SPEED(self, gcmd):
        speed = gcmd.get_float("SPEED", above=0.0)
        self.manual_speed = speed
        gcmd.respond_info("Buffer: manual speed set to %.1f mm/s" % speed)

    def cmd_BUFFER_CLEAR_ERROR(self, gcmd):
        if self.state == STATE_ERROR:
            if self._clear_error():
                gcmd.respond_info("Buffer: error cleared")
            else:
                gcmd.respond_info(
                    "Buffer: error NOT cleared - %s" % self.error_msg)
        else:
            gcmd.respond_info("Buffer: no error to clear")


def load_config(config):
    # Handles the bare [buffer] section.
    return Buffer(config)


def load_config_prefix(config):
    # Handles named sections like [buffer my_buf] for multi-buffer setups.
    return Buffer(config)
