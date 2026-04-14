# LLL-Plus Klipper Plugin

A Klipper extras module for real-time control of the Mellow LLL Plus filament buffer. Uses the AFC/Happy Hare motion strategy: the buffer stepper is synchronized with the main extruder via a shared trapq, and hall effect sensors dynamically adjust `rotation_distance` to keep the filament loop centered on the middle sensor.

## Features

- **Trapq sync** -- buffer stepper follows the extruder step-for-step via `extruder_stepper` and shared motion queue
- **Middle-zone seeking** -- three hall sensors define a five-zone state; rotation_distance feedback keeps the filament centered on the middle sensor (dead-band at multiplier 1.0)
- **Fault escalation** -- sustained time in a safety zone (EMPTY/FULL) increases the correction gain, then triggers an error or forced retract
- **Initial fill** -- continuous forward feed on first filament insertion via `force_move`
- **Manual override** -- physical feed/retract buttons and GCode commands for filament loading
- **Error clear via buttons** -- hold both buttons for 2 seconds to clear an error state
- **Error protection** -- forward timeout, sensor conflict detection, safety timeouts, and optional pause-on-runout

## Hardware

- Mellow LLL Plus filament buffer board (STM32F072 MCU)
- TMC2208 / TMC2225 stepper driver (step/dir control; UART used only for initial config)
- Three hall effect sensors (empty, middle, full positions)
- Filament presence switch
- Feed and retract buttons

## Flashing

There is a good guide for flashing and calibrating the buffer with klipper [Here](https://github.com/ss1gohan13/BufferPLUS-klipper)

## Installation

```bash
cd ~/lll-plus-klipper-plugin/klipper
./install.sh
```

The script symlinks `buffer.py` into `~/klipper/klippy/extras/`. Restart Klipper afterward:

```bash
sudo systemctl restart klipper
```

## Macro Integration

Add `BUFFER_ENABLE` and `BUFFER_DISABLE` to your printer macros so the buffer activates during prints and deactivates for maintenance operations.

### Print start / end

```ini
[gcode_macro PRINT_START]
gcode:
    # ... homing, heating, bed mesh, etc.
    BUFFER_ENABLE
    # ... purge line, start printing

[gcode_macro PRINT_END]
gcode:
    BUFFER_DISABLE
    # ... retract, park, cooldown, etc.

[gcode_macro CANCEL_PRINT]
rename_existing: BASE_CANCEL_PRINT
gcode:
    BUFFER_DISABLE
    BASE_CANCEL_PRINT
```

### Pause / resume

The buffer automatically pauses the print on filament runout (`pause_on_runout: True`). If you use custom pause/resume macros, disable the buffer on pause and re-enable on resume so it doesn't fight the parked extruder:

```ini
[gcode_macro PAUSE]
rename_existing: BASE_PAUSE
gcode:
    BUFFER_DISABLE
    BASE_PAUSE

[gcode_macro RESUME]
rename_existing: BASE_RESUME
gcode:
    BUFFER_ENABLE
    BASE_RESUME
```

### Filament change

When the buffer is synced, it follows extruder moves automatically -- `G1 E-50` to retract from the hotend makes the buffer stepper retract in lockstep. No need to disable for normal load/unload. `BUFFER_FEED` / `BUFFER_RETRACT` are for threading filament *through the buffer tube itself* (spool to extruder entrance) when the extruder is not involved. With no `DIST` argument, `BUFFER_FEED` runs continuously until the full sensor triggers and `BUFFER_RETRACT` runs until the empty sensor triggers:

```ini
[gcode_macro UNLOAD_FILAMENT]
gcode:
    BUFFER_DISABLE
    G1 E-50 F600              ; retract from hotend
    BUFFER_RETRACT ; retracts until buffer enters the empty state
 #   BUFFER_RETRACT_UNTIL_CLEAR ; optionally, retract until filament leaves the tube

[gcode_macro LOAD_FILAMENT]
gcode:
    BUFFER_FEED                ; thread filament through tube until full sensor
    G1 E50 F300               ; push into hotend
    BUFFER_ENABLE
```

### Notes

- `BUFFER_ENABLE` syncs the buffer stepper to the extruder. Call it after heating and before the first extrusion move.
- `BUFFER_DISABLE` unsyncs and stops the motor. Always call it before parking, tool changes, or filament operations.
- `BUFFER_FEED` and `BUFFER_RETRACT` accept `SPEED=` (mm/s) and `DIST=` (mm) parameters for manual moves.
- The physical feed/retract buttons work independently of these macros and can be used any time (except in error state).
- Hold both buttons for 2 seconds to clear an error without needing a console.

## Configuration

Copy `sample_config/lll-plus.cfg` into your Klipper config directory and adjust pin assignments and serial path for your setup.

**Important:** The buffer stepper is now configured as `[extruder_stepper buffer_stepper]` (not `[manual_stepper]`). The `extruder:` field must name the extruder to sync with (typically `extruder`).

### Key parameters

| Parameter              | Default | Description                                                      |
|------------------------|---------|------------------------------------------------------------------|
| `drift_gain`           | 0.02    | Multiplier offset in EMPTY_MIDDLE / FULL_MIDDLE zones            |
| `multiplier_low`       | 0.9     | rd_multiplier in FULL zone (under-feed). AFC default              |
| `multiplier_high`      | 1.1     | rd_multiplier in EMPTY zone (over-feed). AFC default              |
| `fault_multiplier_low` | 0.36    | rd_multiplier in FULL after fault_escalation_time (AFC formula)   |
| `fault_multiplier_high`| 1.65    | rd_multiplier in EMPTY after fault_escalation_time (AFC formula)  |
| `fault_escalation_time`| 5.0     | Seconds in a safety zone before jumping to fault_multiplier_*    |
| `empty_safety_timeout` | 30.0    | Seconds in EMPTY zone before raising an error                    |
| `full_safety_timeout`  | 10.0    | Seconds in FULL zone before forced retract                       |
| `manual_speed`         | 15.0    | Speed (mm/s) for manual feed/retract                             |
| `manual_accel`         | 100.0   | Acceleration (mm/s^2) for manual feed/retract                    |
| `manual_move_distance` | 10.0    | Distance (mm) per manual / safety-retract chunk                  |
| `error_clear_hold_time`| 2.0     | Seconds both buttons must be held to clear error                 |
| `initial_fill_timeout` | 10.0    | Duration (s) of forward feed on first filament insertion         |
| `manual_feed_full_timeout` | 3.0 | Seconds full sensor must hold before auto-stopping manual feed   |
| `pause_on_runout`      | True    | Pause print on filament runout or safety timeout                 |
| `control_interval`     | 0.5     | Reactor timer interval (s) for safety timeout checks             |
| `debug`                | False   | Enable debug logging of zone transitions and multiplier changes  |

See `sample_config/lll-plus.cfg` for the full annotated reference.

## GCode Commands

| Command                              | Description                                                  |
|--------------------------------------|--------------------------------------------------------------|
| `BUFFER_STATUS`                      | Report state, sensors, zone, rd_multiplier, sync status      |
| `BUFFER_ENABLE`                      | Sync to extruder and enable automatic control                |
| `BUFFER_DISABLE`                     | Unsync and disable automatic control                         |
| `BUFFER_FEED [SPEED=<mm/s>] [DIST=<mm>]` | Forward feed. No DIST = feed until full sensor stops it |
| `BUFFER_RETRACT [SPEED=<mm/s>] [DIST=<mm>]` | Retract. No DIST = retract until empty sensor stops it |
| `BUFFER_RETRACT_UNTIL_CLEAR [SPEED=<mm/s>]` | Retract until filament presence switch clears          |
| `BUFFER_STOP`                        | Stop any manual move, re-sync if auto-enabled                |
| `BUFFER_SET_SPEED SPEED=<mm/s>`      | Set manual feed/retract speed                                |
| `BUFFER_CLEAR_ERROR`                 | Clear error state (also via 2s both-button hold)             |

## Motion Strategy

### How it works

The buffer stepper is configured as a Klipper `[extruder_stepper]` and synced to the main extruder's motion queue (trapq) at startup. This means the buffer motor follows the extruder step-for-step through Klipper's standard motion pipeline -- no G-code hooks, no velocity prediction, no timing gap.

Three hall effect sensors provide reactive feedback by adjusting the buffer stepper's `rotation_distance`:

```
rd_new = base_rotation_distance / multiplier
```

- **multiplier > 1.0** -- smaller rotation_distance -- more steps per mm -- delivers MORE filament
- **multiplier < 1.0** -- larger rotation_distance -- fewer steps per mm -- delivers LESS filament
- **multiplier = 1.0** -- baseline -- buffer follows extruder 1:1 (dead-band)

### Sensor Zones

| Empty | Middle | Full | Zone         | Multiplier                |
|:-----:|:------:|:----:|--------------|---------------------------|
|   1   |   *    |   1  | **ERROR**    | sensor conflict           |
|   1   |   *    |   0  | EMPTY        | `multiplier_high` (1.1)   |
|   0   |   0    |   0  | EMPTY_MIDDLE | 1.0 + `drift_gain`        |
|   0   |   1    |   0  | **MIDDLE**   | **1.00 (dead-band)**      |
|   0   |   1    |   1  | FULL_MIDDLE  | 1.0 - `drift_gain`        |
|   0   |   0    |   1  | FULL         | `multiplier_low` (0.9)    |

The MIDDLE zone is the target equilibrium. When the middle sensor alone is active, the multiplier is exactly 1.0 -- the buffer rides the extruder step-for-step with zero correction. Deviations toward empty or full apply proportional corrections that push the filament loop back toward center.

### Fault escalation

If a safety zone (EMPTY or FULL) persists for `fault_escalation_time` seconds, the multiplier jumps to the absolute `fault_multiplier_low` / `fault_multiplier_high` value (AFC defaults: 0.36 / 1.65 — strong corrections that drain or fill the loop quickly). If it persists past the safety timeout, an error is raised (EMPTY) or a forced retract is executed (FULL).

### State Transitions

| From               | To             | Trigger                                          |
|--------------------|----------------|--------------------------------------------------|
| DISABLED           | IDLE           | `klippy:ready`                                   |
| IDLE               | FEEDING        | `BUFFER_ENABLE` or filament inserted              |
| IDLE               | DISABLED       | `BUFFER_DISABLE`                                 |
| FEEDING            | IDLE           | Filament runout                                   |
| any (not ERROR)    | MANUAL_FEED    | `BUFFER_FEED` or feed button press               |
| any (not ERROR)    | MANUAL_RETRACT | `BUFFER_RETRACT` or retract button press         |
| MANUAL_*           | STOPPED/IDLE   | Button release                                   |
| any (not ERROR)    | STOPPED/IDLE   | Both buttons pressed (toggles auto-enable)       |
| any                | ERROR          | Sensor conflict, forward timeout, safety timeout |
| ERROR              | STOPPED/IDLE   | `BUFFER_CLEAR_ERROR` or 2s both-button hold      |
| any                | DISABLED       | `BUFFER_DISABLE` or `klippy:shutdown`            |

## Multi-Buffer / Multi-Extruder Support

Multi-extruder printers can declare a separate buffer per extruder. Each buffer is fully independent — its own sensors, stepper, state machine, and rotation_distance feedback loop.

### Named config sections

Use `[buffer <name>]` to declare additional buffers alongside (or instead of) the bare `[buffer]` section:

```ini
[buffer buffer_t0]
stepper: buffer_stepper_t0
extruder: extruder
sensor_empty_pin: ...
...

[buffer buffer_t1]
stepper: buffer_stepper_t1
extruder: extruder1
sensor_empty_pin: ...
...
```

### Optional `extruder` parameter

| `extruder` setting | Behavior |
|--------------------|----------|
| Set (e.g. `extruder: extruder1`) | **Bound.** Buffer only syncs while `extruder1` is the active extruder. On tool change: auto-syncs when its extruder becomes active, auto-unsyncs when another extruder becomes active. |
| Omitted | **Unbound.** Buffer follows whatever extruder is currently active; re-syncs to the new extruder on every tool change. This is the default single-buffer behavior. |

Tool changes are detected by polling the active extruder in the control timer (every `control_interval`, default 0.5s), so any tool-change mechanism (`ACTIVATE_EXTRUDER`, `T0`/`T1` macros, etc.) works without additional configuration.

Multiple unbound buffers are allowed but log a warning at startup — two unbound buffers will both follow the same active extruder, which is usually a misconfiguration.

### Targeting buffers in GCode

All `BUFFER_*` commands accept an optional `BUFFER=<name>` parameter to select a specific buffer:

```
BUFFER_STATUS BUFFER=buffer_t0
BUFFER_ENABLE BUFFER=buffer_t1
BUFFER_FEED BUFFER=buffer_t0 DIST=10
```

| Configuration | `BUFFER_STATUS` alone | `BUFFER_STATUS BUFFER=name` |
|--------------|-----------------------|-----------------------------|
| Only `[buffer]` | Targets `[buffer]` | `BUFFER=buffer` targets `[buffer]` |
| `[buffer]` + `[buffer other]` | Targets `[buffer]` (bare section is the default) | `BUFFER=other` targets the named buffer |
| Only `[buffer t0]` + `[buffer t1]` (no bare `[buffer]`) | Error: `BUFFER` param required | `BUFFER=t0` targets the named buffer |

Existing single-buffer setups are fully backward compatible — `[buffer]` with no `extruder` param behaves exactly as before, and every `BUFFER_*` command works without a `BUFFER=` argument.

### Status fields

Each buffer registers as a separate Klipper object (`buffer`, `buffer buffer_t0`, etc.) for Moonraker/Mainsail/Fluidd. The status dict now includes `name` and `extruder` fields so UIs can distinguish buffers.

## Tests

```bash
pytest tests/
```

The test suite uses pure mock objects with no Klipper dependency. Coverage includes rotation_distance feedback, zone classification, dead-band invariant, fault escalation, safety timeouts, manual control, button handling, error conditions, and state transitions.


## Credits
[@ss1gohan13](https://github.com/ss1gohan13) for his inital work with a macro version and execlent flashing instrucations.

Hardware and original firmware by [Mellow 3D](https://github.com/mellow-3d).

Armoured Turtle [AFC](https://www.armoredturtle.xyz/docs/afc-klipper-add-on/index.html) - Used for ideas when researching speed matching strategies.


## License

GPLv3 -- see [LICENSE](LICENSE).
