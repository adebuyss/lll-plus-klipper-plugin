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

## Installation

```bash
cd ~/lll-plus-klipper-plugin/klipper
./install.sh
```

The script symlinks `buffer.py` into `~/klipper/klippy/extras/`. Restart Klipper afterward:

```bash
sudo systemctl restart klipper
```

## Configuration

Copy `sample_config/lll-plus.cfg` into your Klipper config directory and adjust pin assignments and serial path for your setup.

**Important:** The buffer stepper is now configured as `[extruder_stepper buffer_stepper]` (not `[manual_stepper]`). The `extruder:` field must name the extruder to sync with (typically `extruder`).

### Key parameters

| Parameter              | Default | Description                                                      |
|------------------------|---------|------------------------------------------------------------------|
| `drift_gain`           | 0.02    | Multiplier offset in EMPTY_MIDDLE / FULL_MIDDLE zones            |
| `safety_gain`          | 0.05    | Multiplier offset in EMPTY / FULL safety zones                   |
| `fault_escalation_time`| 5.0     | Seconds in a safety zone before gain escalates to 1.5x           |
| `empty_safety_timeout` | 30.0    | Seconds in EMPTY zone before raising an error                    |
| `full_safety_timeout`  | 10.0    | Seconds in FULL zone before forced retract                       |
| `manual_speed`         | 10.0    | Speed (mm/s) for manual feed/retract                             |
| `manual_accel`         | 100.0   | Acceleration (mm/s^2) for manual feed/retract                    |
| `error_clear_hold_time`| 2.0     | Seconds both buttons must be held to clear error                 |
| `forward_timeout`      | 60.0    | Seconds of continuous forward before error (0 = disable)         |
| `initial_fill_timeout` | 10.0    | Duration (s) of forward feed on first filament insertion         |
| `pause_on_runout`      | True    | Pause print on filament runout or safety timeout                 |

See `sample_config/lll-plus.cfg` for the full annotated reference.

## GCode Commands

| Command                              | Description                                                  |
|--------------------------------------|--------------------------------------------------------------|
| `BUFFER_STATUS`                      | Report state, sensors, zone, rd_multiplier, sync status      |
| `BUFFER_ENABLE`                      | Sync to extruder and enable automatic control                |
| `BUFFER_DISABLE`                     | Unsync and disable automatic control                         |
| `BUFFER_FEED [SPEED=<mm/s>] [DIST=<mm>]` | Manual forward feed (default 50mm)                      |
| `BUFFER_RETRACT [SPEED=<mm/s>] [DIST=<mm>]` | Manual retract (default 50mm)                         |
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
|   1   |   *    |   0  | EMPTY        | 1.0 + `safety_gain`       |
|   0   |   0    |   0  | EMPTY_MIDDLE | 1.0 + `drift_gain`        |
|   0   |   1    |   0  | **MIDDLE**   | **1.00 (dead-band)**      |
|   0   |   1    |   1  | FULL_MIDDLE  | 1.0 - `drift_gain`        |
|   0   |   0    |   1  | FULL         | 1.0 - `safety_gain`       |

The MIDDLE zone is the target equilibrium. When the middle sensor alone is active, the multiplier is exactly 1.0 -- the buffer rides the extruder step-for-step with zero correction. Deviations toward empty or full apply proportional corrections that push the filament loop back toward center.

### Fault escalation

If a safety zone (EMPTY or FULL) persists for `fault_escalation_time` seconds, the gain increases to `safety_gain * 1.5`. If it persists past the safety timeout, an error is raised (EMPTY) or a forced retract is executed (FULL).

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

### Limitations

- **Single extruder only.** The buffer syncs to whichever extruder is active at `klippy:ready`. Multi-extruder tool changes are not handled.

## Tests

```bash
pytest tests/
```

The test suite uses pure mock objects with no Klipper dependency. Coverage includes rotation_distance feedback, zone classification, dead-band invariant, fault escalation, safety timeouts, manual control, button handling, error conditions, and state transitions.

## License

GPLv3 -- see [LICENSE](LICENSE).
