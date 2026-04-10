# LLL-Plus Klipper Plugin

A Klipper extras module for real-time control of the Mellow LLL Plus filament buffer. Replaces the stock macro-based configuration with a native module that tracks extruder velocity and drives the buffer motor in closed-loop using hall effect sensor feedback.

## Features

- **Velocity tracking** -- hooks into G0/G1/G2/G3 moves to match the buffer motor speed to the extruder in real time
- **Zone-based control** -- three hall sensors (empty, middle, full) define buffer zones; the motor feeds, matches, or retracts accordingly
- **Initial fill** -- continuous forward feed on first filament insertion to fill the buffer tube before burst logic kicks in
- **Burst recovery** -- automatically pulses the motor when the buffer empties during travel moves
- **Retraction following** -- mirrors extruder retractions when idle; smooths through them during printing
- **Manual override** -- physical feed/retract buttons and GCode commands for filament loading
- **Error protection** -- forward timeout, sensor conflict detection, burst exhaustion limit, and optional pause-on-runout
- **Rate-limited UART** -- deferred motor writes prevent overwhelming the TMC2208 on the STM32F072

## Hardware

- Mellow LLL Plus filament buffer board (STM32F072 MCU)
- TMC2208 / TMC2225 stepper driver (controlled via VACTUAL over SPI UART)
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

Copy `sample_config/lll-plus.cfg` into your Klipper config directory and adjust pin assignments and serial path for your setup. Key parameters:

| Parameter            | Default | Description                                                      |
|----------------------|---------|------------------------------------------------------------------|
| `speed_rpm`          | 120     | Base motor speed (RPM) for manual ops and safety overrides       |
| `velocity_factor`    | 1.05    | Middle-zone speed multiplier to compensate for friction          |
| `correction_factor`  | 1.3     | Speed multiplier when catching up from the empty zone            |
| `slowdown_factor`    | 0.5     | Speed multiplier in the full zone before retract kicks in        |
| `forward_timeout`    | 60.0    | Seconds of continuous forward feeding before error (0 = disable) |
| `full_zone_timeout`  | 3.0     | Seconds in full zone before switching from slowdown to retract   |
| `burst_feed_time`    | 0.5     | Duration (s) of each burst pulse                                 |
| `burst_delay`        | 0.5     | Idle time (s) in empty zone before burst triggers                |
| `velocity_window`    | 0.3     | Sliding window (s) for smoothed velocity                         |
| `drive_interval`     | 0.1     | Minimum interval (s) between motor UART writes                   |
| `initial_fill_timeout` | 10.0  | Duration (s) of continuous forward feed on first filament insertion |
| `pause_on_runout`    | True    | Pause print on filament runout or forward timeout                |
| `follow_retract`     | True    | Follow extruder retractions when not printing                    |

See `sample_config/lll-plus.cfg` for the full annotated reference.

## GCode Commands

| Command                        | Description                                                  |
|--------------------------------|--------------------------------------------------------------|
| `BUFFER_STATUS`                | Report current state, sensor readings, velocities, and zone  |
| `BUFFER_ENABLE`                | Enable automatic buffer control                              |
| `BUFFER_DISABLE`               | Stop the motor and disable automatic control                 |
| `BUFFER_FEED [SPEED=<rpm>]`    | Manual forward feed (auto-stops when full sensor triggers)   |
| `BUFFER_RETRACT [SPEED=<rpm>]` | Manual retract                                               |
| `BUFFER_STOP`                  | Stop the motor and return to automatic mode                  |
| `BUFFER_SET_SPEED SPEED=<rpm>` | Change the base motor speed                                  |
| `BUFFER_CLEAR_ERROR`           | Clear an error state and return to idle                      |

## State Machine

### Sensor Zones

The three hall sensors map to a zone that drives motor behavior. Sensors read `1` when the filament loop blocks the magnet.

| Empty | Middle | Full | Zone                       |
|:-----:|:------:|:----:|----------------------------|
|   1   |   *    |   1  | **ERROR** -- sensor conflict |
|   1   |   *    |   0  | EMPTY                       |
|   0   |   0    |   1  | FULL                        |
|   0   |   1    |   1  | FULL_MIDDLE                 |
|   0   |   1    |   0  | MIDDLE                      |
|   0   |   0    |   0  | EMPTY_MIDDLE                |

### Zone Behavior

Each zone decides a motor direction and speed. `V` is the smoothed extruder velocity. Speeds shown are the VACTUAL conversion of the velocity multiplied by the listed factor.

| Zone        | Condition                          | Motor          | Speed                    |
|-------------|------------------------------------|----------------|--------------------------|
| EMPTY       | initial fill active                | FORWARD        | `speed_rpm`              |
| EMPTY       | initial fill timed out             | STOP → STOPPED | --                       |
| EMPTY       | V > min                            | FORWARD        | V * `correction_factor`  |
| EMPTY       | V = 0, burst active                | FORWARD        | `speed_rpm`              |
| EMPTY       | V = 0, burst delay elapsed         | FORWARD (burst) | `speed_rpm`             |
| EMPTY       | V = 0, burst delay pending         | STOP           | --                       |
| EMPTY       | burst count >= 5                   | **ERROR**      | --                       |
| MIDDLE      | V > min                            | FORWARD        | V * `velocity_factor`    |
| MIDDLE      | V = 0                              | STOP           | --                       |
| FULL_MIDDLE | V > min                            | FORWARD        | V * `slowdown_factor`    |
| FULL_MIDDLE | V = 0                              | STOP           | --                       |
| FULL        | V > min, feed time < timeout       | FORWARD        | V * `slowdown_factor`    |
| FULL        | V = 0, feed time < timeout         | STOP           | --                       |
| FULL        | feed time >= `full_zone_timeout`   | BACK           | `speed_rpm`              |
| EMPTY_MIDDLE | held velocity > min                          | FORWARD        | held V * `correction_factor` |
| EMPTY_MIDDLE | no held velocity                             | FORWARD        | `speed_rpm`              |

The EMPTY_MIDDLE zone (all sensors inactive) always feeds forward. On entry it captures the current extruder velocity and holds it, ignoring subsequent retractions and velocity decay. This ensures the buffer reaches the middle sensor. Retraction following and velocity decay are also suppressed while in this zone.

When not printing and the extruder is retracting, zone evaluation is normally bypassed and the motor follows the retraction directly (`follow_retract`). During printing, retractions zero the velocity instead, allowing the smoothing window to bridge brief gaps. Neither applies in EMPTY_MIDDLE — the motor always feeds forward.

### State Transitions

| From                               | To             | Trigger                                                              |
|------------------------------------|----------------|----------------------------------------------------------------------|
| DISABLED                           | IDLE           | `klippy:ready`                                                       |
| IDLE                               | STOPPED        | `BUFFER_ENABLE` or filament inserted (triggers initial fill)         |
| IDLE                               | DISABLED       | `BUFFER_DISABLE`                                                     |
| STOPPED                            | FEEDING        | Zone evaluation drives motor forward                                 |
| STOPPED                            | RETRACTING     | Zone evaluation drives motor back                                    |
| FEEDING                            | STOPPED        | Zone evaluation stops motor                                          |
| FEEDING                            | RETRACTING     | Zone evaluation drives motor back                                    |
| RETRACTING                         | STOPPED        | Zone evaluation stops motor                                          |
| RETRACTING                         | FEEDING        | Zone evaluation drives motor forward                                 |
| FEEDING (initial fill)             | STOPPED        | Initial fill timeout expires                                         |
| FEEDING / STOPPED / RETRACTING     | IDLE           | Filament runout                                                      |
| any (not ERROR)                    | MANUAL_FEED    | `BUFFER_FEED` or feed button press                                   |
| any (not ERROR)                    | MANUAL_RETRACT | `BUFFER_RETRACT` or retract button press                             |
| MANUAL_FEED                        | STOPPED/IDLE   | Button release (restores prior auto-enable state)                    |
| MANUAL_FEED                        | IDLE           | Full sensor sustained for `manual_feed_full_timeout`                 |
| MANUAL_RETRACT                     | STOPPED/IDLE   | Button release (restores prior auto-enable state)                    |
| any (not ERROR)                    | STOPPED/IDLE   | Both buttons pressed (toggles auto-enable)                           |
| any                                | ERROR          | Sensor conflict, forward timeout, or burst exhaustion                |
| ERROR                              | STOPPED        | `BUFFER_CLEAR_ERROR` (auto-enabled)                                  |
| ERROR                              | IDLE           | `BUFFER_CLEAR_ERROR` (not auto-enabled)                              |
| any                                | DISABLED       | `BUFFER_DISABLE` or `klippy:shutdown`                                |

**Invariants:**

- **ERROR** blocks all automatic control and manual feed/retract; only `BUFFER_CLEAR_ERROR` exits it.
- **MANUAL_FEED / MANUAL_RETRACT** bypass zone evaluation; the motor is driven directly.
- **Both buttons** toggle auto-enable: IDLE (disabled) <-> STOPPED (enabled).
- **DISABLED** blocks material-insert auto-enable; only `BUFFER_ENABLE` exits it.

Motor commands are rate-limited through a deferred write system to stay within TMC2208 UART bandwidth on the STM32F072.

## Tests

```bash
pytest tests/
```

The test suite uses pure mock objects with no Klipper dependency. Coverage includes zone transitions, velocity tracking, burst cycles, retraction handling, manual control, error conditions, and deferred drive rate-limiting.

## License

GPLv3 -- see [LICENSE](LICENSE).
