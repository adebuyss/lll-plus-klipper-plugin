# TODO

## Zone oscillation / bang-bang problem

Filament sitting at a sensor boundary can cause rapid zone toggling with no dampening. Sensor callbacks directly call `_evaluate_and_drive()` with zero debouncing, bypassing the drive rate limiter.

### Symptoms

| Boundary | Oscillation | Severity |
|----------|-------------|----------|
| MIDDLE <-> EMPTY | Speed jumps: 1.05x <-> 1.3x (same direction) | Moderate -- rapid VACTUAL writes |
| MIDDLE <-> FULL_MIDDLE | Speed jumps: 1.05x <-> 0.5x (same direction) | High -- extreme speed swing |
| EMPTY <-> EMPTY_MIDDLE | STOP <-> FORWARD (when V=0, held velocity kicks in) | High -- direction flapping |
| FULL <-> FULL_MIDDLE | Retract timeout resets each cycle | High -- retract never triggers |

### Root causes

- Sensor callbacks (`buffer.py:486-495`) trigger immediate re-evaluation -- no debounce
- Zone identification (`buffer.py:796-805`) is pure boolean -- no hysteresis or dead-band
- State resets on zone entry (`_reset_burst_state`, `_full_zone_feed_time`) defeat timeout/error detection during oscillation
  - Burst count resets when oscillating EMPTY <-> MIDDLE, so the 5-burst error limit never fires
  - Full zone feed time resets on every FULL entry, delaying retract indefinitely

### Existing dampening (insufficient)

- Control timer (250ms) -- sensor callbacks bypass it
- Drive rate limit (100ms) -- only applies to G-code path, not sensor callbacks
- Velocity window (300ms max) -- actually prolongs oscillation by keeping velocity alive across zone flaps

### Possible fixes

- Sensor-level debounce timer: suppress re-evaluation within N ms of last zone transition
- Hysteresis: require a sensor to be stable for a minimum dwell time before committing to a zone change
- Rate-limit sensor callbacks through `_request_drive_update` instead of calling `_evaluate_and_drive` directly
- Preserve burst/timeout accumulators across brief zone excursions (e.g. don't reset burst count if we were in EMPTY less than 1s ago)
