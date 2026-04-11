# TODO

## Multi-extruder support

The buffer currently syncs to whichever extruder is active at `klippy:ready` and does not re-sync on tool changes. Multi-extruder printers (T0/T1) will need event-driven re-sync via `toolhead:set_extruder` or equivalent.

## Sensor debounce

Sensor callbacks currently update the rotation_distance multiplier immediately. If a sensor chatters at a boundary, the multiplier will toggle between two values each cycle. Consider adding a minimum dwell time before committing to a multiplier change, or low-pass filtering the multiplier updates.
