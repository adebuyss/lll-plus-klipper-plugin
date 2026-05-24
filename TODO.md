# TODO

## Multi-extruder support

The buffer currently syncs to whichever extruder is active at `klippy:ready` and does not re-sync on tool changes. Multi-extruder printers (T0/T1) will need event-driven re-sync via `toolhead:set_extruder` or equivalent.

## MCU saturation on stm32f072xb (LLL_PLUS host MCU)

Sensor-edge storms (~30 Hz from a bouncing hall sensor) plus buffer-stepper step generation plus TMC UART polling can saturate the 48 MHz F0 enough to produce `Rescheduled timer in the past` shutdowns. Plugin-level rate-limiting (`zone_debounce_ticks`, `_apply_multiplier` 2 Hz cap) takes pressure off the host but does not solve the MCU-side saturation. Companion mitigations to consider:

- Hardware RC debounce on the hall lines (~100 nF + 10 kΩ).
- Tighter Klipper button-pin debounce window on the hall pins.
- Lower microsteps on the buffer stepper if mechanical resolution permits.
- Move the buffer stepper off LLL_PLUS to a less-loaded MCU.
