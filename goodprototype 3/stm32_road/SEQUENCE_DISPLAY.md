# Sequence and dashboard changes (2026-10-02)

Original files before this request: `../goodprototype 2/`. Previous backup
`goodprototype 1` is unchanged. Do not compile backup folders in CubeIDE.

## Acceleration warning

Bit 64 is latched after speed+ is attempted while the lead blocks acceleration.
Releasing the button does not clear it. It clears when the requested acceleration
is available, no lead is detected, data is invalid/stale, reset is pressed or a
sequence ends. No echo cannot prove the road is clear: it removes this warning,
but is still reported as NO RETURN. The warning does not stop the simulation.

The remembered request `accel_request_mps` is used only to decide when to clear
the warning. It never automatically raises cruise. Without it, lowering cruise
would itself clear the warning even while the obstacle remains.

## Cruise follows braking

As requested, cruise falls to actual speed after a braking integration step
when the lead contributes to braking. A manually lowered target is not raised.
When the stop sequence reaches zero speed, cruise becomes zero too. Clearing
the road or resetting does not restore the former cruise; press speed+ to resume.
The existing day/night caps still apply. This changes behavior intentionally.

## One-digit 7-segment

Pin source: `E:/Toyota/source/exam1/files/seven_segment.c` and `gpio_config.c`:
BCD bit 0 PC7, bit 1 PA8, bit 2 PB10, bit 3 PA9. Pins are push-pull outputs.
BSRR writes preserve neighboring LED, button and ultrasonic pins. No new IRQ.

Show floor(gap_m / 10), clamped to 9: 40..49.99 m -> 4, 0..9.99 m -> 0,
90 m and above -> 9. Invalid/no return/stale -> alternate 0 and 9 every 500 ms.
Only valid decimal codes are used; a blank code has not been established.
Updates from the MCU's scaled gap every 10 ms, independently of render smoothing.
Crash exists only on the PC, so the physical digit continues showing live range
after a displayed crash. Hardware decoder wiring must still be tested on board.

## Python presentation

Warning appears in the center like the brake banner. End-of-sequence/Crash and
the night popup have priority. Cruise is larger and positioned beside speed.
Ego brake lamps use bright red while braking, dark red otherwise. No lead brake
state is available, so the program does not invent it.

An immutable original BoardTelemetry packet is saved once at sequence end.
The summary gives ego speed, estimated lead speed (unknown if unavailable),
gap and ego road position. Reset/reboot clears the saved result. Further sensor
changes cannot overwrite it. Crash values are captured before the PC forces
rendered speed to zero and labeled 'At predicted crash'. The stopping model
predicts inability to stop; it does not measure impact speed or impact location.

## Verification and deployment

C application/encoder tests run on ARM emulation, including latched warning,
cruise reduction, reset and restart. Python offscreen checks verify summaries,
summary persistence/reset and warning priority. Full firmware compile and
GPIO/7-segment behavior still require CubeIDE and the physical shield.

Copy road_app.c/.h and road_board.c into the existing CubeIDE project, retain
the bit-64 road_protocol.c from the previous feature, Clean/Build/flash, then
restart the updated Python viewer. No UART field additions in this change.
