# Desktop distance calibration: 1 cm = 2 m

Current source sets: `misra_fixed/` and `stm32_road/` have matching calibration.
The original sources are saved in `goodprototype 4 before scale 1to2 25691008-180907/`.

## Mapping and presence

- ROAD_METRES_PER_SENSOR_MM = 0.2: 10 real mm represents 2 road metres.
- ROAD_SENSOR_VIEW_MAX_MM = 700: echoes through 70 cm are inside the demo.
- ROAD_MAX_GAP_M is derived from these constants: 140 m.
- Beyond 700 mm, the MCU clears present/lead_valid and resets the range-rate
  history. The raw echo remains valid and its real distance is still transmitted.
  This is a desktop scene rule, not a real-vehicle safety policy.
- Python uses the same scale and 140 m visibility boundary, and also requires
  the board's present flag. A 120 cm reading displays 240 m but no lead car.
- No echo retains the existing behavior; invalid/stale data remains a fault.
- The 2 m control minimum remains. No hysteresis was added at the 70 cm boundary;
  readings that oscillate across it can toggle presence. Stay away from the edge
  when demonstrating a steady lead vehicle.
- Completed sequences keep their frozen car positions until physical Reset.

| Real cm | Road m | Seven-segment |
| --- | --- | --- |
| 10 | 20 | 1 |
| 20 | 40 | 2 |
| 40 | 80 | 4 |
| 60 | 120 | 6 |
| 70 | 140 | 7 |
| >70 | Outside scene | alternating 0/9 |

The single-digit BCD display now uses 20 m per step (round down), not 10 m.
The live Python raw-distance panel remains available outside the scene.

## Preserved behavior

IDM parameters, cruise-follow-down, LDR threshold 3000 with existing dwell,
blue LED, UART RD1 field order/CRC, sensor timing, and PC crash algorithm stay
unchanged. Red LED continues to use physical distance: off at >=60 cm, full at
<=8 cm by day / <=14 cm at night, with 500 ms full-range fade.
The range-rate filter has not been retuned: the same physical motion/noise now
represents twice the road motion/noise. Test real measurements before changing it.
140 m is a demo range, not a guarantee of stopping at any particular speed.

## Install together

For the MISRA source set, replace road_config.h, road_app.c and road_board.c
in your existing CubeIDE project using the files from misra_fixed/. Keep its
other .c/.h files from that same source set. Clean, Build, flash and Resume.
Do not compile the duplicate stm32_road source files alongside misra_fixed.

Close the old simulator and CoolTerm, then from the repository run:

```powershell
.\.venv\Scripts\python.exe road_simulator_board.py --port COM15
```

Both road_board_io.py and road_simulator_board.py must be updated. RD1 has no
calibration-version field: old firmware can still parse but its distances will
not match this Python calibration. Flash and restart as a pair.

## Verification

ARM GCC + Unicorn runs the application/protocol regression suite against both
source sets. It covers 400 mm -> 80 m, 700 mm -> 140 m, 701 mm outside the scene,
reacquisition, doubled range derivative, unchanged LED thresholds, LDR, cruise,
warning and sequence logic. Python tests cover the 140 m parser boundary,
visibility, raw distance, link loss, and sequence freeze. Offscreen UI checks
exercise 140 m appearance and 240 m hidden-distance display.
These checks do not validate electrical wiring or physical sensor performance.
