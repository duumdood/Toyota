# Acceleration warning

Original sources: `../goodprototype 1/` (14 files, verified SHA256 manifest).
Keep this backup outside CubeIDE source search paths to avoid duplicate symbols.

`accel_blocked` is diagnostic only. It does not reject speed-button increments,
change the cruise target, change acceleration, or alter LED/Crash behavior.

RoadApp_Tick requires: valid sensors, running sequence, a lead vehicle, speed-up
pressed alone, target > pre-integration speed + 0.05 m/s, actual IDM acceleration
<= 0.05 m/s², and free-road IDM acceleration > 0.05 m/s². The last comparison
attributes the restriction to the lead vehicle rather than approaching cruise
speed. The night cap is applied before evaluation. Clear at tick start and when
the sequence finishes so fault/reset/done/release cannot retain the warning.

Changed production files:
- road_app.h: output boolean.
- road_app.c: decision and lifecycle.
- road_protocol.c: flags bit 6 (mask 64); RD1 fields and CRC unchanged.
- ../road_board_io.py: flags 0..127 and boolean decode; legacy flag defaults false.
- ../road_simulator_board.py: CANNOT ACCELERATE - CAR AHEAD banner, hidden during
  disconnect/fault/done/Crash/night-transition popup.

road_board.c needs no change; it already sends RoadOutput via the encoder.
Install both Python files with the new firmware: old Python rejects flags >63.
New Python still accepts old RD1 packets without the flag.

This is a held-button state, not a latched event. A tap shorter than the 50 ms
telemetry period may be missed. No display hold or threshold hysteresis is added.
The 0.05 tolerances have different units: m/s for speed, m/s² for acceleration.

Verification: actual ARM C tests for close/far lead, release, both buttons,
reset, fault/stale, no echo, done, near cruise speed, night cap and night warning;
C encoder-to-Python decode including old and unknown bits; offscreen UI for
display and suppression. Hardware timing and full CMSIS firmware build still
need validation in CubeIDE/on the board.

An existing LED test failed before the feature changes because it expected the
older speed-dependent LED. Updated only its expected distances to the current
fixed 12 m far edge; preserved the existing LED implementation.
