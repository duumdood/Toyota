# Desktop range calibration

Backup before this change: ../goodprototype 3/.

70 cm real sensor distance maps to 70 m in the simulator. Scale is 0.1 road
metres per sensor mm (1 cm = 1 m). Valid echoes beyond 70 cm are clamped to
70 m, not treated as absent. No-echo behavior and echo timings are unchanged.
Minimum simulated gap remains 2 m. Python reads scaled gap from the board;
no Python scaling constant or crash-physics change is required.

Examples: 10/20/40/60/70 cm -> 10/20/40/60/70 m. Seven-segment displays
1/2/4/6/7 respectively. No return still alternates 0/9.

Red LED calibration now uses real millimetres directly: off at >=60 cm,
full at <=8 cm in day / <=14 cm at night, with 500 ms full-range fade.
This preserves the previous full-brightness distances and expands the fade
range for desktop experiments. Vehicle speed does not change LED thresholds.

Suggested demo: begin with a broad flat target 60–70 cm away and cruise 50–60
km/h; move gradually closer to observe braking and cruise reduction. For the
PC crash prediction at 60 km/h, required distance is about 21.51 m (21.51 cm
real with this scale). A deficit >0.5 m means below about 21.01 cm, sustained
over the confirmation interval; braking during that interval can change the
outcome. Reset then speed+ again after stopping because cruise follows braking.

Update road_config.h and road_app.c in CubeIDE, Clean/Build/flash. Real GPIO,
sensor measurements and perceived brightness still need on-board validation.
