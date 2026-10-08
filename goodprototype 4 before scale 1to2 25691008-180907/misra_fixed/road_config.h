#ifndef ROAD_CONFIG_H
#define ROAD_CONFIG_H

/* All calibration lives here. Values are demo defaults, NOT measured calibration.
 * MISRA checksheet Rule 5: every tuning constant used by the application is
 * named here instead of being written as a "magic" number in the code. */

/* ---------------------------------------------------------------- timing */
#define ROAD_TICK_MS             10U
#define ROAD_TELEMETRY_MS        50U
#define ROAD_UART_BAUD           115200U
#define ROAD_CLOCK_HZ            16000000U /* HSI, AHB/APB1/APB2 divide by 1 */
#define ROAD_MS_TO_S             0.001f
#define ROAD_SENSOR_STALE_MS     250U

/* --------------------------------------------------------------- buttons */
#define ROAD_BUTTON_ACTIVE_LOW   1U /* Shield pull-ups: pressed = GPIO LOW. */
#define ROAD_DEBOUNCE_MS         25U
#define ROAD_HOLD_DELAY_MS       350U
#define ROAD_TAP_MPS             0.5f /* 1.8 km/h per tap */
#define ROAD_HOLD_MPS_PER_SEC    4.0f /* 14.4 km/h per second held */

/* ------------------------------------------------------------------- LDR */
/* Lecture 0400_ADC p42: LDR below fixed resistor => darker = higher ADC.
 * Cover LDR completely to enter night. Reverse this flag for reversed wiring.
 * Watch g_road_inputs.ldr_adc in debugger; tune after measuring room/covered.
 * These thresholds act on dark_score (raw, or 4095-raw), NOT lux. */
#define ROAD_ADC_MAX             4095U /* 12-bit full scale */
#define ROAD_LDR_DARK_IS_HIGH    1U
#define ROAD_NIGHT_ENTER_ADC     3000U
#define ROAD_NIGHT_EXIT_ADC      3000U
#define ROAD_NIGHT_ENTER_MS      400U
#define ROAD_NIGHT_EXIT_MS       200U

/* ---------------------------------------------------- distance / US-100 */
/* Desktop demo: 10 mm measured = 1 m of road; 70 cm covers the full scene.
 * Physical distance stays separately available as distance_mm. */
#define ROAD_METRES_PER_SENSOR_MM 0.1f
#define ROAD_MIN_GAP_M           2.0f
#define ROAD_MAX_GAP_M           70.0f
#define ROAD_ECHO_PERIOD_US      60000U
#define ROAD_ECHO_TIMEOUT_US     35000U
#define ROAD_ECHO_TRIGGER_US     12U
#define ROAD_ECHO_MIN_US         100U
#define ROAD_ECHO_MAX_US         26000U

/* range_status values written by the driver */
#define ROAD_RANGE_VALID         0U
#define ROAD_RANGE_NO_ECHO       1U
#define ROAD_RANGE_FAULT         2U

/* ---------------------------------------------------- IDM (Treiber 2000) */
#define ROAD_IDM_A_MAX           3.0f  /* maximum acceleration, m/s^2 */
#define ROAD_IDM_DECEL_LIMIT     (-7.0f) /* -2 x comfortable braking (3.5) */
#define ROAD_IDM_STOP_DECEL      (-3.0f) /* target speed zero while moving */
#define ROAD_IDM_TWO_SQRT_AB     6.480740698f /* 2*sqrt(a_max*b) = 2*sqrt(3*3.5) */
#define ROAD_IDM_T_DAY_S         1.0f  /* time headway */
#define ROAD_IDM_T_NIGHT_S       1.8f
#define ROAD_IDM_S0_DAY_M        4.0f  /* jam distance */
#define ROAD_IDM_S0_NIGHT_M      7.0f
#define ROAD_IDM_DYNAMIC_MAX_M   100000.0f /* upper clamp of v*T + v*dv term */

/* ------------------------------------------------------- speed / cruise */
#define ROAD_KMH_PER_MPS         3.6f
#define ROAD_INITIAL_TARGET_KMH  50.0f
#define ROAD_VMAX_DAY_KMH        120.0f
#define ROAD_VMAX_NIGHT_KMH      100.0f
#define ROAD_SPEED_LIMIT_MPS     40.0f /* hard ceiling of integrated speed */
#define ROAD_ODOMETER_WRAP_M     4000000.0f
#define ROAD_BRAKING_ACCEL       (-0.35f) /* below this = "braking" */

/* --------------------------------------------- lead estimation (filter) */
#define ROAD_RATE_MAX_DT_S       0.25f /* gap between samples too long */
#define ROAD_RATE_MAX_JUMP_MPS   80.0f /* reject discontinuous samples */
#define ROAD_RATE_FILTER_TAU_S   0.14f /* first order low-pass */
#define ROAD_LEAD_VALID_AGE_S    0.12f

/* ----------------------------------------- accel_blocked warning */
#define ROAD_SPEED_TOL_MPS       0.05f
#define ROAD_ACCEL_TOL_MPS2      0.05f

/* --------------------------------------------------- end of sequence */
#define ROAD_STOP_SPEED_MPS      0.3f
#define ROAD_STABLE_RATE_MPS     0.5f
#define ROAD_STABLE_ACCEL_MPS2   0.4f
#define ROAD_SETTLE_TIME_S       1.2f
#define ROAD_DONE_RUNNING        0U
#define ROAD_DONE_STOPPED        1U
#define ROAD_DONE_SETTLED        2U

/* ------------------------------------------------------------ red LED */
/* Red LED brightness ramps purely against the REAL measured US-100 distance.
 * red_near_mm/red_far_mm are FIXED reference points, never derived from
 * speed or the IDM's dynamic desired gap -- brightness must not move just
 * because the car sped up while the object's real distance stayed put. */
#define ROAD_RED_NEAR_DAY_MM     80.0f
#define ROAD_RED_NEAR_NIGHT_MM   140.0f
#define ROAD_RED_FAR_MM          600.0f
#define ROAD_RED_PWM_PERIOD_US   1000U
#define ROAD_RED_FADE_MS         500U /* full off-to-on ramp duration */
#define ROAD_PERMILLE_FULL       1000U
#define ROAD_PERMILLE_SCALE      1000.0f
#define ROAD_ROUND_HALF          0.5f

#if (ROAD_RED_FADE_MS < ROAD_TICK_MS) || (ROAD_RED_FADE_MS > (ROAD_RED_PWM_PERIOD_US * ROAD_TICK_MS))
#error "Red LED fade duration is outside supported range"
#endif

#if ROAD_NIGHT_ENTER_ADC < ROAD_NIGHT_EXIT_ADC
#error "Night entry must be at least as dark as night exit"
#endif
#endif
