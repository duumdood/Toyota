#ifndef ROAD_CONFIG_H
#define ROAD_CONFIG_H

/* All calibration lives here. Values are demo defaults, NOT measured calibration. */
#define ROAD_TICK_MS             10U
#define ROAD_TELEMETRY_MS        50U
#define ROAD_UART_BAUD           115200U
#define ROAD_CLOCK_HZ            16000000U /* HSI, AHB/APB1/APB2 divide by 1 */
#define ROAD_BUTTON_ACTIVE_LOW   1U /* Shield pull-ups: pressed = GPIO LOW. */
#define ROAD_DEBOUNCE_MS         25U
#define ROAD_HOLD_DELAY_MS       350U
#define ROAD_TAP_MPS             0.5f /* 1.8 km/h per tap */
#define ROAD_HOLD_MPS_PER_SEC    4.0f /* 14.4 km/h per second held */

/* Lecture 0400_ADC p42: LDR below fixed resistor => darker = higher ADC.
 * Cover LDR completely to enter night. Reverse this flag for reversed wiring.
 * Watch g_road_inputs.ldr_adc in debugger; tune after measuring room/covered.
 * These thresholds act on dark_score (raw, or 4095-raw), NOT lux. */
#define ROAD_LDR_DARK_IS_HIGH    1U
#define ROAD_NIGHT_ENTER_ADC     3000U
#define ROAD_NIGHT_EXIT_ADC      3000U
#define ROAD_NIGHT_ENTER_MS      400U
#define ROAD_NIGHT_EXIT_MS       200U
#define ROAD_SENSOR_STALE_MS     250U

/* Preserve mock visual scale: 10 mm measured = 0.5 m of road.
 * Physical distance stays separately available as distance_mm. */
#define ROAD_METRES_PER_SENSOR_MM 0.05f
#define ROAD_MIN_GAP_M           2.0f
#define ROAD_MAX_GAP_M           100.0f
#define ROAD_ECHO_PERIOD_US      60000U
#define ROAD_ECHO_TIMEOUT_US     35000U
#define ROAD_ECHO_TRIGGER_US     12U
#define ROAD_ECHO_MIN_US         100U
#define ROAD_ECHO_MAX_US         26000U

/* Red LED brightness ramps purely against the REAL measured US-100 distance.
 * red_near_mm/red_far_mm are FIXED reference points, never derived from
 * speed or the IDM's dynamic desired gap -- brightness must not move just
 * because the car sped up while the object's real distance stayed put. */
#define ROAD_RED_FAR_M           12.0f /* fixed far edge: LED fully off here */
#define ROAD_RED_MIN_SPAN_M      1.0f /* minimum fade band in road metres */
#define ROAD_RED_PWM_PERIOD_US   1000U
#define ROAD_RED_FADE_MS         500U /* full off-to-on ramp duration */

#if ROAD_RED_FADE_MS < ROAD_TICK_MS || ROAD_RED_FADE_MS > (ROAD_RED_PWM_PERIOD_US * ROAD_TICK_MS)
#error "Red LED fade duration is outside supported range"
#endif

#if ROAD_NIGHT_ENTER_ADC < ROAD_NIGHT_EXIT_ADC
#error "Night entry must be at least as dark as night exit"
#endif
#endif
