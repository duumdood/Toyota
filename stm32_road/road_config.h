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
#define ROAD_NIGHT_ENTER_ADC     3800U
#define ROAD_NIGHT_EXIT_ADC      3500U
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

#if ROAD_NIGHT_ENTER_ADC <= ROAD_NIGHT_EXIT_ADC
#error "Night entry must be darker than night exit"
#endif
#endif
