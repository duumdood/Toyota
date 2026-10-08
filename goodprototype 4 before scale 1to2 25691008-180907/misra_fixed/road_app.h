#ifndef ROAD_APP_H
#define ROAD_APP_H
#include <stdbool.h>
#include <stdint.h>

/* INPUT CONTRACT: only the board driver writes these values.
 * No keyboard, Python command, potentiometer or synthetic input is used. */
typedef struct {
    bool speed_up_pressed;    /* PA10 / D2, debounced: true while held */
    bool speed_down_pressed;  /* PB3 / D3, debounced: true while held */
    bool reset_pressed;       /* PB4 / D5, debounced: reset at ANY time */
    uint16_t ldr_adc;         /* PA1 / A1, ADC1_IN1: raw 12-bit 0..4095 */
    uint32_t ldr_ms;          /* completion timestamp, milliseconds */
    bool ldr_ready;
    uint32_t distance_mm;     /* US-100 measured distance, real millimetres */
    uint32_t range_ms;        /* capture/timeout timestamp */
    uint32_t range_sequence;  /* increments once per completed measurement */
    uint8_t range_status;     /* 0=valid echo, 1=no return, 2=invalid/fault */
    bool range_ready;
} RoadInputs;

/* OUTPUT CONTRACT: Python receives these results, never runs IDM itself. */
typedef struct {
    float speed_mps;
    float target_mps;
    float acceleration_mps2;
    float odometer_m;
    float gap_m;
    float lead_speed_mps;
    /* Local LED diagnostics; the RD1 wire format stays unchanged. */
    float desired_gap_m;
    float red_near_mm;
    float red_far_mm;
    uint32_t red_target_permille;
    bool night;
    bool present;
    bool lead_valid;
    bool braking;
    bool done;
    bool fault;
    bool accel_blocked;   /* Latched warning: cleared when requested acceleration is available. */
    uint8_t done_reason;  /* 0=running, 1=stop complete, 2=deceleration complete */
    uint32_t reset_id;
} RoadOutput;

typedef struct {
    RoadOutput out;
    bool previous_up;
    bool previous_down;
    bool previous_reset;
    bool did_brake;
    uint32_t hold_up_ms;
    uint32_t hold_down_ms;
    uint32_t light_ms;
    uint32_t seen_range;
    uint32_t previous_range_ms;
    bool have_previous_range;
    float previous_gap;
    float range_rate;
    float estimate_age;
    float settled_s;
    float accel_request_mps; /* remembered request for warning clearance only */
} RoadApp;

void RoadApp_Init(RoadApp *app);
void RoadApp_Tick(RoadApp *app, const RoadInputs *input, uint32_t now_ms);
float Road_Idm(float speed, float target, float gap, float closing,
               bool present, bool night);
float Road_DesiredGap(float speed, float closing, bool night);
#endif
