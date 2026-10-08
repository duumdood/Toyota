/* Application layer: IDM car-following, cruise buttons, day/night and
 * end-of-sequence logic. Hardware independent (no register access).
 * MISRA checksheet version: no ternary operator, one assignment per line,
 * every if terminated with else, named constants from road_config.h. */
#include "road_app.h"
#include "road_config.h"

#define TRAPEZOID_HALF 0.5f /* average of old and new speed */

static float limit(float x, float low, float high);
static float magnitude(float x);
static float speed_cap_mps(bool night);
static float idm_with_desired(float speed, float target, float gap,
                              bool present, float desired);
static void update_red_target(RoadApp *app, const RoadInputs *input, bool valid);
static void reset_sequence(RoadApp *app);
static void update_light(RoadApp *app, const RoadInputs *input);
static void range_lost(RoadApp *app);
static void range_accept(RoadApp *app, const RoadInputs *input);
static void update_range(RoadApp *app, const RoadInputs *input);
static float button_delta(bool pressed, bool previous, uint32_t *held_ms);
static void handle_speed_buttons(RoadApp *app, const RoadInputs *input);
static void freeze_outputs(RoadApp *app);
static void update_accel_warning(RoadApp *app, const RoadInputs *input, float old);
static void integrate_motion(RoadApp *app, float old, float dt);
static void follow_cruise(RoadApp *app, float free_accel);
static void check_sequence_end(RoadApp *app, float dt);
static void run_control_step(RoadApp *app, const RoadInputs *input, float old, float dt);

/* Clamp x into [low, high]. */
static float limit(float x, float low, float high)
{
    float result = x;
    if (x < low) {
        result = low;
    } else if (x > high) {
        result = high;
    } else {
        /* x is already inside the range */
    }
    return result;
}

/* Absolute value of a float. */
static float magnitude(float x)
{
    float result = x;
    if (x < 0.0f) {
        result = -x;
    } else {
        /* already positive */
    }
    return result;
}

/* Maximum cruise target: 120 km/h by day, 100 km/h by night. */
static float speed_cap_mps(bool night)
{
    float cap_kmh = ROAD_VMAX_DAY_KMH;
    if (night) {
        cap_kmh = ROAD_VMAX_NIGHT_KMH;
    } else {
        /* day limit already selected */
    }
    return cap_kmh / ROAD_KMH_PER_MPS;
}

/* IDM desired gap: s_star = s0 + v*T + v*dv / (2*sqrt(a*b)) */
float Road_DesiredGap(float speed, float closing, bool night)
{
    float headway = ROAD_IDM_T_DAY_S;
    float jam_gap = ROAD_IDM_S0_DAY_M;
    float dynamic = 0.0f;
    if (night) {
        headway = ROAD_IDM_T_NIGHT_S;
        jam_gap = ROAD_IDM_S0_NIGHT_M;
    } else {
        /* day parameters already selected */
    }
    dynamic = (speed * headway) + ((speed * closing) / ROAD_IDM_TWO_SQRT_AB);
    return jam_gap + limit(dynamic, 0.0f, ROAD_IDM_DYNAMIC_MAX_M);
}

/* IDM acceleration: a = a_max * [1 - (v/v0)^4 - (s_star / s)^2] */
static float idm_with_desired(float speed, float target, float gap,
                              bool present, float desired)
{
    float result = 0.0f;
    if (target <= 0.0f) {
        if (speed > 0.0f) {
            result = ROAD_IDM_STOP_DECEL;
        } else {
            result = 0.0f;
        }
    } else {
        float interaction = 0.0f;
        float speed_ratio = 0.0f;
        float square = 0.0f;
        if (present) {
            float gap_ratio = desired / limit(gap, ROAD_MIN_GAP_M, ROAD_MAX_GAP_M);
            interaction = gap_ratio * gap_ratio;
        } else {
            /* free road: no interaction term */
        }
        speed_ratio = speed / target;
        square = speed_ratio * speed_ratio;
        result = limit(ROAD_IDM_A_MAX * ((1.0f - (square * square)) - interaction),
                       ROAD_IDM_DECEL_LIMIT, ROAD_IDM_A_MAX);
    }
    return result;
}

float Road_Idm(float speed, float target, float gap, float closing,
               bool present, bool night)
{
    return idm_with_desired(speed, target, gap, present,
                            Road_DesiredGap(speed, closing, night));
}

static void update_red_target(RoadApp *app, const RoadInputs *input, bool valid)
{
    /* Physical LED calibration is independent of the simulator distance scale. */
    float near_mm = ROAD_RED_NEAR_DAY_MM;
    if (app->out.night) {
        near_mm = ROAD_RED_NEAR_NIGHT_MM;
    } else {
        /* day reference already selected */
    }
    app->out.red_near_mm = near_mm;
    app->out.red_far_mm = ROAD_RED_FAR_MM;
    app->out.red_target_permille = 0U;
    if (valid && (input->range_status == ROAD_RANGE_VALID)) {
        float level = (app->out.red_far_mm - (float)input->distance_mm)
                    / (app->out.red_far_mm - app->out.red_near_mm);
        app->out.red_target_permille =
            (uint32_t)((limit(level, 0.0f, 1.0f) * ROAD_PERMILLE_SCALE) + ROAD_ROUND_HALF);
    } else {
        /* no valid echo: LED target stays off */
    }
}

void RoadApp_Init(RoadApp *app)
{
    static const RoadApp zero_app = {0};
    *app = zero_app;
    app->out.target_mps = ROAD_INITIAL_TARGET_KMH / ROAD_KMH_PER_MPS;
    app->out.gap_m = ROAD_MAX_GAP_M;
    app->out.fault = true; /* Do not move until first sensor completions. */
}

static void reset_sequence(RoadApp *app)
{
    app->out.speed_mps = 0.0f;
    app->out.acceleration_mps2 = 0.0f;
    app->out.odometer_m = 0.0f;
    app->out.done = false;
    app->out.braking = false;
    app->did_brake = false;
    app->out.done_reason = ROAD_DONE_RUNNING;
    app->out.reset_id++;
    app->settled_s = 0.0f;
    app->hold_up_ms = 0U;
    app->hold_down_ms = 0U;
    /* A physical object does NOT disappear on reset; retain measured gap.
     * Preserve cruise setting and LDR mode. */
}

static void update_light(RoadApp *app, const RoadInputs *input)
{
#if ROAD_LDR_DARK_IS_HIGH
    uint32_t dark = (uint32_t)input->ldr_adc;
#else
    uint32_t dark = ROAD_ADC_MAX - (uint32_t)input->ldr_adc;
#endif
    bool crossing = false;
    uint32_t dwell = ROAD_NIGHT_ENTER_MS;
    /* Strict comparisons also support a single threshold: equality holds
     * the current mode instead of alternately entering and exiting night. */
    if (app->out.night) {
        crossing = (dark < ROAD_NIGHT_EXIT_ADC);
        dwell = ROAD_NIGHT_EXIT_MS;
    } else {
        crossing = (dark > ROAD_NIGHT_ENTER_ADC);
        dwell = ROAD_NIGHT_ENTER_MS;
    }
    if (crossing) {
        app->light_ms = app->light_ms + ROAD_TICK_MS;
    } else {
        app->light_ms = 0U;
    }
    if (app->light_ms >= dwell) {
        app->out.night = !app->out.night;
        app->light_ms = 0U;
    } else {
        /* keep current mode until dwell time is reached */
    }
}

/* No echo / invalid echo / outside demo range: forget the lead vehicle. */
static void range_lost(RoadApp *app)
{
    app->out.present = false;
    app->out.lead_valid = false;
    app->have_previous_range = false;
    app->range_rate = 0.0f;
    app->estimate_age = 0.0f;
    app->settled_s = 0.0f;
    app->did_brake = false;
}

/* Valid echo: scale to road metres and low-pass the closing rate. */
static void range_accept(RoadApp *app, const RoadInputs *input)
{
    float gap = limit((float)input->distance_mm * ROAD_METRES_PER_SENSOR_MM,
                      ROAD_MIN_GAP_M, ROAD_MAX_GAP_M);
    float dt = (float)(input->range_ms - app->previous_range_ms) * ROAD_MS_TO_S;
    float rate = 0.0f;
    if (dt > 0.0f) {
        rate = (gap - app->previous_gap) / dt;
    } else {
        rate = 0.0f;
    }
    if ((!app->have_previous_range) || (dt <= 0.0f) || (dt > ROAD_RATE_MAX_DT_S)
        || (magnitude(rate) > ROAD_RATE_MAX_JUMP_MPS)) {
        app->out.lead_valid = false;
        app->range_rate = 0.0f;
        app->estimate_age = 0.0f;
        app->settled_s = 0.0f;
    } else {
        app->estimate_age += dt;
        app->range_rate += ((rate - app->range_rate) * dt) / (ROAD_RATE_FILTER_TAU_S + dt);
        app->out.lead_valid = (app->estimate_age >= ROAD_LEAD_VALID_AGE_S);
    }
    app->out.present = true;
    app->have_previous_range = true;
    app->out.gap_m = gap;
    app->previous_gap = gap;
    app->previous_range_ms = input->range_ms;
}

static void update_range(RoadApp *app, const RoadInputs *input)
{
    bool fresh = input->range_ready && (input->range_sequence != app->seen_range);
    if (fresh) {
        app->seen_range = input->range_sequence;
        if ((input->range_status != ROAD_RANGE_VALID)
            || (input->distance_mm > ROAD_SENSOR_VIEW_MAX_MM)) {
            range_lost(app);
        } else {
            range_accept(app, input);
        }
    } else {
        /* no new measurement since last tick */
    }
}

/* Tap = one fixed step; hold longer than ROAD_HOLD_DELAY_MS = ramp. */
static float button_delta(bool pressed, bool previous, uint32_t *held_ms)
{
    float delta = 0.0f;
    if (!pressed) {
        *held_ms = 0U;
    } else if (!previous) {
        *held_ms = 0U;
        delta = ROAD_TAP_MPS;
    } else {
        if (*held_ms < ROAD_HOLD_DELAY_MS) {
            *held_ms += ROAD_TICK_MS;
        } else {
            /* hold delay already reached */
        }
        if (*held_ms >= ROAD_HOLD_DELAY_MS) {
            delta = ROAD_HOLD_MPS_PER_SEC * ((float)ROAD_TICK_MS * ROAD_MS_TO_S);
        } else {
            /* still waiting for hold delay */
        }
    }
    return delta;
}

static void handle_speed_buttons(RoadApp *app, const RoadInputs *input)
{
    /* Both speed buttons together are neutral. No conflicting increments. */
    if (input->speed_up_pressed && input->speed_down_pressed) {
        app->hold_up_ms = 0U;
        app->hold_down_ms = 0U;
    } else {
        app->out.target_mps += button_delta(input->speed_up_pressed,
                                            app->previous_up, &app->hold_up_ms);
        app->out.target_mps -= button_delta(input->speed_down_pressed,
                                            app->previous_down, &app->hold_down_ms);
    }
}

/* Freeze simulation on sensor fault / finished / reset; never assume free road. */
static void freeze_outputs(RoadApp *app)
{
    app->out.accel_blocked = false;
    app->accel_request_mps = 0.0f;
    app->out.acceleration_mps2 = 0.0f;
    app->out.braking = false;
    if (app->out.fault) {
        app->have_previous_range = false;
        app->out.lead_valid = false;
        app->hold_up_ms = 0U;
        app->hold_down_ms = 0U;
    } else {
        /* done or reset: keep lead estimate */
    }
}

/* Diagnostic only: warn when speed+ is blocked by the lead vehicle, not
 * merely by approaching the cruise target or the night speed cap. */
static void update_accel_warning(RoadApp *app, const RoadInputs *input, float old)
{
    float requested = app->out.target_mps;
    float requested_accel = 0.0f;
    float requested_free = 0.0f;
    bool obstacle_blocks = false;
    if (app->out.accel_blocked && (app->accel_request_mps > requested)) {
        requested = app->accel_request_mps;
    } else {
        /* use current cruise target */
    }
    requested = limit(requested, 0.0f, speed_cap_mps(app->out.night));
    requested_accel = idm_with_desired(old, requested, app->out.gap_m,
                                       app->out.present, app->out.desired_gap_m);
    requested_free = idm_with_desired(old, requested, app->out.gap_m,
                                      false, app->out.desired_gap_m);
    obstacle_blocks = app->out.present
                   && (requested > (old + ROAD_SPEED_TOL_MPS))
                   && (requested_accel <= ROAD_ACCEL_TOL_MPS2)
                   && (requested_free > ROAD_ACCEL_TOL_MPS2);
    if (!obstacle_blocks) {
        app->out.accel_blocked = false;
        app->accel_request_mps = 0.0f;
    } else if (input->speed_up_pressed && (!input->speed_down_pressed)) {
        /* Latch after an attempted increase, even when the button is released.
         * Clear when the lead no longer prevents acceleration or data is invalid. */
        app->out.accel_blocked = true;
        app->accel_request_mps = requested;
    } else {
        /* keep latched state */
    }
}

/* Euler speed integration and trapezoid odometer. */
static void integrate_motion(RoadApp *app, float old, float dt)
{
    app->out.speed_mps = limit(old + (app->out.acceleration_mps2 * dt),
                               0.0f, ROAD_SPEED_LIMIT_MPS);
    app->out.odometer_m += ((old + app->out.speed_mps) * TRAPEZOID_HALF) * dt;
    if (app->out.odometer_m >= ROAD_ODOMETER_WRAP_M) {
        app->out.odometer_m = 0.0f;
    } else {
        /* no wrap needed */
    }
    app->out.lead_speed_mps = app->out.speed_mps + app->range_rate;
    app->out.braking = (app->out.acceleration_mps2 < ROAD_BRAKING_ACCEL);
}

/* Follow actual speed when the lead contributes to automatic braking.
 * Never raise cruise automatically; speed+ is required after the road clears. */
static void follow_cruise(RoadApp *app, float free_accel)
{
    if (app->out.present && app->out.braking
        && (free_accel > (app->out.acceleration_mps2 + ROAD_ACCEL_TOL_MPS2))
        && (app->out.target_mps > app->out.speed_mps)) {
        app->out.target_mps = app->out.speed_mps;
    } else {
        /* cruise target unchanged */
    }
}

static void check_sequence_end(RoadApp *app, float dt)
{
    bool stable = false;
    if (app->out.present && app->out.braking) {
        app->did_brake = true;
    } else {
        /* keep previous did_brake */
    }
    stable = app->out.present && app->did_brake && app->out.lead_valid
          && (magnitude(app->range_rate) < ROAD_STABLE_RATE_MPS)
          && (magnitude(app->out.acceleration_mps2) < ROAD_STABLE_ACCEL_MPS2);
    if (stable) {
        app->settled_s = app->settled_s + dt;
    } else {
        app->settled_s = 0.0f;
    }
    if (app->out.present && app->did_brake && (app->out.speed_mps < ROAD_STOP_SPEED_MPS)) {
        app->out.speed_mps = 0.0f;
        app->out.target_mps = 0.0f;
        app->out.done = true;
        app->out.done_reason = ROAD_DONE_STOPPED;
    } else if (app->settled_s > ROAD_SETTLE_TIME_S) {
        app->out.done = true;
        app->out.done_reason = ROAD_DONE_SETTLED;
    } else {
        /* sequence still running */
    }
    if (app->out.done) {
        app->out.accel_blocked = false;
    } else {
        /* warning state unchanged */
    }
}

static void run_control_step(RoadApp *app, const RoadInputs *input, float old, float dt)
{
    float free_accel = 0.0f;
    app->out.acceleration_mps2 = idm_with_desired(old, app->out.target_mps,
                       app->out.gap_m, app->out.present, app->out.desired_gap_m);
    /* Free-road IDM: used to attribute braking to the lead vehicle. */
    free_accel = idm_with_desired(old, app->out.target_mps,
                       app->out.gap_m, false, app->out.desired_gap_m);
    update_accel_warning(app, input, old);
    integrate_motion(app, old, dt);
    follow_cruise(app, free_accel);
    check_sequence_end(app, dt);
}

void RoadApp_Tick(RoadApp *app, const RoadInputs *input, uint32_t now_ms)
{
    const float dt = (float)ROAD_TICK_MS * ROAD_MS_TO_S;
    bool reset = input->reset_pressed && (!app->previous_reset);
    bool adc_ok = input->ldr_ready && (input->ldr_adc <= ROAD_ADC_MAX)
               && ((uint32_t)(now_ms - input->ldr_ms) <= ROAD_SENSOR_STALE_MS);
    bool range_ok = input->range_ready && (input->range_status <= ROAD_RANGE_NO_ECHO)
                 && ((uint32_t)(now_ms - input->range_ms) <= ROAD_SENSOR_STALE_MS);
    float old = 0.0f;
    float closing = 0.0f;

    app->out.fault = (!adc_ok) || (!range_ok);
    if (adc_ok) {
        update_light(app, input);
    } else {
        app->light_ms = 0U;
    }
    update_range(app, input);
    if (reset) {
        reset_sequence(app);
    } else {
        /* no reset edge */
    }
    if ((!app->out.done) && (!app->out.fault) && (!reset)) {
        handle_speed_buttons(app, input);
    } else {
        /* buttons ignored while finished, faulted or resetting */
    }
    app->previous_up = input->speed_up_pressed;
    app->previous_down = input->speed_down_pressed;
    app->previous_reset = input->reset_pressed;
    app->out.target_mps = limit(app->out.target_mps, 0.0f, speed_cap_mps(app->out.night));

    old = app->out.speed_mps;
    if (app->out.lead_valid) {
        closing = -app->range_rate;
    } else {
        closing = old;
    }
    app->out.desired_gap_m = Road_DesiredGap(old, closing, app->out.night);
    update_red_target(app, input, adc_ok && range_ok);

    if (app->out.fault || app->out.done || reset) {
        freeze_outputs(app);
    } else {
        run_control_step(app, input, old, dt);
    }
}
