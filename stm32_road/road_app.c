#include "road_app.h"
#include "road_config.h"

static float limit(float x, float low, float high)
{
    return (x < low) ? low : ((x > high) ? high : x);
}
static float magnitude(float x) { return (x < 0.0f) ? -x : x; }

float Road_Idm(float speed, float target, float gap, float closing,
               bool present, bool night)
{
    float interaction = 0.0f;
    if (target <= 0.0f) { return (speed > 0.0f) ? -3.0f : 0.0f; }
    if (present) {
        /* sqrt(3.0 * 3.5) evaluated as a constant; delta = 4. */
        float dynamic = speed * (night ? 1.8f : 1.0f)
                      + speed * closing / 6.480740698f;
        float desired = (night ? 7.0f : 4.0f) + limit(dynamic, 0.0f, 100000.0f);
        float ratio = desired / limit(gap, ROAD_MIN_GAP_M, ROAD_MAX_GAP_M);
        interaction = ratio * ratio;
    }
    float ratio = speed / target;
    float square = ratio * ratio;
    return limit(3.0f * (1.0f - square * square - interaction), -7.0f, 3.0f);
}

void RoadApp_Init(RoadApp *app)
{
    *app = (RoadApp){0};
    app->out.target_mps = 50.0f / 3.6f;
    app->out.gap_m = ROAD_MAX_GAP_M;
    app->out.fault = true; /* Do not move until first sensor completions. */
}

static void reset_sequence(RoadApp *app)
{
    app->out.speed_mps = 0.0f;
    app->out.acceleration_mps2 = 0.0f;
    app->out.odometer_m = 0.0f;
    app->out.done = app->out.braking = app->did_brake = false;
    app->out.done_reason = 0U;
    app->out.reset_id++;
    app->settled_s = 0.0f;
    app->hold_up_ms = app->hold_down_ms = 0U;
    /* A physical object does NOT disappear on reset; retain measured gap.
     * Preserve cruise setting and LDR mode. */
}

static void update_light(RoadApp *app, const RoadInputs *input)
{
    uint32_t dark = ROAD_LDR_DARK_IS_HIGH ? input->ldr_adc : (4095U-input->ldr_adc);
    bool crossing = app->out.night ? (dark <= ROAD_NIGHT_EXIT_ADC)
                                  : (dark >= ROAD_NIGHT_ENTER_ADC);
    app->light_ms = crossing ? app->light_ms + ROAD_TICK_MS : 0U;
    uint32_t dwell = app->out.night ? ROAD_NIGHT_EXIT_MS : ROAD_NIGHT_ENTER_MS;
    if (app->light_ms >= dwell) {
        app->out.night = !app->out.night;
        app->light_ms = 0U;
    }
}

static void update_range(RoadApp *app, const RoadInputs *input)
{
    if (!input->range_ready || input->range_sequence == app->seen_range) { return; }
    app->seen_range = input->range_sequence;
    if (input->range_status != 0U) {
        app->out.present = app->out.lead_valid = app->have_previous_range = false;
        app->range_rate = app->estimate_age = app->settled_s = 0.0f;
        app->did_brake = false;
        return;
    }
    float gap = limit((float)input->distance_mm * ROAD_METRES_PER_SENSOR_MM,
                      ROAD_MIN_GAP_M, ROAD_MAX_GAP_M);
    float dt = (float)(input->range_ms-app->previous_range_ms) * 0.001f;
    float rate = (dt > 0.0f) ? (gap-app->previous_gap)/dt : 0.0f;
    if (!app->have_previous_range || dt <= 0.0f || dt > 0.25f || magnitude(rate) > 80.0f) {
        app->out.lead_valid = false;
        app->range_rate = app->estimate_age = app->settled_s = 0.0f;
    } else {
        app->estimate_age += dt;
        app->range_rate += (rate-app->range_rate) * dt/(0.14f+dt);
        app->out.lead_valid = (app->estimate_age >= 0.12f);
    }
    app->out.present = app->have_previous_range = true;
    app->out.gap_m = app->previous_gap = gap;
    app->previous_range_ms = input->range_ms;
}

static float button_delta(bool pressed, bool previous, uint32_t *held_ms)
{
    if (!pressed) { *held_ms = 0U; return 0.0f; }
    if (!previous) { *held_ms = 0U; return ROAD_TAP_MPS; }
    if (*held_ms < ROAD_HOLD_DELAY_MS) { *held_ms += ROAD_TICK_MS; }
    return (*held_ms >= ROAD_HOLD_DELAY_MS)
         ? ROAD_HOLD_MPS_PER_SEC * ((float)ROAD_TICK_MS * 0.001f) : 0.0f;
}

void RoadApp_Tick(RoadApp *app, const RoadInputs *input, uint32_t now_ms)
{
    const float dt = (float)ROAD_TICK_MS * 0.001f;
    bool reset = input->reset_pressed && !app->previous_reset;
    bool adc_ok = input->ldr_ready && input->ldr_adc <= 4095U
               && (uint32_t)(now_ms-input->ldr_ms) <= ROAD_SENSOR_STALE_MS;
    bool range_ok = input->range_ready && input->range_status <= 1U
                 && (uint32_t)(now_ms-input->range_ms) <= ROAD_SENSOR_STALE_MS;
    app->out.fault = !adc_ok || !range_ok;
    if (adc_ok) { update_light(app, input); } else { app->light_ms = 0U; }
    update_range(app, input);
    if (reset) { reset_sequence(app); }
    if (!app->out.done && !app->out.fault && !reset) {
        /* Both speed buttons together are neutral. No conflicting increments. */
        if (input->speed_up_pressed && input->speed_down_pressed) {
            app->hold_up_ms = app->hold_down_ms = 0U;
        } else {
            app->out.target_mps += button_delta(input->speed_up_pressed,
                                      app->previous_up, &app->hold_up_ms);
            app->out.target_mps -= button_delta(input->speed_down_pressed,
                                      app->previous_down, &app->hold_down_ms);
        }
    }
    app->previous_up = input->speed_up_pressed;
    app->previous_down = input->speed_down_pressed;
    app->previous_reset = input->reset_pressed;
    app->out.target_mps = limit(app->out.target_mps, 0.0f,
                                (app->out.night ? 100.0f : 120.0f)/3.6f);
    if (app->out.fault || app->out.done || reset) {
        app->out.acceleration_mps2 = 0.0f;
        app->out.braking = false;
        if (app->out.fault) {
            app->have_previous_range = app->out.lead_valid = false;
            app->hold_up_ms = app->hold_down_ms = 0U;
        }
        return; /* Freeze simulation on sensor fault; never assume free road. */
    }
    float old = app->out.speed_mps;
    float closing = app->out.lead_valid ? -app->range_rate : old;
    app->out.acceleration_mps2 = Road_Idm(old, app->out.target_mps,
                       app->out.gap_m, closing, app->out.present, app->out.night);
    app->out.speed_mps = limit(old+app->out.acceleration_mps2*dt, 0.0f, 40.0f);
    app->out.odometer_m += (old+app->out.speed_mps)*0.5f*dt;
    if (app->out.odometer_m >= 4000000.0f) { app->out.odometer_m = 0.0f; }
    app->out.lead_speed_mps = app->out.speed_mps + app->range_rate;
    app->out.braking = app->out.acceleration_mps2 < -0.35f;
    if (app->out.present && app->out.braking) { app->did_brake = true; }
    bool stable = app->out.present && app->did_brake && app->out.lead_valid
               && magnitude(app->range_rate)<0.5f && magnitude(app->out.acceleration_mps2)<0.4f;
    app->settled_s = stable ? app->settled_s+dt : 0.0f;
    if (app->out.present && app->did_brake && app->out.speed_mps < 0.3f) {
        app->out.speed_mps = 0.0f;
        app->out.done = true;
        app->out.done_reason = 1U;
    } else if (app->settled_s > 1.2f) {
        app->out.done = true;
        app->out.done_reason = 2U;
    }
}
