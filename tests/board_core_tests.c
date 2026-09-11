/* Runs the actual C application and encoder under ARM emulation, no GPIO mocks
 * are treated as hardware validation. Return zero or the failing source line. */
#include "road_app.h"
#include "road_config.h"
#include "road_protocol.h"
#define CHECK(c) do { if (!(c)) { return __LINE__; } } while (0)
char g_test_frame[ROAD_FRAME_CAPACITY];
uint32_t g_test_frame_size;

static void step(RoadApp *a, RoadInputs *i, uint32_t *ms, uint32_t count)
{
    for (uint32_t n=0U; n<count; ++n) {
        *ms += ROAD_TICK_MS;
        i->ldr_ms = *ms;
        if ((*ms%60U)==0U || !i->range_ready) {
            i->range_ms=*ms; i->range_sequence++; i->range_ready=true;
        }
        RoadApp_Tick(a,i,*ms);
    }
}
int run_tests(void);
int run_tests(void)
{
    RoadApp a;
    RoadInputs i = {0};
    uint32_t ms=0U;
    RoadApp_Init(&a);
    RoadApp_Tick(&a,&i,10U);
    CHECK(a.out.fault && a.out.speed_mps==0.0f);
    i.ldr_ready=true; i.ldr_adc=1600U; i.range_status=1U;
    step(&a,&i,&ms,10U);
    CHECK(!a.out.fault && a.out.speed_mps>0.0f && !a.out.night);
    float initial=a.out.target_mps;
    i.speed_up_pressed=true;
    step(&a,&i,&ms,1U);
    CHECK(a.out.target_mps>initial+0.49f && a.out.target_mps<initial+0.51f);
    i.speed_up_pressed=false; step(&a,&i,&ms,4U);
    CHECK(a.out.target_mps<initial+0.51f);
    i.speed_up_pressed=true; step(&a,&i,&ms,100U);
    CHECK(a.out.target_mps>initial+3.0f && a.out.target_mps<initial+3.8f);
    i.speed_down_pressed=true; initial=a.out.target_mps;
    step(&a,&i,&ms,100U); CHECK(a.out.target_mps==initial);
    i.speed_up_pressed=false; i.speed_down_pressed=false;
    step(&a,&i,&ms,1U);
    i.speed_down_pressed=true; step(&a,&i,&ms,1U);
    CHECK(a.out.target_mps<initial-0.49f);
    i.speed_down_pressed=false;

    a.out.target_mps=120.0f/3.6f;
    i.ldr_adc=3900U; step(&a,&i,&ms,39U); CHECK(!a.out.night);
    step(&a,&i,&ms,1U); CHECK(a.out.night && a.out.target_mps<=100.0f/3.6f);
    i.ldr_adc=3650U; step(&a,&i,&ms,80U); CHECK(a.out.night);
    i.ldr_adc=3200U; step(&a,&i,&ms,19U); CHECK(a.out.night);
    step(&a,&i,&ms,1U); CHECK(!a.out.night);
    i.ldr_adc=3790U; step(&a,&i,&ms,100U); CHECK(!a.out.night);

    /* Constant measured range: signed lead estimate matches ego speed. */
    i.range_status=0U; i.distance_mm=1000U;
    step(&a,&i,&ms,150U);
    CHECK(a.out.present && a.out.lead_valid && a.out.gap_m==50.0f);
    CHECK(a.out.lead_speed_mps-a.out.speed_mps<0.001f);
    i.range_status=1U; step(&a,&i,&ms,6U);
    CHECK(!a.out.present && !a.out.lead_valid);

    /* Close sudden appearance: conservative lead unknown, eventual STOP. */
    i.range_status=0U; i.distance_mm=40U;
    step(&a,&i,&ms,1000U);
    CHECK(a.out.done && a.out.done_reason==1U && a.out.speed_mps==0.0f);
    float target=a.out.target_mps;
    i.reset_pressed=true; step(&a,&i,&ms,1U);
    CHECK(!a.out.done && a.out.reset_id==1U && a.out.speed_mps==0.0f);
    CHECK(a.out.present && a.out.target_mps==target); /* no fake object removal */
    step(&a,&i,&ms,5U); CHECK(a.out.reset_id==1U); /* holding reset: once */
    i.reset_pressed=false; i.range_status=1U; step(&a,&i,&ms,30U);
    CHECK(a.out.done); /* object was still there after reset, so it stopped again */
    i.reset_pressed=true; step(&a,&i,&ms,1U);
    CHECK(a.out.reset_id==2U && a.out.speed_mps==0.0f);
    i.reset_pressed=false;
    step(&a,&i,&ms,30U); CHECK(!a.out.done && a.out.speed_mps>0.0f);
    i.reset_pressed=true; step(&a,&i,&ms,1U);
    CHECK(a.out.reset_id==3U && a.out.speed_mps==0.0f); /* reset while running */
    i.reset_pressed=false;
    a.out.target_mps=0.0f; step(&a,&i,&ms,100U);
    CHECK(!a.out.done && a.out.speed_mps==0.0f); /* user zero is not BREAK */

    /* Stale ADC / bad echo freeze instead of silently accelerating. */
    a.out.target_mps=20.0f; step(&a,&i,&ms,50U);
    float speed=a.out.speed_mps, odo=a.out.odometer_m;
    RoadApp_Tick(&a,&i,ms+1000U);
    CHECK(a.out.fault && a.out.speed_mps==speed && a.out.odometer_m==odo);
    i.range_status=2U; step(&a,&i,&ms,50U); CHECK(a.out.fault);
    i.range_status=1U; step(&a,&i,&ms,50U); CHECK(!a.out.fault);
    CHECK(Road_Idm(30.0f,10.0f,2.0f,30.0f,true,true)>=-7.0f);
    CHECK(Road_Idm(0.0f,30.0f,100.0f,0.0f,false,false)==3.0f);

    /* Derivative uses sample time, not render/control frequency. */
    RoadApp_Init(&a); ms=0U; i=(RoadInputs){0};
    i.ldr_ready=true; i.ldr_adc=1600U; i.range_status=0U; i.distance_mm=1200U;
    for (uint32_t n=0U;n<100U;++n) {
        i.distance_mm-=2U; step(&a,&i,&ms,6U);
    }
    CHECK(a.out.lead_valid);
    float relative=a.out.lead_speed_mps-a.out.speed_mps;
    CHECK(relative < -1.60f && relative > -1.73f);
    i.distance_mm=40U; step(&a,&i,&ms,6U);
    CHECK(!a.out.lead_valid); /* discontinuous acquisition does not invent speed */

    RoadApp_Init(&a); ms=0U; i=(RoadInputs){0};
    i.ldr_ready=true; i.ldr_adc=1600U; i.range_status=0U; i.distance_mm=1000U;
    a.out.speed_mps=30.0f; a.out.target_mps=30.0f;
    step(&a,&i,&ms,6000U);
    CHECK(a.out.done && a.out.done_reason==2U && a.out.speed_mps>0.3f);

    /* Generate a real C frame to be parsed/CRC checked by Python. */
    a.out.night=true; a.out.target_mps=25.0f; a.out.speed_mps=20.0f;
    a.out.acceleration_mps2=-2.0f; a.out.odometer_m=123.0f;
    a.out.present=true; a.out.gap_m=20.0f; a.out.lead_valid=true;
    a.out.lead_speed_mps=12.0f; a.out.done=false; a.out.done_reason=0U;
    a.out.braking=true; a.out.fault=false;
    i.ldr_adc=3900U; i.distance_mm=400U;
    g_test_frame_size=(uint32_t)RoadProtocol_Encode(g_test_frame,sizeof g_test_frame,&a.out,&i,42U,12345U);
    CHECK(g_test_frame_size>0U && g_test_frame_size<ROAD_FRAME_CAPACITY);
    char tiny[5]; CHECK(RoadProtocol_Encode(tiny,sizeof tiny,&a.out,&i,0U,0U)==0U);
    return 0;
}
