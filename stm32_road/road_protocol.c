#include "road_protocol.h"

typedef struct { char *data; size_t used, capacity; bool good; } Writer;
static void put(Writer *w, char c)
{
    if (w->used < w->capacity) { w->data[w->used++] = c; }
    else { w->good = false; }
}
static void number(Writer *w, uint32_t value)
{
    char reverse[10];
    size_t n = 0U;
    do { reverse[n++] = (char)('0'+(value%10U)); value /= 10U; } while (value != 0U);
    while (n != 0U) { put(w, reverse[--n]); }
}
static void field(Writer *w, int32_t value)
{
    put(w, ',');
    if (value < 0) { put(w, '-'); number(w, 0U-(uint32_t)value); }
    else { number(w, (uint32_t)value); }
}
static void unsigned_field(Writer *w, uint32_t value)
{
    put(w, ','); number(w, value);
}
size_t RoadProtocol_Encode(char *buffer, size_t capacity, const RoadOutput *out,
                           const RoadInputs *input, uint32_t seq, uint32_t ms)
{
    if (buffer == NULL || out == NULL || input == NULL) { return 0U; }
    Writer w = {buffer, 0U, capacity, true};
    uint32_t flags = (out->night ? 1U : 0U) | (out->present ? 2U : 0U)
                   | (out->lead_valid ? 4U : 0U) | (out->braking ? 8U : 0U)
                   | (out->done ? 16U : 0U) | (out->fault ? 32U : 0U);
    put(&w, 'R'); put(&w, 'D'); put(&w, '1');
    unsigned_field(&w, seq);
    unsigned_field(&w, ms);
    field(&w, (int32_t)(out->speed_mps*1000.0f));
    field(&w, (int32_t)(out->target_mps*1000.0f));
    field(&w, (int32_t)(out->acceleration_mps2*1000.0f));
    unsigned_field(&w, (uint32_t)(out->odometer_m*1000.0f));
    field(&w, out->present ? (int32_t)(out->gap_m*1000.0f) : -1);
    field(&w, out->lead_valid ? (int32_t)(out->lead_speed_mps*1000.0f) : 0);
    unsigned_field(&w, flags);
    unsigned_field(&w, input->ldr_adc);
    field(&w, (input->range_ready && input->range_status == 0U)
              ? (int32_t)input->distance_mm : -1);
    unsigned_field(&w, out->reset_id);
    unsigned_field(&w, out->done_reason);
    unsigned_field(&w, input->range_ready ? input->range_status : 2U);
    if (!w.good) { return 0U; }
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i=0U; i<w.used; ++i) {
        crc ^= (uint8_t)buffer[i];
        for (uint32_t bit=0U; bit<8U; ++bit) {
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    crc ^= 0xFFFFFFFFU;
    put(&w, '*');
    const char hex[] = "0123456789ABCDEF";
    for (uint32_t i=0U; i<8U; ++i) { put(&w, hex[(crc >> (28U-4U*i)) & 15U]); }
    put(&w, '\n');
    return w.good ? w.used : 0U; /* length-based DMA, no NUL needed */
}
