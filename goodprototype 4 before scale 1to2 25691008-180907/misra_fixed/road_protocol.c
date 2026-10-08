/* RD1 telemetry encoder: CSV of integer SI-scaled outputs + "*CRC32\n".
 * CRC is standard reflected IEEE CRC32, compatible with Python zlib.crc32. */
#include "road_protocol.h"
#include "road_config.h"

#define PROTO_SCALE          1000.0f /* SI value -> milli units on the wire */
#define PROTO_ABSENT         (-1)    /* field value when data is not valid */
#define PROTO_DECIMAL_BASE   10U
#define PROTO_MAX_DIGITS     10U     /* uint32_t has at most 10 decimal digits */
#define CRC_INITIAL          0xFFFFFFFFU
#define CRC_FINAL_XOR        0xFFFFFFFFU
#define CRC_POLY_REFLECTED   0xEDB88320U
#define CRC_LSB_MASK         1U
#define BITS_PER_BYTE        8U
#define CRC_HEX_DIGITS       8U
#define NIBBLE_BITS          4U
#define NIBBLE_MASK          15U
#define CRC_TOP_NIBBLE_SHIFT 28U

#define FLAG_NIGHT           1U
#define FLAG_PRESENT         2U
#define FLAG_LEAD_VALID      4U
#define FLAG_BRAKING         8U
#define FLAG_DONE            16U
#define FLAG_FAULT           32U
#define FLAG_ACCEL_BLOCKED   64U

typedef struct {
    char *data;
    size_t used;
    size_t capacity;
    bool good;
} Writer;

static const char hex_digits[] = "0123456789ABCDEF";

static void put(Writer *w, char c);
static void number(Writer *w, uint32_t value);
static void field(Writer *w, int32_t value);
static void unsigned_field(Writer *w, uint32_t value);
static uint32_t build_flags(const RoadOutput *out);
static int32_t scaled(float value);
static uint32_t crc32_of(const char *buffer, size_t length);
static void write_body(Writer *w, const RoadOutput *out,
                       const RoadInputs *input, uint32_t seq, uint32_t ms);

static void put(Writer *w, char c)
{
    if (w->used < w->capacity) {
        w->data[w->used] = c;
        w->used++;
    } else {
        w->good = false;
    }
}

static void number(Writer *w, uint32_t value)
{
    char reverse[PROTO_MAX_DIGITS] = {'0'};
    size_t n = 0U;
    uint32_t rest = value;
    do {
        reverse[n] = hex_digits[rest % PROTO_DECIMAL_BASE];
        n++;
        rest /= PROTO_DECIMAL_BASE;
    } while (rest != 0U);
    while (n != 0U) {
        n--;
        put(w, reverse[n]);
    }
}

static void field(Writer *w, int32_t value)
{
    put(w, ',');
    if (value < 0) {
        put(w, '-');
        number(w, 0U - (uint32_t)value);
    } else {
        number(w, (uint32_t)value);
    }
}

static void unsigned_field(Writer *w, uint32_t value)
{
    put(w, ',');
    number(w, value);
}

static uint32_t build_flags(const RoadOutput *out)
{
    uint32_t flags = 0U;
    if (out->night) {
        flags |= FLAG_NIGHT;
    } else {
        /* bit clear */
    }
    if (out->present) {
        flags |= FLAG_PRESENT;
    } else {
        /* bit clear */
    }
    if (out->lead_valid) {
        flags |= FLAG_LEAD_VALID;
    } else {
        /* bit clear */
    }
    if (out->braking) {
        flags |= FLAG_BRAKING;
    } else {
        /* bit clear */
    }
    if (out->done) {
        flags |= FLAG_DONE;
    } else {
        /* bit clear */
    }
    if (out->fault) {
        flags |= FLAG_FAULT;
    } else {
        /* bit clear */
    }
    if (out->accel_blocked) {
        flags |= FLAG_ACCEL_BLOCKED;
    } else {
        /* bit clear */
    }
    return flags;
}

static int32_t scaled(float value)
{
    return (int32_t)(value * PROTO_SCALE);
}

static uint32_t crc32_of(const char *buffer, size_t length)
{
    uint32_t crc = CRC_INITIAL;
    for (size_t i = 0U; i < length; ++i) {
        crc ^= (uint32_t)(uint8_t)buffer[i];
        for (uint32_t bit = 0U; bit < BITS_PER_BYTE; ++bit) {
            if ((crc & CRC_LSB_MASK) != 0U) {
                crc = (crc >> 1U) ^ CRC_POLY_REFLECTED;
            } else {
                crc = crc >> 1U;
            }
        }
    }
    return crc ^ CRC_FINAL_XOR;
}

static void write_body(Writer *w, const RoadOutput *out,
                       const RoadInputs *input, uint32_t seq, uint32_t ms)
{
    int32_t gap = PROTO_ABSENT;
    int32_t lead = 0;
    int32_t distance = PROTO_ABSENT;
    uint32_t range_status = ROAD_RANGE_FAULT;
    if (out->present) {
        gap = scaled(out->gap_m);
    } else {
        /* no object: send -1 */
    }
    if (out->lead_valid) {
        lead = scaled(out->lead_speed_mps);
    } else {
        /* lead speed unknown: send 0 */
    }
    if (input->range_ready && (input->range_status == ROAD_RANGE_VALID)) {
        distance = (int32_t)input->distance_mm;
    } else {
        /* no valid echo: send -1 */
    }
    if (input->range_ready) {
        range_status = (uint32_t)input->range_status;
    } else {
        /* sensor never completed: report fault */
    }
    put(w, 'R');
    put(w, 'D');
    put(w, '1');
    unsigned_field(w, seq);
    unsigned_field(w, ms);
    field(w, scaled(out->speed_mps));
    field(w, scaled(out->target_mps));
    field(w, scaled(out->acceleration_mps2));
    unsigned_field(w, (uint32_t)(out->odometer_m * PROTO_SCALE));
    field(w, gap);
    field(w, lead);
    unsigned_field(w, build_flags(out));
    unsigned_field(w, (uint32_t)input->ldr_adc);
    field(w, distance);
    unsigned_field(w, out->reset_id);
    unsigned_field(w, (uint32_t)out->done_reason);
    unsigned_field(w, range_status);
}

size_t RoadProtocol_Encode(char *buffer, size_t capacity, const RoadOutput *out,
                           const RoadInputs *input, uint32_t seq, uint32_t ms)
{
    size_t length = 0U;
    if ((buffer != NULL) && (out != NULL) && (input != NULL)) {
        Writer w = {buffer, 0U, capacity, true};
        write_body(&w, out, input, seq, ms);
        if (w.good) {
            uint32_t crc = crc32_of(buffer, w.used);
            put(&w, '*');
            for (uint32_t i = 0U; i < CRC_HEX_DIGITS; ++i) {
                uint32_t shift = CRC_TOP_NIBBLE_SHIFT - (NIBBLE_BITS * i);
                put(&w, hex_digits[(crc >> shift) & NIBBLE_MASK]);
            }
            put(&w, '\n');
            if (w.good) {
                length = w.used; /* length-based DMA, no NUL needed */
            } else {
                /* buffer too small: drop frame */
            }
        } else {
            /* buffer too small: drop frame */
        }
    } else {
        /* invalid arguments */
    }
    return length;
}
