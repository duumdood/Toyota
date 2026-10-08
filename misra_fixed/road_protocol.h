#ifndef ROAD_PROTOCOL_H
#define ROAD_PROTOCOL_H
#include <stddef.h>
#include "road_app.h"
#define ROAD_FRAME_CAPACITY 240U
/* RD1 CSV of integer SI-scaled outputs, followed by *CRC32 and newline.
 * CRC is standard reflected IEEE CRC32, compatible with Python zlib.crc32. */
size_t RoadProtocol_Encode(char *buffer, size_t capacity, const RoadOutput *out,
                           const RoadInputs *input, uint32_t seq, uint32_t ms);
#endif
