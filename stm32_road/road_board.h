#ifndef ROAD_BOARD_H
#define ROAD_BOARD_H
#include "road_app.h"

/* Debugger-visible REAL INPUTS and computed OUTPUTS. Not manual overrides.
 * IRQs own g_road_inputs. Application snapshots it atomically every 10 ms. */
extern volatile RoadInputs g_road_inputs;
extern volatile RoadOutput g_road_output;
extern volatile uint32_t g_road_ms;
extern volatile uint32_t g_uart_dropped_frames;
extern volatile uint32_t g_uart_dma_errors;
void RoadBoard_Init(void);
#endif
