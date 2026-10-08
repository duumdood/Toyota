/* Replace the generated main.c in a NEW STM32F411RE CMSIS/Empty project.
 * Retain ST startup_stm32f411retx.s, system_stm32f4xx.c and linker script.
 * Do NOT call HAL_Init(), SystemClock_Config(), MX_* or another driver init.
 * Only one definition of each IRQ handler may be linked (see README). */
#ifndef STM32F411xE
#define STM32F411xE
#endif

#include "stm32f4xx.h"
#include "road_board.h"

#define FPU_CP10_CP11_FULL_ACCESS  (15U << 20U) /* CPACR bits 20..23 */
#define RCC_CLOCK_TREE_MASK        (RCC_CFGR_SWS | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)

int main(void)
{
    SCB->CPACR |= FPU_CP10_CP11_FULL_ACCESS; /* Cortex-M4 single-precision FPU */
    __DSB();
    __ISB();
    /* Fail visibly in debugger if startup changed the reset clock assumptions. */
    if ((RCC->CFGR & RCC_CLOCK_TREE_MASK) != 0U) {
        __disable_irq();
        for (;;) {
            __WFI();
        }
    } else {
        RoadBoard_Init();
    }
    for (;;) {
        __WFI(); /* CPU sleeps: peripherals/interrupts do all work. */
    }
}
