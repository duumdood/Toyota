/* Bare-metal CMSIS register style, following lectures 0100 / 0300 / 0400 /
 * 0500. No HAL, no peripheral busy-wait loops, no ADC/UART polling.
 * Owns ADC1, USART2, DMA1 Stream6, TIM3, TIM4 and EXTI3/4/10 exclusively.
 * Clock assumption: RESET HSI 16 MHz, all bus prescalers /1. */
#ifndef STM32F411xE
#define STM32F411xE
#endif

#include "stm32f4xx.h"
#include "road_board.h"
#include "road_config.h"
#include "road_protocol.h"

volatile RoadInputs g_road_inputs;
volatile RoadOutput g_road_output;
volatile uint32_t g_road_ms;
volatile uint32_t g_uart_dropped_frames;
volatile uint32_t g_uart_dma_errors;
static RoadApp app;
static char tx_buffer[ROAD_FRAME_CAPACITY];
static volatile bool tx_busy;
static volatile bool adc_busy;
static uint32_t packet_sequence;
static bool waiting_echo, saw_rise;
static uint32_t echo_rise;
typedef struct { bool pending; uint32_t edge_ms; } ButtonEdge;
static volatile ButtonEdge buttons[3];

/* Handlers have fixed CMSIS vector names, as in lecture 0500. */
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void ADC_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);

static bool pin_pressed(GPIO_TypeDef *port, uint32_t pin)
{
    bool high = (port->IDR & (1U << pin)) != 0U;
    return ROAD_BUTTON_ACTIVE_LOW ? !high : high;
}
static void edge(uint32_t index)
{
    buttons[index].edge_ms = g_road_ms;
    buttons[index].pending = true; /* Restart debounce on EVERY edge. */
}
void EXTI15_10_IRQHandler(void)
{
    if ((EXTI->PR & (1U<<10U)) != 0U) { EXTI->PR = 1U<<10U; edge(0U); }
}
void EXTI3_IRQHandler(void)
{
    if ((EXTI->PR & (1U<<3U)) != 0U) { EXTI->PR = 1U<<3U; edge(1U); }
}
void EXTI4_IRQHandler(void)
{
    if ((EXTI->PR & (1U<<4U)) != 0U) { EXTI->PR = 1U<<4U; edge(2U); }
}
static void debounce(void)
{
    /* Short atomic confirm prevents an EXTI edge racing the deadline test.
     * GPIO is sampled ONLY after an edge/deadline, not continuously polled. */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    for (uint32_t i=0U; i<3U; ++i) {
        if (buttons[i].pending && (uint32_t)(g_road_ms-buttons[i].edge_ms)>=ROAD_DEBOUNCE_MS) {
            buttons[i].pending = false;
            if (i==0U) { g_road_inputs.speed_up_pressed = pin_pressed(GPIOA,10U); }
            else if (i==1U) { g_road_inputs.speed_down_pressed = pin_pressed(GPIOB,3U); }
            else { g_road_inputs.reset_pressed = pin_pressed(GPIOB,4U); }
        }
    }
    __set_PRIMASK(mask);
}
void ADC_IRQHandler(void)
{
    uint32_t status = ADC1->SR;
    if ((status & ADC_SR_OVR) != 0U) {
        (void)ADC1->DR;
        ADC1->SR = ~(ADC_SR_OVR | ADC_SR_EOC);
        g_road_inputs.ldr_ready = false;
        adc_busy = false;
    } else if ((status & ADC_SR_EOC) != 0U) {
        g_road_inputs.ldr_adc = (uint16_t)(ADC1->DR & 4095U);
        g_road_inputs.ldr_ms = g_road_ms;
        g_road_inputs.ldr_ready = true;
        adc_busy = false;
    }
}
static void range_complete(uint8_t status, uint32_t mm)
{
    g_road_inputs.distance_mm = mm;
    g_road_inputs.range_status = status;
    g_road_inputs.range_ms = g_road_ms;
    g_road_inputs.range_sequence++;
    g_road_inputs.range_ready = true;
    waiting_echo = false;
}
void TIM3_IRQHandler(void)
{
    uint32_t status = TIM3->SR;
    /* rc_w0: write zero ONLY to observed flags; preserve newly arrived flags. */
    TIM3->SR = ~status;
    if ((status & TIM_SR_UIF) != 0U) { waiting_echo = true; saw_rise = false; }
    if ((status & (TIM_SR_CC1OF | TIM_SR_CC2OF)) != 0U) {
        (void)TIM3->CCR1; (void)TIM3->CCR2;
        range_complete(2U,0U);
    } else {
        if ((status & TIM_SR_CC1IF) != 0U) {
            uint32_t value = TIM3->CCR1;
            if (waiting_echo) { echo_rise = value; saw_rise = true; }
        }
        if ((status & TIM_SR_CC2IF) != 0U) {
            uint32_t fall = TIM3->CCR2;
            if (waiting_echo && saw_rise) {
                uint32_t width = (fall+ROAD_ECHO_PERIOD_US-echo_rise)%ROAD_ECHO_PERIOD_US;
                if (width>=ROAD_ECHO_MIN_US && width<=ROAD_ECHO_MAX_US) {
                    range_complete(0U,(width*343U+1000U)/2000U);
                } else { range_complete(2U,0U); }
            }
        }
        if ((status & TIM_SR_CC4IF) != 0U && waiting_echo) {
            /* Missing echo cannot distinguish open space from disconnected
             * sensor. Status=1 exposes this limitation, not "sensor healthy". */
            range_complete(1U,0U);
        }
    }
}
void DMA1_Stream6_IRQHandler(void)
{
    uint32_t status = DMA1->HISR;
    DMA1->HIFCR = DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6
                | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6;
    if ((status & (DMA_HISR_TEIF6 | DMA_HISR_DMEIF6 | DMA_HISR_FEIF6)) != 0U) {
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        g_uart_dma_errors++;
    }
    if ((status & (DMA_HISR_TCIF6 | DMA_HISR_TEIF6 | DMA_HISR_DMEIF6 | DMA_HISR_FEIF6)) != 0U) {
        tx_busy = false;
    }
}
static void transmit(const RoadInputs *snapshot)
{
    /* Never change storage still owned by DMA. Dropping a whole telemetry
     * frame is preferable to blocking the control loop or interleaving bytes. */
    if (tx_busy || (DMA1_Stream6->CR & DMA_SxCR_EN) != 0U) {
        g_uart_dropped_frames++; return;
    }
    size_t n = RoadProtocol_Encode(tx_buffer, sizeof tx_buffer, &app.out,
                                   snapshot, packet_sequence++, g_road_ms);
    if (n==0U) { g_uart_dropped_frames++; return; }
    DMA1->HIFCR = DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6
                | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6;
    DMA1_Stream6->M0AR = (uint32_t)(uintptr_t)tx_buffer;
    DMA1_Stream6->NDTR = (uint32_t)n;
    tx_busy = true;
    __DMB();
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}
void TIM4_IRQHandler(void)
{
    if ((TIM4->SR & TIM_SR_UIF) == 0U) { return; }
    TIM4->SR = ~TIM_SR_UIF;
    g_road_ms++;
    debounce();
    if ((g_road_ms % ROAD_TICK_MS) == 0U) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        RoadInputs snapshot = g_road_inputs;
        __set_PRIMASK(mask);
        RoadApp_Tick(&app, &snapshot, g_road_ms);
        g_road_output = app.out;
        GPIOA->BSRR = app.out.night ? (1U<<5U) : (1U<<21U);
        if ((g_road_ms % ROAD_TELEMETRY_MS) == 0U) { transmit(&snapshot); }
        if (!adc_busy) { adc_busy = true; ADC1->CR2 |= ADC_CR2_SWSTART; }
    }
}
void RoadBoard_Init(void)
{
    RoadApp_Init(&app);
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR; /* clock-enable propagation */

    /* Shield: PA10 speed+, PB3 speed-, PB4 reset. Keep SWD PA13/14;
     * disable SWO/JTAG use of PB3/PB4 by configuring GPIO input. */
    GPIOA->MODER &= ~(3U<<20U);
    GPIOB->MODER &= ~((3U<<6U)|(3U<<8U));
    uint32_t pull = ROAD_BUTTON_ACTIVE_LOW ? 1U : 2U;
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3U<<20U)) | (pull<<20U);
    GPIOB->PUPDR = (GPIOB->PUPDR & ~((3U<<6U)|(3U<<8U))) | (pull<<6U)|(pull<<8U);
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~(15U<<12U)) | (1U<<12U); /* PB3 */
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~15U) | 1U; /* PB4 */
    SYSCFG->EXTICR[2] &= ~(15U<<8U); /* PA10 */
    uint32_t lines = (1U<<3U)|(1U<<4U)|(1U<<10U);
    EXTI->PR = lines;
    EXTI->RTSR |= lines; EXTI->FTSR |= lines; EXTI->IMR |= lines;
    for (uint32_t i=0U; i<3U; ++i) { edge(i); } /* include buttons held at boot */

    GPIOA->MODER = (GPIOA->MODER & ~(3U<<10U)) | (1U<<10U); /* PA5 LED */
    GPIOA->OTYPER &= ~(1U<<5U);
    GPIOA->BSRR = 1U<<21U;
    GPIOA->MODER |= 3U<<2U; /* PA1 analog */
    GPIOA->PUPDR &= ~(3U<<2U);
    ADC->CCR &= ~ADC_CCR_ADCPRE; /* PCLK2/2 = 8 MHz */
    ADC1->CR1 = ADC_CR1_EOCIE | ADC_CR1_OVRIE;
    ADC1->SMPR2 = 7U<<3U; /* channel 1, 480-cycle sample for LDR */
    ADC1->SQR1 = 0U; /* L encodes N-1: ZERO means ONE conversion */
    ADC1->SQR2 = 0U; ADC1->SQR3 = 1U;
    ADC1->CR2 = ADC_CR2_ADON; /* first conversion after 10ms startup settling */

    /* USART2 TX PA2 AF7 -> ST-LINK virtual COM, 115200 8N1, DMA only.
     * PA3 RX intentionally unused: the PC cannot command the board. */
    GPIOA->MODER = (GPIOA->MODER & ~(3U<<4U)) | (2U<<4U);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(15U<<8U)) | (7U<<8U);
    GPIOA->OTYPER &= ~(1U<<2U);
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U<<4U)) | (2U<<4U);
    USART2->CR1 = 0U; USART2->CR2 = 0U;
    USART2->BRR = (ROAD_CLOCK_HZ + ROAD_UART_BAUD/2U)/ROAD_UART_BAUD;
    USART2->CR3 = USART_CR3_DMAT;
    DMA1_Stream6->CR = (4U<<DMA_SxCR_CHSEL_Pos) | DMA_SxCR_DIR_0 | DMA_SxCR_MINC
                    | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    DMA1_Stream6->FCR = 0U;
    DMA1_Stream6->PAR = (uint32_t)(uintptr_t)&USART2->DR;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;

    /* US-100, jumper REMOVED, 5V supply for the tested module, common ground.
     * Morpho PC6 = ECHO / TIM3_CH1 AF2; PC8 = TRIG / TIM3_CH3 AF2.
     * Avoid PA0 (shield NTC) and PC7 (shield 7-segment decoder).
     * Confirm these Morpho pins are accessible on your shield revision. */
    GPIOC->MODER = (GPIOC->MODER & ~((3U<<12U)|(3U<<16U))) | (2U<<12U)|(2U<<16U);
    GPIOC->AFR[0] = (GPIOC->AFR[0] & ~(15U<<24U)) | (2U<<24U);
    GPIOC->AFR[1] = (GPIOC->AFR[1] & ~15U) | 2U;
    GPIOC->OTYPER &= ~(1U<<8U);
    /* PC6 is an FT digital input. Keep its internal pulls disabled when the
     * tested 5V-powered US-100 drives ECHO above the MCU supply voltage. */
    GPIOC->PUPDR &= ~((3U<<12U)|(3U<<16U));
    TIM3->PSC = ROAD_CLOCK_HZ/1000000U-1U;
    TIM3->ARR = ROAD_ECHO_PERIOD_US-1U;
    /* CH1 rising direct TI1, CH2 falling indirect TI1. Hardware timestamps. */
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1 | (3U<<TIM_CCMR1_IC1F_Pos) | (3U<<TIM_CCMR1_IC2F_Pos);
    TIM3->CCMR2 = (6U<<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE; /* PWM1 */
    TIM3->CCR3 = ROAD_ECHO_TRIGGER_US;
    TIM3->CCR4 = ROAD_ECHO_TIMEOUT_US;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC3E;
    TIM3->EGR = TIM_EGR_UG; TIM3->SR = 0U;
    TIM3->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC4IE;
    waiting_echo = true;

    TIM4->PSC = ROAD_CLOCK_HZ/1000000U-1U;
    TIM4->ARR = 999U;
    TIM4->EGR = TIM_EGR_UG; TIM4->SR = 0U;
    TIM4->DIER = TIM_DIER_UIE;
    NVIC_SetPriorityGrouping(0U);
    NVIC_SetPriority(TIM3_IRQn,0U);
    NVIC_SetPriority(ADC_IRQn,1U);
    NVIC_SetPriority(EXTI3_IRQn,1U);
    NVIC_SetPriority(EXTI4_IRQn,1U);
    NVIC_SetPriority(EXTI15_10_IRQn,1U);
    NVIC_SetPriority(DMA1_Stream6_IRQn,2U);
    NVIC_SetPriority(TIM4_IRQn,3U);
    NVIC_EnableIRQ(TIM3_IRQn); NVIC_EnableIRQ(ADC_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn); NVIC_EnableIRQ(EXTI4_IRQn); NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn); NVIC_EnableIRQ(TIM4_IRQn);
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM4->CR1 = TIM_CR1_CEN;
}
