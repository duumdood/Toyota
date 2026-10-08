/* Driver layer: bare-metal CMSIS register style, following lectures
 * 0100 / 0300 / 0400 / 0500. No HAL, no peripheral busy-wait loops,
 * no ADC/UART polling.
 * Owns ADC1, USART2, DMA1 Stream6, TIM2/3/4 and EXTI3/4/10 exclusively.
 * Clock assumption: RESET HSI 16 MHz, all bus prescalers /1.
 * MISRA checksheet version: register bit positions and pin numbers are
 * named below (Rule 5), no ternary operator (Rule 6), every if has else
 * (Rule 19), button decoding uses switch with default (Rules 20-22). */
#ifndef STM32F411xE
#define STM32F411xE
#endif

#include "stm32f4xx.h"
#include "road_board.h"
#include "road_config.h"
#include "road_protocol.h"

/* ------------------------------------------------------------ pin map */
#define PIN_LDR             1U   /* PA1  ADC1_IN1 */
#define PIN_UART_TX         2U   /* PA2  USART2_TX */
#define PIN_LED_NIGHT       5U   /* PA5  blue LED */
#define PIN_LED_RED         6U   /* PA6  red LED (software PWM) */
#define PIN_SEG_BIT1        8U   /* PA8  7-segment BCD bit1 */
#define PIN_SEG_BIT3        9U   /* PA9  7-segment BCD bit3 */
#define PIN_SPEED_UP        10U  /* PA10 button D2 */
#define PIN_SPEED_DOWN      3U   /* PB3  button D3 */
#define PIN_RESET           4U   /* PB4  button D5 */
#define PIN_SEG_BIT2        10U  /* PB10 7-segment BCD bit2 */
#define PIN_ECHO            6U   /* PC6  US-100 ECHO, TIM3_CH1 */
#define PIN_SEG_BIT0        7U   /* PC7  7-segment BCD bit0 */
#define PIN_TRIG            8U   /* PC8  US-100 TRIG, TIM3_CH3 */

/* ------------------------------------------------------ GPIO encoding */
#define GPIO_MODE_INPUT     0U
#define GPIO_MODE_OUTPUT    1U
#define GPIO_MODE_AF        2U
#define GPIO_MODE_ANALOG    3U
#define GPIO_PULL_NONE      0U
#define GPIO_PULL_UP        1U
#define GPIO_PULL_DOWN      2U
#define GPIO_SPEED_FAST     2U
#define GPIO_FIELD2_MASK    3U   /* 2-bit fields: MODER, PUPDR, OSPEEDR */
#define GPIO_FIELD2_BITS    2U
#define GPIO_AF_MASK        15U
#define GPIO_AF_BITS        4U
#define GPIO_AF_PER_REG     8U
#define GPIO_BSRR_RESET_OFS 16U
#define AF_TIM3             2U
#define AF_USART2           7U

/* --------------------------------------------------------------- EXTI */
#define EXTI_PORT_A         0U
#define EXTI_PORT_B         1U
#define EXTI_PER_REG        4U
#define EXTI_FIELD_BITS     4U
#define EXTI_FIELD_MASK     15U

/* ------------------------------------------------------------ buttons */
#define BTN_SPEED_UP        0U
#define BTN_SPEED_DOWN      1U
#define BTN_RESET           2U
#define BUTTON_COUNT        3U

/* ------------------------------------------------------------- timers */
#define HZ_PER_MHZ          1000000U
#define TIMER_1MHZ_PSC      ((ROAD_CLOCK_HZ / HZ_PER_MHZ) - 1U) /* 1 us tick */
#define US_PER_MS           1000U
#define TICK_1MS_ARR        (US_PER_MS - 1U)
#define INPUT_FILTER_N8     3U   /* fCK_INT, N = 8 */
#define OC_MODE_PWM1        6U

/* ---------------------------------------------------------- ADC / DMA */
#define ADC_CHANNEL_LDR     1U
#define ADC_DATA_MASK       4095U
#define DMA_CHANNEL_USART2  4U
#define UART_BRR_ROUND      (ROAD_UART_BAUD / 2U) /* round BRR to nearest */

/* ------------------------------------------------- US-100 conversion */
/* distance_mm = width_us * 343 m/s / 2 / 1000, rounded to nearest. */
#define SOUND_SPEED_M_S     343U
#define ECHO_MM_DIVISOR     2000U
#define ECHO_MM_ROUND       (ECHO_MM_DIVISOR / 2U)

/* ------------------------------------------------------- 7-segment */
#define SEG_BCD_BIT0        1U
#define SEG_BCD_BIT1        2U
#define SEG_BCD_BIT2        4U
#define SEG_BCD_BIT3        8U
#define SEG_DIGIT_MAX       9U
#define SEG_METRES_PER_STEP 10.0f
#define SEG_BLINK_PERIOD_MS 1000U
#define SEG_BLINK_ON_MS     500U
#define SEG_NO_DATA_A       0U
#define SEG_NO_DATA_B       9U

/* ---------------------------------------------------- NVIC priority */
#define PRIO_ECHO_CAPTURE   0U   /* TIM3: highest, echo timestamps */
#define PRIO_INPUTS         1U   /* ADC and button EXTI */
#define PRIO_LED_UART       2U   /* TIM2 LED PWM and DMA complete */
#define PRIO_APP_TICK       3U   /* TIM4: application, lowest */

#define PIN_BIT(pin)        (1U << (pin))
#define PIN_CLEAR_BIT(pin)  (1U << ((pin) + GPIO_BSRR_RESET_OFS))

volatile RoadInputs g_road_inputs;
volatile RoadOutput g_road_output;
volatile uint32_t g_road_ms;
volatile uint32_t g_uart_dropped_frames;
volatile uint32_t g_uart_dma_errors;
static RoadApp app;
static char tx_buffer[ROAD_FRAME_CAPACITY];
static volatile bool tx_busy = false;
static volatile bool adc_busy = false;
static uint32_t packet_sequence = 0U;
static bool waiting_echo = false;
static bool saw_rise = false;
static uint32_t echo_rise = 0U;
static volatile uint32_t red_duty_us = 0U;
typedef struct {
    bool pending;
    uint32_t edge_ms;
} ButtonEdge;
static volatile ButtonEdge buttons[BUTTON_COUNT];

/* Handlers have fixed CMSIS vector names, as in lecture 0500. */
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void ADC_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM4_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);

static void gpio_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode);
static void gpio_pull(GPIO_TypeDef *port, uint32_t pin, uint32_t pull);
static void gpio_push_pull(GPIO_TypeDef *port, uint32_t pin);
static void gpio_alternate(GPIO_TypeDef *port, uint32_t pin, uint32_t af);
static void gpio_write(GPIO_TypeDef *port, uint32_t pin, bool on);
static void exti_route(uint32_t pin, uint32_t port_code);
static bool pin_pressed(GPIO_TypeDef *port, uint32_t pin);
static void edge(uint32_t index);
static void debounce(void);
static void range_complete(uint8_t status, uint32_t mm);
static void update_red_led(void);
static void transmit(const RoadInputs *snapshot);
static void display_digit(uint32_t digit);
static void update_seven_segment(const RoadInputs *input);
static void init_seven_segment(void);
static void init_buttons(void);
static void init_leds(void);
static void init_adc(void);
static void init_uart_dma(void);
static void init_ultrasonic(void);
static void init_tick_and_nvic(void);

/* ======================================================= GPIO helpers */
static void gpio_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{
    uint32_t shift = pin * GPIO_FIELD2_BITS;
    port->MODER = (port->MODER & ~(GPIO_FIELD2_MASK << shift)) | (mode << shift);
}

static void gpio_pull(GPIO_TypeDef *port, uint32_t pin, uint32_t pull)
{
    uint32_t shift = pin * GPIO_FIELD2_BITS;
    port->PUPDR = (port->PUPDR & ~(GPIO_FIELD2_MASK << shift)) | (pull << shift);
}

static void gpio_push_pull(GPIO_TypeDef *port, uint32_t pin)
{
    port->OTYPER &= ~PIN_BIT(pin);
}

static void gpio_alternate(GPIO_TypeDef *port, uint32_t pin, uint32_t af)
{
    uint32_t index = pin / GPIO_AF_PER_REG;
    uint32_t shift = (pin % GPIO_AF_PER_REG) * GPIO_AF_BITS;
    port->AFR[index] = (port->AFR[index] & ~(GPIO_AF_MASK << shift)) | (af << shift);
}

/* Atomic set/reset through BSRR. */
static void gpio_write(GPIO_TypeDef *port, uint32_t pin, bool on)
{
    if (on) {
        port->BSRR = PIN_BIT(pin);
    } else {
        port->BSRR = PIN_CLEAR_BIT(pin);
    }
}

/* Select which GPIO port drives EXTI line <pin>. */
static void exti_route(uint32_t pin, uint32_t port_code)
{
    uint32_t index = pin / EXTI_PER_REG;
    uint32_t shift = (pin % EXTI_PER_REG) * EXTI_FIELD_BITS;
    SYSCFG->EXTICR[index] = (SYSCFG->EXTICR[index] & ~(EXTI_FIELD_MASK << shift))
                          | (port_code << shift);
}

/* ================================================== buttons (EXTI) */
static bool pin_pressed(GPIO_TypeDef *port, uint32_t pin)
{
    bool high = ((port->IDR & PIN_BIT(pin)) != 0U);
#if ROAD_BUTTON_ACTIVE_LOW
    return !high;
#else
    return high;
#endif
}

static void edge(uint32_t index)
{
    buttons[index].edge_ms = g_road_ms;
    buttons[index].pending = true; /* Restart debounce on EVERY edge. */
}

void EXTI15_10_IRQHandler(void)
{
    if ((EXTI->PR & PIN_BIT(PIN_SPEED_UP)) != 0U) {
        EXTI->PR = PIN_BIT(PIN_SPEED_UP); /* write 1 to clear */
        edge(BTN_SPEED_UP);
    } else {
        /* other lines 10..15 are not enabled */
    }
}

void EXTI3_IRQHandler(void)
{
    if ((EXTI->PR & PIN_BIT(PIN_SPEED_DOWN)) != 0U) {
        EXTI->PR = PIN_BIT(PIN_SPEED_DOWN);
        edge(BTN_SPEED_DOWN);
    } else {
        /* spurious entry */
    }
}

void EXTI4_IRQHandler(void)
{
    if ((EXTI->PR & PIN_BIT(PIN_RESET)) != 0U) {
        EXTI->PR = PIN_BIT(PIN_RESET);
        edge(BTN_RESET);
    } else {
        /* spurious entry */
    }
}

static void debounce(void)
{
    /* Short atomic confirm prevents an EXTI edge racing the deadline test.
     * GPIO is sampled ONLY after an edge/deadline, not continuously polled. */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    for (uint32_t i = 0U; i < BUTTON_COUNT; ++i) {
        bool expired = ((uint32_t)(g_road_ms - buttons[i].edge_ms) >= ROAD_DEBOUNCE_MS);
        if (buttons[i].pending && expired) {
            buttons[i].pending = false;
            switch (i) {
            case BTN_SPEED_UP:
                g_road_inputs.speed_up_pressed = pin_pressed(GPIOA, PIN_SPEED_UP);
                break;
            case BTN_SPEED_DOWN:
                g_road_inputs.speed_down_pressed = pin_pressed(GPIOB, PIN_SPEED_DOWN);
                break;
            case BTN_RESET:
                g_road_inputs.reset_pressed = pin_pressed(GPIOB, PIN_RESET);
                break;
            default:
                /* unreachable index: ignore */
                break;
            }
        } else {
            /* still bouncing or no edge */
        }
    }
    __set_PRIMASK(mask);
}

/* ======================================================= ADC (LDR) */
void ADC_IRQHandler(void)
{
    uint32_t status = ADC1->SR;
    if ((status & ADC_SR_OVR) != 0U) {
        (void)ADC1->DR;
        ADC1->SR = ~(ADC_SR_OVR | ADC_SR_EOC);
        g_road_inputs.ldr_ready = false;
        adc_busy = false;
    } else if ((status & ADC_SR_EOC) != 0U) {
        g_road_inputs.ldr_adc = (uint16_t)(ADC1->DR & ADC_DATA_MASK);
        g_road_inputs.ldr_ms = g_road_ms;
        g_road_inputs.ldr_ready = true;
        adc_busy = false;
    } else {
        /* no ADC event pending */
    }
}

/* ============================================ US-100 (TIM3 capture) */
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
    if ((status & TIM_SR_UIF) != 0U) {
        waiting_echo = true;
        saw_rise = false;
    } else {
        /* not a new trigger period */
    }
    if ((status & (TIM_SR_CC1OF | TIM_SR_CC2OF)) != 0U) {
        (void)TIM3->CCR1;
        (void)TIM3->CCR2;
        range_complete(ROAD_RANGE_FAULT, 0U);
    } else {
        if ((status & TIM_SR_CC1IF) != 0U) {
            uint32_t value = TIM3->CCR1;
            if (waiting_echo) {
                echo_rise = value;
                saw_rise = true;
            } else {
                /* rising edge outside measurement window */
            }
        } else {
            /* no rising edge */
        }
        if ((status & TIM_SR_CC2IF) != 0U) {
            uint32_t fall = TIM3->CCR2;
            if (waiting_echo && saw_rise) {
                uint32_t width = ((fall + ROAD_ECHO_PERIOD_US) - echo_rise) % ROAD_ECHO_PERIOD_US;
                if ((width >= ROAD_ECHO_MIN_US) && (width <= ROAD_ECHO_MAX_US)) {
                    range_complete(ROAD_RANGE_VALID,
                                   ((width * SOUND_SPEED_M_S) + ECHO_MM_ROUND) / ECHO_MM_DIVISOR);
                } else {
                    range_complete(ROAD_RANGE_FAULT, 0U);
                }
            } else {
                /* falling edge without matching rise */
            }
        } else {
            /* no falling edge */
        }
        if (((status & TIM_SR_CC4IF) != 0U) && waiting_echo) {
            /* Missing echo cannot distinguish open space from disconnected
             * sensor. Status=1 exposes this limitation, not "sensor healthy". */
            range_complete(ROAD_RANGE_NO_ECHO, 0U);
        } else {
            /* timeout not reached or echo already received */
        }
    }
}

/* ===================================================== UART TX DMA */
void DMA1_Stream6_IRQHandler(void)
{
    uint32_t status = DMA1->HISR;
    DMA1->HIFCR = DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6
                | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6;
    if ((status & (DMA_HISR_TEIF6 | DMA_HISR_DMEIF6 | DMA_HISR_FEIF6)) != 0U) {
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        g_uart_dma_errors++;
    } else {
        /* no DMA error */
    }
    if ((status & (DMA_HISR_TCIF6 | DMA_HISR_TEIF6 | DMA_HISR_DMEIF6 | DMA_HISR_FEIF6)) != 0U) {
        tx_busy = false;
    } else {
        /* transfer still running */
    }
}

static void transmit(const RoadInputs *snapshot)
{
    /* Never change storage still owned by DMA. Dropping a whole telemetry
     * frame is preferable to blocking the control loop or interleaving bytes. */
    if (tx_busy || ((DMA1_Stream6->CR & DMA_SxCR_EN) != 0U)) {
        g_uart_dropped_frames++;
    } else {
        uint32_t sequence = packet_sequence;
        size_t n = 0U;
        packet_sequence++;
        n = RoadProtocol_Encode(tx_buffer, sizeof tx_buffer, &app.out,
                                snapshot, sequence, g_road_ms);
        if (n == 0U) {
            g_uart_dropped_frames++;
        } else {
            DMA1->HIFCR = DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6
                        | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6;
            DMA1_Stream6->M0AR = (uint32_t)(uintptr_t)tx_buffer;
            DMA1_Stream6->NDTR = (uint32_t)n;
            tx_busy = true;
            __DMB();
            DMA1_Stream6->CR |= DMA_SxCR_EN;
        }
    }
}

/* ===================================================== red LED (TIM2) */
/* PA6 hardware PWM would share TIM3_CH1 with PC6 echo capture. Use a
 * separate timer's update/compare IRQs to drive PA6 as GPIO instead.
 * At most 2000 short IRQs/s; no delay loops and no peripheral polling. */
void TIM2_IRQHandler(void)
{
    uint32_t status = TIM2->SR;
    TIM2->SR = ~status;
    if ((status & TIM_SR_UIF) != 0U) {
        uint32_t duty = red_duty_us;
        TIM2->CCR1 = duty;
        /* Ignore a stale compare from the previous period. If delayed beyond
         * this pulse, leave the LED off instead of stretching the pulse. */
        TIM2->SR = ~TIM_SR_CC1IF;
        if ((duty == ROAD_RED_PWM_PERIOD_US) || (TIM2->CNT < duty)) {
            GPIOA->BSRR = PIN_BIT(PIN_LED_RED);
        } else {
            GPIOA->BSRR = PIN_CLEAR_BIT(PIN_LED_RED);
        }
    } else if ((status & TIM_SR_CC1IF) != 0U) {
        GPIOA->BSRR = PIN_CLEAR_BIT(PIN_LED_RED);
    } else {
        /* no timer event */
    }
}

static void update_red_led(void)
{
    uint32_t target = (app.out.red_target_permille * ROAD_RED_PWM_PERIOD_US) / ROAD_PERMILLE_FULL;
    /* Slew limit avoids visible jumps, including when echo disappears.
     * Only this function writes the duty; TIM2 reads one aligned word. */
    uint32_t duty = red_duty_us;
    uint32_t step = (ROAD_RED_PWM_PERIOD_US * ROAD_TICK_MS) / ROAD_RED_FADE_MS;
    if (target > duty) {
        if ((target - duty) > step) {
            duty += step;
        } else {
            duty = target;
        }
    } else {
        if ((duty - target) > step) {
            duty -= step;
        } else {
            duty = target;
        }
    }
    red_duty_us = duty;
}

/* ========================================================= 7-segment */
/* Decoder mapping from user's source/exam1/files/seven_segment.c. */
static void display_digit(uint32_t digit)
{
    gpio_write(GPIOC, PIN_SEG_BIT0, ((digit & SEG_BCD_BIT0) != 0U));
    gpio_write(GPIOA, PIN_SEG_BIT1, ((digit & SEG_BCD_BIT1) != 0U));
    gpio_write(GPIOA, PIN_SEG_BIT3, ((digit & SEG_BCD_BIT3) != 0U));
    gpio_write(GPIOB, PIN_SEG_BIT2, ((digit & SEG_BCD_BIT2) != 0U));
}

static void update_seven_segment(const RoadInputs *input)
{
    bool valid = input->range_ready && (input->range_status == ROAD_RANGE_VALID)
              && ((uint32_t)(g_road_ms - input->range_ms) <= ROAD_SENSOR_STALE_MS);
    if (valid) {
        uint32_t digit = (uint32_t)(app.out.gap_m / SEG_METRES_PER_STEP);
        if (digit > SEG_DIGIT_MAX) {
            digit = SEG_DIGIT_MAX;
        } else {
            /* digit already 0..9 */
        }
        display_digit(digit);
    } else {
        /* Alternate valid digits for no data; decoder blank-code is unverified. */
        if ((g_road_ms % SEG_BLINK_PERIOD_MS) < SEG_BLINK_ON_MS) {
            display_digit(SEG_NO_DATA_A);
        } else {
            display_digit(SEG_NO_DATA_B);
        }
    }
}

/* ============================================== 1 ms tick (TIM4) */
void TIM4_IRQHandler(void)
{
    if ((TIM4->SR & TIM_SR_UIF) != 0U) {
        TIM4->SR = ~TIM_SR_UIF;
        g_road_ms++;
        debounce();
        if ((g_road_ms % ROAD_TICK_MS) == 0U) {
            uint32_t mask = __get_PRIMASK();
            RoadInputs snapshot;
            __disable_irq();
            snapshot = g_road_inputs;
            __set_PRIMASK(mask);
            RoadApp_Tick(&app, &snapshot, g_road_ms);
            g_road_output = app.out;
            gpio_write(GPIOA, PIN_LED_NIGHT, app.out.night);
            update_red_led();
            update_seven_segment(&snapshot);
            if ((g_road_ms % ROAD_TELEMETRY_MS) == 0U) {
                transmit(&snapshot);
            } else {
                /* not a telemetry slot */
            }
            if (!adc_busy) {
                adc_busy = true;
                ADC1->CR2 |= ADC_CR2_SWSTART;
            } else {
                /* previous conversion still running */
            }
        } else {
            /* not an application tick */
        }
    } else {
        /* spurious entry */
    }
}

/* ============================================================ init */
static void init_seven_segment(void)
{
    /* 7-segment BCD: bit0 PC7, bit1 PA8, bit2 PB10, bit3 PA9. */
    display_digit(0U);
    gpio_mode(GPIOC, PIN_SEG_BIT0, GPIO_MODE_OUTPUT);
    gpio_mode(GPIOA, PIN_SEG_BIT1, GPIO_MODE_OUTPUT);
    gpio_mode(GPIOA, PIN_SEG_BIT3, GPIO_MODE_OUTPUT);
    gpio_mode(GPIOB, PIN_SEG_BIT2, GPIO_MODE_OUTPUT);
    gpio_push_pull(GPIOC, PIN_SEG_BIT0);
    gpio_push_pull(GPIOA, PIN_SEG_BIT1);
    gpio_push_pull(GPIOA, PIN_SEG_BIT3);
    gpio_push_pull(GPIOB, PIN_SEG_BIT2);
    gpio_pull(GPIOC, PIN_SEG_BIT0, GPIO_PULL_NONE);
    gpio_pull(GPIOA, PIN_SEG_BIT1, GPIO_PULL_NONE);
    gpio_pull(GPIOA, PIN_SEG_BIT3, GPIO_PULL_NONE);
    gpio_pull(GPIOB, PIN_SEG_BIT2, GPIO_PULL_NONE);
}

static void init_buttons(void)
{
    /* Shield: PA10 speed+, PB3 speed-, PB4 reset. Keep SWD PA13/14;
     * disable SWO/JTAG use of PB3/PB4 by configuring GPIO input. */
#if ROAD_BUTTON_ACTIVE_LOW
    uint32_t pull = GPIO_PULL_UP;
#else
    uint32_t pull = GPIO_PULL_DOWN;
#endif
    uint32_t lines = PIN_BIT(PIN_SPEED_DOWN) | PIN_BIT(PIN_RESET) | PIN_BIT(PIN_SPEED_UP);
    gpio_mode(GPIOA, PIN_SPEED_UP, GPIO_MODE_INPUT);
    gpio_mode(GPIOB, PIN_SPEED_DOWN, GPIO_MODE_INPUT);
    gpio_mode(GPIOB, PIN_RESET, GPIO_MODE_INPUT);
    gpio_pull(GPIOA, PIN_SPEED_UP, pull);
    gpio_pull(GPIOB, PIN_SPEED_DOWN, pull);
    gpio_pull(GPIOB, PIN_RESET, pull);
    exti_route(PIN_SPEED_DOWN, EXTI_PORT_B);
    exti_route(PIN_RESET, EXTI_PORT_B);
    exti_route(PIN_SPEED_UP, EXTI_PORT_A);
    EXTI->PR = lines;
    EXTI->RTSR |= lines; /* both edges: press and release */
    EXTI->FTSR |= lines;
    EXTI->IMR |= lines;
    for (uint32_t i = 0U; i < BUTTON_COUNT; ++i) {
        edge(i); /* include buttons held at boot */
    }
}

static void init_leds(void)
{
    /* PA5 blue LED: night indicator. */
    gpio_mode(GPIOA, PIN_LED_NIGHT, GPIO_MODE_OUTPUT);
    gpio_push_pull(GPIOA, PIN_LED_NIGHT);
    gpio_write(GPIOA, PIN_LED_NIGHT, false);
    /* Shield red LED PA6/D12, active high. Blue PA5 remains independent. */
    gpio_write(GPIOA, PIN_LED_RED, false);
    gpio_mode(GPIOA, PIN_LED_RED, GPIO_MODE_OUTPUT);
    gpio_push_pull(GPIOA, PIN_LED_RED);
    gpio_pull(GPIOA, PIN_LED_RED, GPIO_PULL_NONE);
    red_duty_us = 0U;
    TIM2->CR1 = 0U;
    TIM2->PSC = TIMER_1MHZ_PSC;
    TIM2->ARR = ROAD_RED_PWM_PERIOD_US - 1U;
    TIM2->CCMR1 = 0U; /* Compare without pin output or CCR preload. */
    TIM2->CCER = 0U;
    TIM2->CCR1 = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
}

static void init_adc(void)
{
    gpio_mode(GPIOA, PIN_LDR, GPIO_MODE_ANALOG);
    gpio_pull(GPIOA, PIN_LDR, GPIO_PULL_NONE);
    ADC->CCR &= ~ADC_CCR_ADCPRE; /* PCLK2/2 = 8 MHz */
    ADC1->CR1 = ADC_CR1_EOCIE | ADC_CR1_OVRIE;
    ADC1->SMPR2 = ADC_SMPR2_SMP1; /* channel 1, 480-cycle sample for LDR */
    ADC1->SQR1 = 0U; /* L encodes N-1: ZERO means ONE conversion */
    ADC1->SQR2 = 0U;
    ADC1->SQR3 = ADC_CHANNEL_LDR;
    ADC1->CR2 = ADC_CR2_ADON; /* first conversion after 10ms startup settling */
}

static void init_uart_dma(void)
{
    /* USART2 TX PA2 AF7 -> ST-LINK virtual COM, 115200 8N1, DMA only.
     * PA3 RX intentionally unused: the PC cannot command the board. */
    gpio_mode(GPIOA, PIN_UART_TX, GPIO_MODE_AF);
    gpio_alternate(GPIOA, PIN_UART_TX, AF_USART2);
    gpio_push_pull(GPIOA, PIN_UART_TX);
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(GPIO_FIELD2_MASK << (PIN_UART_TX * GPIO_FIELD2_BITS)))
                   | (GPIO_SPEED_FAST << (PIN_UART_TX * GPIO_FIELD2_BITS));
    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->BRR = (ROAD_CLOCK_HZ + UART_BRR_ROUND) / ROAD_UART_BAUD;
    USART2->CR3 = USART_CR3_DMAT;
    DMA1_Stream6->CR = (DMA_CHANNEL_USART2 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_DIR_0 | DMA_SxCR_MINC
                     | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    DMA1_Stream6->FCR = 0U;
    DMA1_Stream6->PAR = (uint32_t)(uintptr_t)&USART2->DR;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void init_ultrasonic(void)
{
    /* US-100, jumper REMOVED, 5V supply for the tested module, common ground.
     * Morpho PC6 = ECHO / TIM3_CH1 AF2; PC8 = TRIG / TIM3_CH3 AF2.
     * Avoid PA0 (shield NTC) and PC7 (shield 7-segment decoder).
     * Confirm these Morpho pins are accessible on your shield revision. */
    gpio_mode(GPIOC, PIN_ECHO, GPIO_MODE_AF);
    gpio_mode(GPIOC, PIN_TRIG, GPIO_MODE_AF);
    gpio_alternate(GPIOC, PIN_ECHO, AF_TIM3);
    gpio_alternate(GPIOC, PIN_TRIG, AF_TIM3);
    gpio_push_pull(GPIOC, PIN_TRIG);
    /* PC6 is an FT digital input. Keep its internal pulls disabled when the
     * tested 5V-powered US-100 drives ECHO above the MCU supply voltage. */
    gpio_pull(GPIOC, PIN_ECHO, GPIO_PULL_NONE);
    gpio_pull(GPIOC, PIN_TRIG, GPIO_PULL_NONE);
    TIM3->PSC = TIMER_1MHZ_PSC;
    TIM3->ARR = ROAD_ECHO_PERIOD_US - 1U;
    /* CH1 rising direct TI1, CH2 falling indirect TI1. Hardware timestamps. */
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1
                | (INPUT_FILTER_N8 << TIM_CCMR1_IC1F_Pos)
                | (INPUT_FILTER_N8 << TIM_CCMR1_IC2F_Pos);
    TIM3->CCMR2 = (OC_MODE_PWM1 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM3->CCR3 = ROAD_ECHO_TRIGGER_US;
    TIM3->CCR4 = ROAD_ECHO_TIMEOUT_US;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC3E;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    TIM3->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC4IE;
    waiting_echo = true;
}

static void init_tick_and_nvic(void)
{
    TIM4->PSC = TIMER_1MHZ_PSC;
    TIM4->ARR = TICK_1MS_ARR;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->SR = 0U;
    TIM4->DIER = TIM_DIER_UIE;
    NVIC_SetPriorityGrouping(0U);
    NVIC_SetPriority(TIM3_IRQn, PRIO_ECHO_CAPTURE);
    NVIC_SetPriority(TIM2_IRQn, PRIO_LED_UART);
    NVIC_SetPriority(ADC_IRQn, PRIO_INPUTS);
    NVIC_SetPriority(EXTI3_IRQn, PRIO_INPUTS);
    NVIC_SetPriority(EXTI4_IRQn, PRIO_INPUTS);
    NVIC_SetPriority(EXTI15_10_IRQn, PRIO_INPUTS);
    NVIC_SetPriority(DMA1_Stream6_IRQn, PRIO_LED_UART);
    NVIC_SetPriority(TIM4_IRQn, PRIO_APP_TICK);
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 = TIM_CR1_CEN;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM4->CR1 = TIM_CR1_CEN;
}

void RoadBoard_Init(void)
{
    RoadApp_Init(&app);
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR; /* clock-enable propagation */
    init_seven_segment();
    init_buttons();
    init_leds();
    init_adc();
    init_uart_dma();
    init_ultrasonic();
    init_tick_and_nvic(); /* starts the timers last */
}
