#include <stdint.h>

#ifndef STM32F411xE
#define STM32F411xE
#endif

#include "stm32f4xx.h"

/*
 * Standalone US-100 UART diagnostic for NUCLEO-F411RE.
 *
 * Console: USART2 PA2 -> ST-LINK VCP, 115200 8N1.
 * Sensor:  USART6 PC6 TX -> US-100 Trig/TX, 9600 8N1.
 *          USART6 PC7 RX <- US-100 Echo/RX.
 *
 * The UART SELECT jumper on the sensor must be installed before power-up.
 * This is polling diagnostic code, not final project firmware.
 * It assumes reset HSI 16 MHz and APB1/APB2 prescalers of 1.
 */

#define CLOCK_HZ          16000000U
#define SENSOR_BAUD           9600U
#define RESPONSE_WINDOW_MS     500U
#define REQUEST_INTERVAL_MS   1000U
#define RAW_CAPACITY             8U

static uint32_t elapsed_ms(uint32_t start)
{
    return (uint32_t)(DWT->CYCCNT - start) / (CLOCK_HZ / 1000U);
}

static void delay_ms(uint32_t duration)
{
    uint32_t start = DWT->CYCCNT;
    while (elapsed_ms(start) < duration) {
        /* Standalone bench test only. */
    }
}

static void console_char(char value)
{
    while ((USART2->SR & USART_SR_TXE) == 0U) {
    }
    USART2->DR = (uint32_t)(uint8_t)value;
}

static void console_text(const char *text)
{
    while (*text != '\0') {
        console_char(*text);
        text++;
    }
}

static void console_u32(uint32_t value)
{
    char digits[10];
    uint32_t count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (count != 0U) {
        console_char(digits[--count]);
    }
}

static void console_hex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    console_char(hex[(value >> 4U) & 0x0FU]);
    console_char(hex[value & 0x0FU]);
}

static void console_hex32(uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    for (uint32_t shift = 28U;; shift -= 4U) {
        console_char(hex[(value >> shift) & 0x0FU]);
        if (shift == 0U) {
            break;
        }
    }
}

static void console_init(void)
{
    GPIOA->MODER = (GPIOA->MODER & ~(3U << 4U)) | (2U << 4U);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(15U << 8U)) | (7U << 8U);
    GPIOA->OTYPER &= ~(1U << 2U);
    GPIOA->PUPDR &= ~(3U << 4U);

    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;
    USART2->BRR = (CLOCK_HZ + 57600U) / 115200U;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void sensor_uart_init(void)
{
    /* PC6 AF8 = USART6_TX; PC7 AF8 = USART6_RX. */
    GPIOC->MODER =
        (GPIOC->MODER & ~((3U << 12U) | (3U << 14U))) |
        (2U << 12U) | (2U << 14U);
    GPIOC->AFR[0] =
        (GPIOC->AFR[0] & ~((15U << 24U) | (15U << 28U))) |
        (8U << 24U) | (8U << 28U);
    GPIOC->OTYPER &= ~(1U << 6U);
    GPIOC->PUPDR &= ~((3U << 12U) | (3U << 14U));
    GPIOC->OSPEEDR =
        (GPIOC->OSPEEDR & ~(3U << 12U)) | (2U << 12U);

    USART6->CR1 = 0U;
    USART6->CR2 = 0U;
    USART6->CR3 = 0U;
    USART6->BRR = (CLOCK_HZ + (SENSOR_BAUD / 2U)) / SENSOR_BAUD;
    USART6->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    /* Clear stale receive/error state using the required SR then DR sequence. */
    (void)USART6->SR;
    (void)USART6->DR;
}

static void print_configuration(void)
{
    console_text("CFGR=0x");
    console_hex32(RCC->CFGR);
    console_text(" BRR6=");
    console_u32(USART6->BRR);
    console_text(" MODER_C=0x");
    console_hex32(GPIOC->MODER);
    console_text(" AFRL_C=0x");
    console_hex32(GPIOC->AFR[0]);
    console_text("\r\n");
}

static void drain_sensor_rx(void)
{
    for (uint32_t count = 0U; count < 16U; count++) {
        uint32_t status = USART6->SR;
        if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE |
                       USART_SR_FE | USART_SR_PE)) == 0U) {
            break;
        }
        (void)USART6->DR;
    }
}

static void run_request(uint32_t request_number)
{
    uint8_t raw[RAW_CAPACITY];
    uint32_t raw_count = 0U;
    uint32_t error_flags = 0U;
    uint32_t start;
    uint32_t pc6_before = (GPIOC->IDR >> 6U) & 1U;
    uint32_t pc7_before = (GPIOC->IDR >> 7U) & 1U;

    drain_sensor_rx();

    console_text("REQ ");
    console_u32(request_number);
    console_text(" idle PC6_TX=");
    console_u32(pc6_before);
    console_text(" PC7_RX=");
    console_u32(pc7_before);
    console_text(" SR6=0x");
    console_hex32(USART6->SR);
    console_text("\r\n");

    while ((USART6->SR & USART_SR_TXE) == 0U) {
    }
    USART6->DR = 0x55U;

    /* TC proves USART6 finished shifting the command onto PC6. */
    start = DWT->CYCCNT;
    while ((USART6->SR & USART_SR_TC) == 0U) {
        if (elapsed_ms(start) >= 20U) {
            console_text("ERR USART6 TX DID NOT COMPLETE\r\n");
            return;
        }
    }

    start = DWT->CYCCNT;
    while (elapsed_ms(start) < RESPONSE_WINDOW_MS) {
        uint32_t status = USART6->SR;
        error_flags |= status &
            (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE);

        if ((status & USART_SR_RXNE) != 0U) {
            uint8_t value = (uint8_t)USART6->DR;
            if (raw_count < RAW_CAPACITY) {
                raw[raw_count++] = value;
            }
        } else if ((status & (USART_SR_ORE | USART_SR_NE |
                              USART_SR_FE | USART_SR_PE)) != 0U) {
            (void)USART6->DR;
        }

        if (raw_count >= 2U) {
            break;
        }
    }

    console_text("RESULT bytes=");
    console_u32(raw_count);
    console_text(" raw=");
    if (raw_count == 0U) {
        console_text("--");
    } else {
        for (uint32_t index = 0U; index < raw_count; index++) {
            if (index != 0U) {
                console_char(' ');
            }
            console_hex8(raw[index]);
        }
    }
    console_text(" errors=0x");
    console_hex32(error_flags);
    console_text(" final_PC7=");
    console_u32((GPIOC->IDR >> 7U) & 1U);

    if (raw_count >= 2U) {
        uint32_t mm = ((uint32_t)raw[0] << 8U) | (uint32_t)raw[1];
        console_text(" distance_mm=");
        console_u32(mm);
    }
    console_text("\r\n");
}

int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;
    (void)RCC->APB2ENR;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    console_init();

    console_text("\r\nUS-100 UART DIAGNOSTIC V2\r\n");
    console_text("PC6 -> Trig/TX, PC7 <- Echo/RX, sensor UART jumper ON\r\n");

    if ((RCC->CFGR & (RCC_CFGR_SWS | RCC_CFGR_HPRE |
                      RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) != 0U) {
        console_text("FATAL CLOCK IS NOT RESET HSI 16MHz /1\r\n");
        console_text("RCC_CFGR=0x");
        console_hex32(RCC->CFGR);
        console_text("\r\n");
        for (;;) {
        }
    }

    sensor_uart_init();
    print_configuration();
    delay_ms(1000U);

    for (uint32_t request = 1U;; request++) {
        run_request(request);
        delay_ms(REQUEST_INTERVAL_MS);
    }
}
