/*
 * uart.c - register-level USART2 driver.
 *
 * PA2 = TX (AF7), PA3 = RX (AF7)
 * BRR is computed for APB1 = 16 MHz (HSI, no PLL).
 *   USARTDIV = Fck / baud
 *   115200 baud @ 16 MHz -> USARTDIV = 138.89
 *     mantissa = 138 (0x8A), fraction = round(0.89*16) = 14 (0xE)
 *     BRR = (138 << 4) | 14 = 0x8AE
 */

#include "uart.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

void USART2_Init(uint32_t baud)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2/PA3 -> Alternate function, AF7 (USART2), push-pull, high speed */
    GPIOA->MODER  &= ~((3U << (2 * PC_TX_PIN)) | (3U << (2 * PC_RX_PIN)));
    GPIOA->MODER  |=  ((2U << (2 * PC_TX_PIN)) | (2U << (2 * PC_RX_PIN)));

    GPIOA->OSPEEDR |= (3U << (2 * PC_TX_PIN)) | (3U << (2 * PC_RX_PIN));

    GPIOA->AFR[0] &= ~((0xFU << (PC_TX_PIN * 4)) | (0xFU << (PC_RX_PIN * 4)));
    GPIOA->AFR[0] |=  ((7U   << (PC_TX_PIN * 4)) | (7U   << (PC_RX_PIN * 4))); /* AF7 */

    USART2->CR1 = 0;
    /* Baud rate: computed for 16 MHz APB1. If you change SystemClock_Config
     * to enable the PLL, recompute BRR = SYSTEM_CLOCK_HZ / baud. */
    if (baud == 115200U) {
        USART2->BRR = 0x8AE;
    } else {
        USART2->BRR = (uint16_t)(SYSTEM_CLOCK_HZ / baud);
    }

    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; /* enable TX + RX */
    USART2->CR1 |= USART_CR1_UE;                /* enable USART */
}

void USART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE)) { }
    USART2->DR = (uint8_t)c;
}

void USART2_SendString(const char *s)
{
    while (*s) {
        USART2_SendChar(*s++);
    }
}

void USART2_SendFloat(float val, uint8_t decimals)
{
    char buf[32];
    /* snprintf keeps this simple; if you want to drop libc float
       formatting entirely for a smaller binary, replace this with a
       manual fixed-point-to-string routine. */
    snprintf(buf, sizeof(buf), "%.*f", decimals, (double)val);
    USART2_SendString(buf);
}
