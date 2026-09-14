/*
 * uart.h - minimal bare-metal USART2 driver for STM32F411RE (register
 * level, no HAL). Polling TX, used to stream data to the Python
 * dashboard over the ST-LINK Virtual COM Port / a Proteus COMPIM.
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "stm32f4xx.h"
#include <stdint.h>

void USART2_Init(uint32_t baud);
void USART2_SendChar(char c);
void USART2_SendString(const char *s);
void USART2_SendFloat(float val, uint8_t decimals);

#endif /* UART_DRIVER_H */
