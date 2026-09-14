/*
 * main.h
 * Bare-metal STM32F411RE (Nucleo-F411RE) project header.
 * IMU (MPU6050) sensor fusion with Kalman filter, UART streaming to PC.
 *
 * No HAL is used. All peripheral access is via CMSIS register
 * definitions (stm32f4xx.h), which STM32CubeIDE provides automatically
 * once the target device / Nucleo-F411RE board is selected while
 * creating the project. You do NOT need to add these yourself.
 */

#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx.h"
#include <stdint.h>

/* ---------------- Clock ----------------
 * We stay on the default HSI 16 MHz clock (no PLL) to keep things
 * simple and deterministic in Proteus simulation. SystemCoreClock
 * is therefore fixed at 16,000,000 Hz.
 */
#define SYSTEM_CLOCK_HZ   16000000UL

/* ---------------- I2C1 (MPU6050) ----------------
 * PB8 -> I2C1_SCL (AF4)
 * PB9 -> I2C1_SDA (AF4)
 * External 4.7k pull-ups to 3.3V required on both lines (add in Proteus).
 */
#define MPU6050_I2C            I2C1
#define MPU6050_I2C_GPIO_PORT  GPIOB
#define MPU6050_SCL_PIN        8
#define MPU6050_SDA_PIN        9

/* ---------------- USART2 (PC link) ----------------
 * PA2 -> USART2_TX (AF7)  -> connect to COMPIM / USB-UART RX in Proteus
 * PA3 -> USART2_RX (AF7)  -> connect to COMPIM / USB-UART TX in Proteus
 * This is the same USART2 that is wired to the ST-LINK Virtual COM
 * Port on a real Nucleo-F411RE board.
 */
#define PC_UART            USART2
#define PC_UART_GPIO_PORT  GPIOA
#define PC_TX_PIN          2
#define PC_RX_PIN          3
#define PC_UART_BAUD       115200U

/* Sampling period for the main control/fusion loop, in milliseconds */
#define SAMPLE_PERIOD_MS   20U   /* 50 Hz */

/* ---------------- Function prototypes ---------------- */
void SystemClock_Config(void);
void GPIO_Init(void);
void Delay_Init(void);
void Delay_ms(uint32_t ms);
void ErrorHandler(void);

#endif /* MAIN_H */
