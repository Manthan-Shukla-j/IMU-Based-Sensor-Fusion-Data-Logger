/*
 * main.c - MPU6050 IMU -> Kalman filter -> UART CSV stream, bare-metal
 * on STM32F411RE (Nucleo-F411RE), for use in Proteus simulation
 * (no physical board needed).
 *
 * Output over USART2 @ 115200 baud, once every SAMPLE_PERIOD_MS:
 *   roll,pitch,ax,ay,az,gx,gy,gz\r\n
 * This exact CSV format is what dashboard/dashboard.py expects.
 *
 * Build target: STM32CubeIDE, "Empty" project (no HAL/CubeMX code
 * generation needed), targeting NUCLEO-F411RE. See README.md.
 */

#include "main.h"
#include "i2c.h"
#include "uart.h"
#include "mpu6050.h"
#include "kalman.h"
#include <math.h>
#include <stdio.h>

static volatile uint32_t g_msTicks = 0;
static Kalman_t kalRoll, kalPitch;

void SysTick_Handler(void)
{
    g_msTicks++;
}

void SystemClock_Config(void)
{
    /* Using default HSI 16 MHz, no PLL — nothing to configure. Kept as
       a named function so it's obvious where to add PLL setup later if
       you want a faster core clock on real hardware. */
}

void Delay_Init(void)
{
    /* 1ms SysTick tick, assuming 16 MHz core clock */
    SysTick_Config(SYSTEM_CLOCK_HZ / 1000U);
}

void Delay_ms(uint32_t ms)
{
    uint32_t start = g_msTicks;
    while ((g_msTicks - start) < ms) { }
}

void GPIO_Init(void)
{
    /* Onboard LED (PA5 on Nucleo-F411RE) as a simple heartbeat indicator */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3U << (2 * 5));
    GPIOA->MODER |=  (1U << (2 * 5)); /* output */
}

void ErrorHandler(void)
{
    /* Fast LED blink forever = init failure (e.g. MPU6050 not found) */
    while (1) {
        GPIOA->ODR ^= (1U << 5);
        for (volatile uint32_t i = 0; i < 200000; i++) { }
    }
}

int main(void)
{
    SystemClock_Config();
    Delay_Init();
    GPIO_Init();
    USART2_Init(PC_UART_BAUD);
    I2C1_Init();

    USART2_SendString("BOOT: initializing MPU6050...\r\n");

    if (!MPU6050_Init()) {
        USART2_SendString("ERROR: MPU6050 not detected. Check I2C wiring/address.\r\n");
        ErrorHandler();
    }

    USART2_SendString("BOOT: MPU6050 OK. Streaming roll,pitch,ax,ay,az,gx,gy,gz\r\n");

    Kalman_Init(&kalRoll);
    Kalman_Init(&kalPitch);

    /* Seed the filter with the first accel reading so it doesn't start at 0
       and slew for a while if the board isn't level at boot. */
    MPU6050_Data_t sample;
    if (MPU6050_ReadAll(&sample)) {
        kalRoll.angle  = atan2f(sample.ay, sample.az) * 57.2957795f;
        kalPitch.angle = atan2f(-sample.ax, sqrtf(sample.ay * sample.ay + sample.az * sample.az)) * 57.2957795f;
    }

    const float dt = SAMPLE_PERIOD_MS / 1000.0f;
    char line[128];

    while (1) {
        if (MPU6050_ReadAll(&sample)) {
            /* Accelerometer-derived angles (degrees), valid at rest / low accel */
            float accelRoll  = atan2f(sample.ay, sample.az) * 57.2957795f;
            float accelPitch = atan2f(-sample.ax, sqrtf(sample.ay * sample.ay + sample.az * sample.az)) * 57.2957795f;

            float roll  = Kalman_Update(&kalRoll,  accelRoll,  sample.gx, dt);
            float pitch = Kalman_Update(&kalPitch, accelPitch, sample.gy, dt);

            snprintf(line, sizeof(line), "%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\r\n",
                     roll, pitch,
                     sample.ax, sample.ay, sample.az,
                     sample.gx, sample.gy, sample.gz);
            USART2_SendString(line);

            GPIOA->ODR ^= (1U << 5); /* heartbeat toggle each sample */
        } else {
            USART2_SendString("WARN: IMU read failed\r\n");
        }

        Delay_ms(SAMPLE_PERIOD_MS);
    }
}
