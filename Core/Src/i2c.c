/*
 * i2c.c - register-level I2C1 driver.
 *
 * Pins: PB8 = SCL (AF4, open-drain), PB9 = SDA (AF4, open-drain)
 * Speed: 100 kHz standard mode, APB1 = 16 MHz (HSI, no PLL)
 *
 *   CCR   = Fpclk1 / (2 * Fscl) = 16,000,000 / (2 * 100,000) = 80
 *   TRISE = (Fpclk1_MHz) + 1                                = 17
 */

#include "i2c.h"
#include "main.h"

#define I2C_TIMEOUT   100000UL

static uint8_t I2C1_WaitFlag(volatile uint32_t *reg, uint32_t flag)
{
    uint32_t t = I2C_TIMEOUT;
    while (!((*reg) & flag)) {
        if (--t == 0) return 0; /* timeout -> fail */
    }
    return 1;
}

void I2C1_Init(void)
{
    /* 1. Clock for GPIOB and I2C1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* 2. PB8/PB9 -> Alternate Function, open-drain, pull-up, high speed, AF4 */
    GPIOB->MODER   &= ~((3U << (2 * MPU6050_SCL_PIN)) | (3U << (2 * MPU6050_SDA_PIN)));
    GPIOB->MODER   |=  ((2U << (2 * MPU6050_SCL_PIN)) | (2U << (2 * MPU6050_SDA_PIN))); /* AF mode */

    GPIOB->OTYPER  |= (1U << MPU6050_SCL_PIN) | (1U << MPU6050_SDA_PIN); /* open-drain */

    GPIOB->OSPEEDR |= (3U << (2 * MPU6050_SCL_PIN)) | (3U << (2 * MPU6050_SDA_PIN)); /* very high speed */

    GPIOB->PUPDR   &= ~((3U << (2 * MPU6050_SCL_PIN)) | (3U << (2 * MPU6050_SDA_PIN)));
    GPIOB->PUPDR   |=  ((1U << (2 * MPU6050_SCL_PIN)) | (1U << (2 * MPU6050_SDA_PIN))); /* internal pull-up
                                                                                             (still add external
                                                                                             4.7k in Proteus) */

    GPIOB->AFR[1]  &= ~((0xFU << ((MPU6050_SCL_PIN - 8) * 4)) | (0xFU << ((MPU6050_SDA_PIN - 8) * 4)));
    GPIOB->AFR[1]  |=  ((4U   << ((MPU6050_SCL_PIN - 8) * 4)) | (4U   << ((MPU6050_SDA_PIN - 8) * 4))); /* AF4 */

    /* 3. Reset then configure I2C1 peripheral */
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR2   = 16;              /* APB1 clock in MHz (16 MHz) */
    I2C1->CCR   = 80;              /* 100 kHz standard mode */
    I2C1->TRISE = 17;              /* max rise time + 1 */
    I2C1->CR1  |= I2C_CR1_PE;      /* enable peripheral */
}

static uint8_t I2C1_Start(uint8_t devAddr7, uint8_t direction)
{
    I2C1->CR1 |= I2C_CR1_START;
    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_SB)) return 0;

    (void)I2C1->SR1; /* clear SB by reading SR1 then writing DR */
    I2C1->DR = (uint8_t)((devAddr7 << 1) | direction);

    uint32_t t = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) { I2C1->SR1 &= ~I2C_SR1_AF; return 0; } /* NACK: device not present */
        if (--t == 0) return 0;
    }
    (void)I2C1->SR1;
    (void)I2C1->SR2; /* clear ADDR by reading SR1 then SR2 */
    return 1;
}

static void I2C1_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

uint8_t I2C1_WriteByte(uint8_t devAddr7, uint8_t regAddr, uint8_t data)
{
    if (!I2C1_Start(devAddr7, 0)) return 0;

    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_TXE)) { I2C1_Stop(); return 0; }
    I2C1->DR = regAddr;

    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_TXE)) { I2C1_Stop(); return 0; }
    I2C1->DR = data;

    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_BTF)) { I2C1_Stop(); return 0; }
    I2C1_Stop();
    return 1;
}

uint8_t I2C1_ReadByte(uint8_t devAddr7, uint8_t regAddr, uint8_t *out)
{
    return I2C1_ReadBytes(devAddr7, regAddr, out, 1);
}

uint8_t I2C1_ReadBytes(uint8_t devAddr7, uint8_t regAddr, uint8_t *buf, uint16_t len)
{
    if (len == 0 || buf == NULL) return 0;

    /* Phase 1: write register pointer */
    if (!I2C1_Start(devAddr7, 0)) return 0;
    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_TXE)) { I2C1_Stop(); return 0; }
    I2C1->DR = regAddr;
    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_TXE)) { I2C1_Stop(); return 0; }

    /* Phase 2: repeated START, read `len` bytes */
    I2C1->CR1 |= I2C_CR1_ACK;
    if (!I2C1_Start(devAddr7, 1)) return 0;

    for (uint16_t i = 0; i < len; i++) {
        if (i == (len - 1)) {
            I2C1->CR1 &= ~I2C_CR1_ACK; /* NACK on last byte */
            I2C1_Stop();
        }
        if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_RXNE)) return 0;
        buf[i] = (uint8_t)I2C1->DR;
    }
    return 1;
}
