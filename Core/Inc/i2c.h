/*
 * i2c.h - minimal bare-metal I2C1 driver for STM32F411RE (register-level,
 * no HAL). Standard mode, 100 kHz, polling (no interrupts/DMA) to keep
 * the logic easy to follow and easy to simulate in Proteus.
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "stm32f4xx.h"
#include <stdint.h>

void I2C1_Init(void);
uint8_t I2C1_WriteByte(uint8_t devAddr7, uint8_t regAddr, uint8_t data);
uint8_t I2C1_ReadByte(uint8_t devAddr7, uint8_t regAddr, uint8_t *out);
uint8_t I2C1_ReadBytes(uint8_t devAddr7, uint8_t regAddr, uint8_t *buf, uint16_t len);

#endif /* I2C_DRIVER_H */
