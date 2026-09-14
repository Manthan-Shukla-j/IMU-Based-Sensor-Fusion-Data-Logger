/*
 * mpu6050.c - MPU6050 driver built on top of the bare-metal I2C1 driver.
 *
 * Default full-scale ranges used here: accel +-2g, gyro +-250 deg/s
 *   accel LSB sensitivity = 16384 LSB/g
 *   gyro  LSB sensitivity = 131   LSB/(deg/s)
 */

#include "mpu6050.h"
#include "i2c.h"

#define ACCEL_SENS  16384.0f
#define GYRO_SENS   131.0f

uint8_t MPU6050_Init(void)
{
    uint8_t who = 0;

    if (!I2C1_ReadByte(MPU6050_ADDR, MPU6050_REG_WHO_AM_I, &who)) return 0;
    if (who != 0x68) return 0; /* wrong/absent device */

    /* Wake up device: clear SLEEP bit, use internal 8MHz osc as clock src */
    if (!I2C1_WriteByte(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00)) return 0;

    /* Sample rate divider: Gyro output rate (8kHz, DLPF off) / (1+7) = 1kHz */
    if (!I2C1_WriteByte(MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV, 0x07)) return 0;

    /* DLPF: ~44Hz bandwidth, reduces vibration noise before it reaches
       the Kalman filter */
    if (!I2C1_WriteByte(MPU6050_ADDR, MPU6050_REG_CONFIG, 0x03)) return 0;

    /* Gyro full scale +-250 deg/s */
    if (!I2C1_WriteByte(MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, 0x00)) return 0;

    /* Accel full scale +-2g */
    if (!I2C1_WriteByte(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 0x00)) return 0;

    return 1;
}

uint8_t MPU6050_ReadAll(MPU6050_Data_t *data)
{
    uint8_t raw[14];
    if (!I2C1_ReadBytes(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, raw, 14)) return 0;

    int16_t ax_raw = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4]  << 8) | raw[5]);
    /* raw[6],raw[7] = temperature, skipped */
    int16_t gx_raw = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

    data->ax = ax_raw / ACCEL_SENS;
    data->ay = ay_raw / ACCEL_SENS;
    data->az = az_raw / ACCEL_SENS;

    data->gx = gx_raw / GYRO_SENS;
    data->gy = gy_raw / GYRO_SENS;
    data->gz = gz_raw / GYRO_SENS;

    return 1;
}
