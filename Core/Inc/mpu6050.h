/*
 * mpu6050.h - MPU6050 IMU driver over I2C (accelerometer + gyroscope).
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/* 7-bit I2C address: AD0 pin tied to GND -> 0x68 (tie to 3.3V for 0x69) */
#define MPU6050_ADDR        0x68U

/* Register map (subset used here) */
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_WHO_AM_I     0x75

typedef struct {
    float ax, ay, az;   /* accel, g */
    float gx, gy, gz;   /* gyro, deg/s */
} MPU6050_Data_t;

/* Returns 1 on success (WHO_AM_I matches, device configured), 0 on failure */
uint8_t MPU6050_Init(void);

/* Reads and converts one full accel+gyro sample. Returns 1 on success. */
uint8_t MPU6050_ReadAll(MPU6050_Data_t *data);

#endif /* MPU6050_H */
