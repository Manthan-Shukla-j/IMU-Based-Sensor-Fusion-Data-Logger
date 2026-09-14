/*
 * kalman.h - 1D Kalman filter for fusing accelerometer angle (noisy but
 * drift-free) with gyroscope rate (smooth but drifts) into a clean angle
 * estimate. One instance is used per axis (roll, pitch).
 *
 * Model: state = [angle, gyro_bias]^T
 */

#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float Q_angle;   /* process noise variance for angle */
    float Q_bias;    /* process noise variance for gyro bias */
    float R_measure; /* measurement noise variance (accel angle) */

    float angle;     /* filtered angle, degrees */
    float bias;       /* estimated gyro bias, deg/s */
    float rate;        /* unbiased rate, deg/s (output, for reference) */

    float P[2][2];    /* error covariance matrix */
} Kalman_t;

void Kalman_Init(Kalman_t *k);

/* newAngle: accel-derived angle (deg), newRate: gyro rate (deg/s), dt: seconds */
float Kalman_Update(Kalman_t *k, float newAngle, float newRate, float dt);

#endif /* KALMAN_H */
