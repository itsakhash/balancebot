#ifndef TILT_H
#define TILT_H

#include <stdint.h>

/* Gyro scale for GYRO_CONFIG = 0x08 (+/-500 deg/s). */
#define TILT_GYRO_LSB_PER_DPS 65.5f

typedef struct
{
    float alpha;         /* complementary filter weight on the gyro, e.g. 0.98 */
    float dt;            /* seconds between updates, e.g. 0.005 for 200 Hz     */
    float angle_deg;     /* current tilt estimate                              */
    float gyro_bias_dps; /* resting gyro offset found by calibration           */
    int32_t cal_sum;     /* running sum of raw gyro readings while calibrating */
    uint32_t cal_count;  /* how many readings were summed                      */
    uint8_t started;     /* 0 until the first update seeds the angle           */
} tilt_t;

void tilt_init(tilt_t *t, float alpha, float dt);

/* Call with the raw gyro-Y reading while the board is held still. */
void tilt_calibrate_add(tilt_t *t, int16_t gy_raw);

/* Averages the readings added so far and stores the result as the bias. */
void tilt_calibrate_finish(tilt_t *t);

/* Accelerometer-only tilt in degrees: atan2(-ax, az). */
float tilt_accel_angle_deg(int16_t ax, int16_t az);

/* One filter step. Returns the new tilt estimate in degrees.
 * The first call seeds the estimate from the accelerometer. */
float tilt_update(tilt_t *t, int16_t ax, int16_t az, int16_t gy_raw);

#endif