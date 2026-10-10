#include "tilt.h"

#include <math.h>

#define RAD_TO_DEG 57.29577951f

void tilt_init(tilt_t *t, float alpha, float dt)
{
    t->alpha = alpha;
    t->dt = dt;
    t->angle_deg = 0.0f;
    t->gyro_bias_dps = 0.0f;
    t->cal_sum = 0;
    t->cal_count = 0;
    t->started = 0;
}

void tilt_calibrate_add(tilt_t *t, int16_t gy_raw)
{
    t->cal_sum += gy_raw;
    t->cal_count++;
}

void tilt_calibrate_finish(tilt_t *t)
{
    if (t->cal_count == 0)
    {
        return;
    }
    t->gyro_bias_dps = ((float)t->cal_sum / (float)t->cal_count) / TILT_GYRO_LSB_PER_DPS;
}

float tilt_accel_angle_deg(int16_t ax, int16_t az)
{
    return atan2f(-(float)ax, (float)az) * RAD_TO_DEG;
}

float tilt_update(tilt_t *t, int16_t ax, int16_t az, int16_t gy_raw)
{
    float accel_angle = tilt_accel_angle_deg(ax, az);
    float rate_dps = (float)gy_raw / TILT_GYRO_LSB_PER_DPS - t->gyro_bias_dps;

    if (!t->started)
    {
        t->angle_deg = accel_angle;
        t->started = 1;
        return t->angle_deg;
    }

    t->angle_deg = t->alpha * (t->angle_deg + rate_dps * t->dt) + (1.0f - t->alpha) * accel_angle;
    return t->angle_deg;
}