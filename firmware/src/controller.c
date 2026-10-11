#include "controller.h"

static float clampf(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
}

void pid_reset(pid_ctrl_t *pid)
{
    pid->integral = 0.0f;
}

float pid_update(pid_ctrl_t *pid, float error, float error_rate, float dt)
{
    pid->integral += error * dt;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * error_rate;
}

float balance_pid_accel(pid_ctrl_t *pid, float tilt, float tilt_rate, float dt, float max_accel)
{
    /* Setpoint is upright (0), so error = -tilt. Leaning forward -> drive forward. */
    float accel = -pid_update(pid, -tilt, -tilt_rate, dt);
    return clampf(accel, -max_accel, max_accel);
}

float balance_lqr_accel(const float *k, float position, float speed, float tilt, float tilt_rate,
                        float max_accel)
{
    float accel = -(k[0] * position + k[1] * speed + k[2] * tilt + k[3] * tilt_rate);
    return clampf(accel, -max_accel, max_accel);
}

float accel_to_command(float accel, float max_accel)
{
    if (max_accel <= 0.0f)
    {
        return 0.0f;
    }
    return clampf(accel / max_accel, -1.0f, 1.0f);
}

int balance_should_cut(float tilt, float limit)
{
    float magnitude = (tilt < 0.0f) ? -tilt : tilt;
    return magnitude > limit;
}