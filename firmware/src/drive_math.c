#include "drive_math.h"

void motor_drive_from_command(float cmd, int invert, uint16_t period, float deadband,
                              motor_drive_t *out)
{
    if (cmd > 1.0f)
    {
        cmd = 1.0f;
    }
    if (cmd < -1.0f)
    {
        cmd = -1.0f;
    }
    if (invert)
    {
        cmd = -cmd;
    }

    float magnitude = (cmd < 0.0f) ? -cmd : cmd;
    if (magnitude < 0.001f)
    {
        out->in1 = 0;
        out->in2 = 0;
        out->duty = 0;
        return;
    }

    if (deadband < 0.0f)
    {
        deadband = 0.0f;
    }
    if (deadband > 0.9f)
    {
        deadband = 0.9f;
    }

    float fraction = deadband + (1.0f - deadband) * magnitude;
    float ticks = fraction * (float)period + 0.5f;
    if (ticks > (float)period)
    {
        ticks = (float)period;
    }

    out->duty = (uint16_t)ticks;
    out->in1 = (cmd > 0.0f) ? 1 : 0;
    out->in2 = (cmd < 0.0f) ? 1 : 0;
}

int32_t encoder_delta16(uint16_t previous, uint16_t current)
{
    return (int16_t)(uint16_t)(current - previous);
}

int32_t encoder_delta32(uint32_t previous, uint32_t current)
{
    return (int32_t)(current - previous);
}

float counts_to_revs(int32_t counts, float counts_per_rev)
{
    return (float)counts / counts_per_rev;
}

float speed_rpm(int32_t delta_counts, float counts_per_rev, float dt_seconds)
{
    if (dt_seconds <= 0.0f)
    {
        return 0.0f;
    }
    return ((float)delta_counts / counts_per_rev) / dt_seconds * 60.0f;
}