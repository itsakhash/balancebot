#include "balance.h"

#define DEG_TO_RAD 0.017453292519943295f
#define PI_F 3.14159265f

static float absf(float x) { return x < 0.0f ? -x : x; }
void balance_init(balance_t *b, int use_lqr)
{
    b->use_lqr = use_lqr;
    b->tilt_sign = 1.0f;
    b->trim_deg = 0.0f;
    b->arm_window_deg = BALANCE_ARM_WINDOW_DEG;
    b->arm_rate_dps = BALANCE_ARM_RATE_DPS;
    b->arm_hold_s = BALANCE_ARM_HOLD_S;
    b->cut_deg = BALANCE_CUT_DEG;
    b->max_accel = BALANCE_MAX_ACCEL;
    b->k[0] = BALANCE_LQR_K0;
    b->k[1] = BALANCE_LQR_K1;
    b->k[2] = BALANCE_LQR_K2;
    b->k[3] = BALANCE_LQR_K3;
    pid_init(&b->pid, BALANCE_PID_KP, BALANCE_PID_KI, BALANCE_PID_KD);

    b->state = BAL_DISARMED;
    b->held_s = 0.0f;
    b->prev_pos_m = 0.0f;
    b->pos_origin_m = 0.0f;
    b->speed_mps = 0.0f;
    b->tilt_rad = 0.0f;
    b->accel = 0.0f;
    b->falls = 0;
}

void balance_force_disarm(balance_t *b)
{
    b->state = BAL_DISARMED;
    b->held_s = 0.0f;
    b->accel = 0.0f;
}

float balance_step(balance_t *b, const bal_input_t *in, float dt)
{
    float tilt_deg = b->tilt_sign * (in->tilt_deg - b->trim_deg);
    float rate_dps = b->tilt_sign * in->rate_dps;
    float tilt = tilt_deg * DEG_TO_RAD;
    float rate = rate_dps * DEG_TO_RAD;
    b->tilt_rad = tilt;

    /* Speed from the change in position, low-pass filtered (encoder counts are coarse). */
    if (dt > 0.0f)
    {
        float raw_speed = (in->position_m - b->prev_pos_m) / dt;
        b->speed_mps += BALANCE_SPEED_LP * (raw_speed - b->speed_mps);
    }
    b->prev_pos_m = in->position_m;

    if (b->state == BAL_DISARMED)
    {
        b->accel = 0.0f;
        if (absf(tilt_deg) < b->arm_window_deg && absf(rate_dps) < b->arm_rate_dps)
        {
            b->held_s += dt;
        }
        else
        {
            b->held_s = 0.0f;
        }
        if (b->held_s >= b->arm_hold_s)
        {
            b->state = BAL_RUNNING;
            b->pos_origin_m = in->position_m;
            b->speed_mps = 0.0f;
            pid_reset(&b->pid);
        }
        return 0.0f;
    }

    /* RUNNING */
    if (absf(tilt_deg) > b->cut_deg)
    {
        balance_force_disarm(b);
        b->falls++;
        return 0.0f;
    }

    if (b->use_lqr)
    {
        b->accel = balance_lqr_accel(b->k, in->position_m - b->pos_origin_m, b->speed_mps, tilt,
                                     rate, b->max_accel);
    }
    else
    {
        b->accel = balance_pid_accel(&b->pid, tilt, rate, dt, b->max_accel);
    }
    return accel_to_command(b->accel, b->max_accel);
}

float counts_to_meters(float counts, float counts_per_rev, float wheel_diameter_m)
{
    return counts / counts_per_rev * PI_F * wheel_diameter_m;
}

float gyro_dps(int16_t raw, float lsb_per_dps, float bias_dps)
{
    return (float)raw / lsb_per_dps - bias_dps;
}