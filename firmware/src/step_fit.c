#include "step_fit.h"

#include <math.h>

#define LEVEL_LOW 0.25f
#define LEVEL_HIGH 0.632121f /* 1 - 1/e */
#define MIN_STEADY_CPS 40.0f /* below this the motor effectively did not move */
#define MAX_N 512

/* Time at which `rate` (a smoothed speed, counts per sample) first reaches `level`,
 * with linear interpolation between samples. Returns -1 if it never does. */
static float first_crossing(const float *rate, const float *when, int n, float level)
{
    for (int i = 0; i < n; i++)
    {
        if (rate[i] >= level)
        {
            if (i == 0)
                return when[0];
            float span = rate[i] - rate[i - 1];
            float frac = (span > 0.0f) ? (level - rate[i - 1]) / span : 1.0f;
            return when[i - 1] + frac * (when[i] - when[i - 1]);
        }
    }
    return -1.0f;
}

/* Time of the last upward crossing of `level` that happens before `t_end`: a noise blip that
 * pokes above the level early does not count unless the speed stays up. Returns -1 if none. */
static float last_crossing_before(const float *rate, const float *when, int n, float level, float t_end)
{
    float found = -1.0f;
    for (int i = 0; i < n && (i == 0 || when[i - 1] < t_end); i++)
    {
        if (rate[i] >= level && (i == 0 || rate[i - 1] < level))
        {
            if (i == 0)
                found = when[0];
            else
            {
                float span = rate[i] - rate[i - 1];
                float frac = (span > 0.0f) ? (level - rate[i - 1]) / span : 1.0f;
                found = when[i - 1] + frac * (when[i] - when[i - 1]);
            }
        }
    }
    return found;
}

int step_fit(const int16_t *deltas, int n, float dt, int smooth, step_fit_t *out)
{
    float rate[MAX_N]; /* smoothed speed in counts per sample */
    float when[MAX_N]; /* time the smoothed value refers to */

    out->ok = 0;
    out->steady_cps = out->t25_s = out->t63_s = out->delay_s = out->tau_s = 0.0f;
    if (deltas == 0 || n < 16 || n > MAX_N || dt <= 0.0f || smooth < 1 || smooth > n / 4)
        return 0;

    /* Trailing moving average. The window covers intervals i-m+1..i, so it is centred
     * on time (i + 1 - m/2) * dt. The first few windows are shorter. */
    for (int i = 0; i < n; i++)
    {
        int m = (i + 1 < smooth) ? i + 1 : smooth;
        float sum = 0.0f;
        for (int j = i - m + 1; j <= i; j++)
            sum += (float)deltas[j];
        rate[i] = sum / (float)m;
        when[i] = ((float)(i + 1) - 0.5f * (float)m) * dt;
    }

    int tail = n / 4;
    float tail_sum = 0.0f;
    for (int i = n - tail; i < n; i++)
        tail_sum += (float)deltas[i];
    float steady = tail_sum / (float)tail; /* counts per sample */
    out->steady_cps = steady / dt;
    if (out->steady_cps < MIN_STEADY_CPS)
        return 0;

    float t_high = first_crossing(rate, when, n, LEVEL_HIGH * steady);
    float t_low = last_crossing_before(rate, when, n, LEVEL_LOW * steady, t_high);
    if (t_low < 0.0f || t_high < 0.0f || t_high <= t_low)
        return 0;

    /* For y = 1 - exp(-(t-d)/tau):  t25 = d + 0.28768 tau,  t63 = d + tau. */
    const float k10 = 0.287682f; /* -ln(0.75) */
    float tau = (t_high - t_low) / (1.0f - k10);
    float delay = t_low - k10 * tau;
    if (delay < 0.0f)
        delay = 0.0f;

    out->t25_s = t_low;
    out->t63_s = t_high;
    out->tau_s = tau;
    out->delay_s = delay;
    out->ok = 1;
    return 1;
}

int fit_line(const float *x, const float *y, int n, float *slope, float *intercept)
{
    if (n < 2)
        return 0;
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    for (int i = 0; i < n; i++)
    {
        sx += x[i];
        sy += y[i];
        sxx += x[i] * x[i];
        sxy += x[i] * y[i];
    }
    float denom = (float)n * sxx - sx * sx;
    if (fabsf(denom) < 1e-9f)
        return 0;
    *slope = ((float)n * sxy - sx * sy) / denom;
    *intercept = (sy - *slope * sx) / (float)n;
    return 1;
}

float command_for_accel(const motor_model_t *m, float accel, float speed)
{
    if (m->gain <= 0.0f || m->tau <= 0.0f)
        return 0.0f;
    float u = (accel * m->tau + speed) / m->gain;
    if (u > 1.0f)
        u = 1.0f;
    if (u < -1.0f)
        u = -1.0f;
    return u;
}