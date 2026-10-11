#ifndef STEP_FIT_H
#define STEP_FIT_H

#include <stdint.h>

/* Result of fitting "dead time + first-order lag" to a motor step response:
 *     speed(t) = steady * (1 - exp(-(t - delay) / tau))   for t > delay
 * Times are in seconds, speeds in encoder counts per second. */
typedef struct
{
    int ok;           /* 1 if the fit succeeded, 0 if the motor did not move or never settled */
    float steady_cps; /* final speed, counts per second (mean of the last quarter of the record) */
    float t25_s;      /* time the speed reaches 25% of steady */
    float t63_s;      /* time the speed first reaches 63.2% of steady */
    float delay_s;    /* dead time before the speed starts to rise (never below 0) */
    float tau_s;      /* lag time constant */
} step_fit_t;

/* deltas[i] is the number of encoder counts gained in sample interval i (interval i
 * covers t = i*dt to (i+1)*dt after the command was applied). smooth is the number of
 * samples averaged to tame the count quantization (4 is a good start). Returns out->ok. */
int step_fit(const int16_t *deltas, int n, float dt, int smooth, step_fit_t *out);

/* Least-squares line y = slope * x + intercept. Returns 1 on success, 0 if n < 2 or
 * all x are equal. */
int fit_line(const float *x, const float *y, int n, float *slope, float *intercept);

/* First-order motor model: speed follows  d(speed)/dt = (gain * cmd - speed) / tau.
 * gain is the steady speed per unit command (m/s per 1.0). */
typedef struct
{
    float gain;
    float tau;
} motor_model_t;

/* Command (-1..+1) that makes a wheel accelerate at `accel` (m/s^2) when it is
 * currently moving at `speed` (m/s), under the model above. Returns 0 if the model
 * is invalid (gain or tau not positive). */
float command_for_accel(const motor_model_t *m, float accel, float speed);

#endif