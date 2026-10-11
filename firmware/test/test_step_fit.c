/* Host-side test for the motor step-response fit. Build and run on your PC (from firmware/):
 *   gcc -Wall -Wextra -Iinclude test/test_step_fit.c src/step_fit.c -o /tmp/test_step_fit -lm && /tmp/test_step_fit
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "step_fit.h"

static int failures = 0;

#define CHECK(cond, msg)               \
    do                                 \
    {                                  \
        if (!(cond))                   \
        {                              \
            printf("FAIL: %s\n", msg); \
            failures++;                \
        }                              \
        else                           \
        {                              \
            printf("ok:   %s\n", msg); \
        }                              \
    } while (0)

static int near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

#define N 200
#define DT 0.005f

/* Fake encoder log of a motor with the given steady speed, delay and lag. Counts are
 * whole numbers, as on the real encoder: the integer part of the true position is
 * kept and the remainder carries to the next sample. noise adds +/-noise counts. */
static void make_step(int16_t *d, float steady_cps, float delay, float tau, int noise, unsigned seed)
{
    srand(seed);
    long prev = 0;
    for (int i = 0; i < N; i++)
    {
        float t = (float)(i + 1) * DT;
        float pos = 0.0f;
        if (t > delay)
        {
            float u = t - delay;
            pos = steady_cps * (u - tau * (1.0f - expf(-u / tau)));
        }
        long now = (long)floorf(pos);
        long delta = now - prev;
        prev = now;
        if (noise > 0)
            delta += (rand() % (2 * noise + 1)) - noise;
        d[i] = (int16_t)delta;
    }
}

static void test_clean_fit(void)
{
    int16_t d[N];
    step_fit_t f;

    make_step(d, 3000.0f, 0.020f, 0.060f, 0, 1);
    int ok = step_fit(d, N, DT, 4, &f);
    CHECK(ok == 1 && f.ok == 1, "clean step fits");
    CHECK(near(f.steady_cps, 3000.0f, 60.0f), "steady speed recovered (within 2%)");
    CHECK(near(f.tau_s, 0.060f, 0.008f), "tau 60 ms recovered (within 8 ms)");
    CHECK(near(f.delay_s, 0.020f, 0.008f), "delay 20 ms recovered (within 8 ms)");

    make_step(d, 3000.0f, 0.0f, 0.030f, 0, 1);
    step_fit(d, N, DT, 4, &f);
    CHECK(f.ok && near(f.tau_s, 0.030f, 0.008f), "tau 30 ms, no delay recovered");
    CHECK(f.delay_s >= 0.0f && f.delay_s < 0.008f, "no delay reads near zero and never negative");

    make_step(d, 1500.0f, 0.010f, 0.120f, 0, 1);
    step_fit(d, N, DT, 4, &f);
    CHECK(f.ok && near(f.tau_s, 0.120f, 0.015f), "slow motor (tau 120 ms) recovered");
}

static void test_noisy_fit(void)
{
    int16_t d[N];
    step_fit_t f;
    int good = 0;
    for (unsigned seed = 1; seed <= 20; seed++)
    {
        /* +/-1 count of random noise on top of the rounding: about three times the
         * quantization noise the real encoder should have. */
        make_step(d, 3000.0f, 0.015f, 0.050f, 1, seed);
        if (step_fit(d, N, DT, 4, &f) && near(f.tau_s, 0.050f, 0.010f))
            good++;
    }
    CHECK(good == 20, "tau within 10 ms in all 20 noisy runs");
}

static void test_failures(void)
{
    int16_t d[N] = {0};
    step_fit_t f;

    CHECK(step_fit(d, N, DT, 4, &f) == 0 && f.ok == 0, "motor that never moves: no fit");

    for (int i = 0; i < N; i++)
        d[i] = (i < 3) ? 0 : 15; /* 3000 counts/s, jumps at once */
    int ok = step_fit(d, N, DT, 1, &f);
    CHECK(ok == 1 && f.tau_s < 0.010f, "instant jump gives a tiny tau");

    CHECK(step_fit(d, 10, DT, 4, &f) == 0, "too few samples rejected");
    CHECK(step_fit(d, N, 0.0f, 4, &f) == 0, "dt of zero rejected");
    CHECK(step_fit(d, N, DT, 0, &f) == 0, "smooth of zero rejected");
    CHECK(step_fit(0, N, DT, 4, &f) == 0, "null data rejected");
    CHECK(step_fit(d, 2000, DT, 4, &f) == 0, "record longer than the buffer rejected");

    for (int i = 0; i < N; i++)
        d[i] = (int16_t)(i < N - 50 ? 20 : 2); /* speed collapses at the end */
    CHECK(step_fit(d, N, DT, 4, &f) == 0, "never reaches 63% of a tiny final speed: no fit");
}

static void test_fit_line(void)
{
    float x[] = {0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
    float y[5];
    float slope, intercept;

    for (int i = 0; i < 5; i++)
        y[i] = 2000.0f * x[i] - 300.0f;
    CHECK(fit_line(x, y, 5, &slope, &intercept) == 1, "line fit succeeds");
    CHECK(near(slope, 2000.0f, 0.5f), "slope recovered");
    CHECK(near(intercept, -300.0f, 0.5f), "intercept recovered");
    CHECK(near(-intercept / slope, 0.15f, 0.001f), "zero-speed command (dead zone) is 15%");

    CHECK(fit_line(x, y, 1, &slope, &intercept) == 0, "one point rejected");
    float same[] = {0.3f, 0.3f, 0.3f};
    CHECK(fit_line(same, y, 3, &slope, &intercept) == 0, "equal x values rejected");
}

static void test_command_for_accel(void)
{
    motor_model_t m = {0.60f, 0.05f}; /* 0.6 m/s at full command, 50 ms lag */

    CHECK(near(command_for_accel(&m, 0.0f, 0.0f), 0.0f, 1e-6f), "no accel from rest: zero command");
    CHECK(near(command_for_accel(&m, 0.0f, 0.30f), 0.5f, 1e-5f), "holding 0.30 m/s needs 50%");
    CHECK(near(command_for_accel(&m, 6.0f, 0.0f), 0.5f, 1e-5f), "6 m/s^2 from rest needs 50% (6*0.05/0.6)");
    CHECK(near(command_for_accel(&m, -6.0f, 0.0f), -0.5f, 1e-5f), "negative accel gives negative command");
    CHECK(near(command_for_accel(&m, 100.0f, 0.0f), 1.0f, 1e-6f), "clamped to +1");
    CHECK(near(command_for_accel(&m, -100.0f, 0.0f), -1.0f, 1e-6f), "clamped to -1");

    motor_model_t bad = {0.0f, 0.05f};
    CHECK(near(command_for_accel(&bad, 5.0f, 0.0f), 0.0f, 1e-6f), "zero gain: zero command");
    motor_model_t bad2 = {0.6f, 0.0f};
    CHECK(near(command_for_accel(&bad2, 5.0f, 0.0f), 0.0f, 1e-6f), "zero tau: zero command");

    /* Closed check: simulate the motor with the model and the command, accel should track. */
    float v = 0.0f, dt = 0.001f;
    for (int i = 0; i < 100; i++)
    {
        float u = command_for_accel(&m, 2.0f, v);
        v += dt * (m.gain * u - v) / m.tau;
    }
    CHECK(near(v, 0.2f, 0.002f), "commanding 2 m/s^2 for 0.1 s reaches 0.2 m/s");
}

int main(void)
{
    test_clean_fit();
    test_noisy_fit();
    test_failures();
    test_fit_line();
    test_command_for_accel();

    if (failures == 0)
        printf("\nALL TESTS PASSED (0 failures)\n");
    else
        printf("\n%d FAILURE(S)\n", failures);
    return failures == 0 ? 0 : 1;
}