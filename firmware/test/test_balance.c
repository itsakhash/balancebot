/* Host-side test for the balance supervisor. Build and run on your PC:
 *   gcc -Wall -Wextra -Iinclude test/test_balance.c src/balance.c src/controller.c -o /tmp/test_balance -lm && /tmp/test_balance
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "balance.h"

#define DT 0.005f
#define PI 3.14159265358979323846

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

static int near(double a, double b, double tol) { return fabs(a - b) <= tol; }

/* Feed the same input for the given number of seconds; returns the last command. */
static float hold(balance_t *b, float tilt_deg, float rate_dps, float pos, float seconds)
{
    bal_input_t in = {tilt_deg, rate_dps, pos};
    float cmd = 0.0f;
    int steps = (int)(seconds / DT + 0.5f);
    for (int i = 0; i < steps; i++)
    {
        cmd = balance_step(b, &in, DT);
    }
    return cmd;
}

static void test_helpers(void)
{
    float one_rev = counts_to_meters(937.2f, 937.2f, 0.065f);
    CHECK(near(one_rev, PI * 0.065, 1e-4), "one wheel revolution is pi * diameter (0.2042 m)");
    CHECK(near(counts_to_meters(-468.6f, 937.2f, 0.065f), -PI * 0.065 / 2.0, 1e-4), "half a turn backward is negative");
    CHECK(near(gyro_dps(655, 65.5f, 0.0f), 10.0, 1e-4), "gyro: 655 raw is 10 deg/s");
    CHECK(near(gyro_dps(79, 65.5f, 1.2f), 0.006, 0.01), "gyro: resting bias is subtracted");
}

static void test_arming(void)
{
    balance_t b;
    balance_init(&b, 1);
    CHECK(b.state == BAL_DISARMED, "starts disarmed");

    float cmd = hold(&b, 10.0f, 0.0f, 0.0f, 2.0f);
    CHECK(b.state == BAL_DISARMED && cmd == 0.0f, "stays disarmed while tilted 10 degrees");

    cmd = hold(&b, 0.5f, 0.0f, 0.0f, 0.8f);
    CHECK(b.state == BAL_DISARMED && cmd == 0.0f, "not armed after only 0.8 s upright");
    cmd = hold(&b, 0.5f, 0.0f, 0.0f, 0.3f);
    CHECK(b.state == BAL_RUNNING, "armed after 1 s upright");

    balance_init(&b, 1);
    hold(&b, 0.5f, 0.0f, 0.0f, 0.8f);
    hold(&b, 6.0f, 0.0f, 0.0f, 0.1f);
    hold(&b, 0.5f, 0.0f, 0.0f, 0.8f);
    CHECK(b.state == BAL_DISARMED, "leaving the window resets the hold timer");

    balance_init(&b, 1);
    hold(&b, 0.5f, 100.0f, 0.0f, 2.0f);
    CHECK(b.state == BAL_DISARMED, "does not arm while rotating fast");

    balance_init(&b, 1);
    hold(&b, 0.5f, 0.0f, 0.0f, 1.2f);
    balance_force_disarm(&b);
    CHECK(b.state == BAL_DISARMED && b.accel == 0.0f, "force disarm works");
}

static void test_running(void)
{
    balance_t b;
    balance_init(&b, 1);
    hold(&b, 0.0f, 0.0f, 0.0f, 1.2f);
    float cmd = hold(&b, 2.0f, 0.0f, 0.0f, 0.01f);
    CHECK(cmd > 0.0f, "running: leaning forward gives a positive command");
    cmd = hold(&b, -2.0f, 0.0f, 0.0f, 0.01f);
    CHECK(cmd < 0.0f, "running: leaning back gives a negative command");

    balance_init(&b, 1);
    b.tilt_sign = -1.0f;
    hold(&b, 0.0f, 0.0f, 0.0f, 1.2f);
    cmd = hold(&b, 2.0f, 0.0f, 0.0f, 0.01f);
    CHECK(cmd < 0.0f, "tilt_sign = -1 flips the command");

    balance_init(&b, 1);
    b.trim_deg = 2.0f;
    hold(&b, 2.0f, 0.0f, 0.0f, 1.2f);
    CHECK(b.state == BAL_RUNNING, "trim: arms with the board at the trim angle");
    cmd = hold(&b, 2.0f, 0.0f, 0.0f, 0.01f);
    CHECK(near(cmd, 0.0, 1e-4), "trim: balance point is shifted by trim_deg");

    balance_init(&b, 1);
    hold(&b, 0.0f, 0.0f, 10.0f, 1.2f);
    cmd = hold(&b, 0.0f, 0.0f, 10.0f, 0.01f);
    CHECK(near(cmd, 0.0, 1e-4), "position is relative to where it armed (arming at 10 m is fine)");

    balance_init(&b, 1);
    hold(&b, 0.0f, 0.0f, 0.0f, 1.2f);
    bal_input_t in = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 400; i++)
    {
        in.position_m += 0.5f * DT; /* rolling at 0.5 m/s */
        balance_step(&b, &in, DT);
    }
    CHECK(near(b.speed_mps, 0.5, 0.02), "speed filter converges to the true speed");

    balance_init(&b, 0);
    hold(&b, 0.0f, 0.0f, 0.0f, 1.2f);
    cmd = hold(&b, 2.0f, 0.0f, 0.0f, 0.01f);
    CHECK(cmd > 0.0f, "PID mode: leaning forward gives a positive command");
}

static void test_cutoff(void)
{
    balance_t b;
    balance_init(&b, 1);
    hold(&b, 0.0f, 0.0f, 0.0f, 1.2f);
    float cmd = hold(&b, 40.0f, 0.0f, 0.0f, 0.01f);
    CHECK(b.state == BAL_DISARMED && cmd == 0.0f && b.falls == 1, "tip past 35 degrees cuts the motors");
    cmd = hold(&b, 5.0f, 0.0f, 0.0f, 3.0f);
    CHECK(b.state == BAL_DISARMED && cmd == 0.0f, "stays off while still tilted");
    hold(&b, 0.0f, 0.0f, 0.0f, 1.2f);
    CHECK(b.state == BAL_RUNNING, "re-arms after being held upright again");
}

/* Closed loop: the supervisor driving the simulated pendulum (ideal sensors and motors).
 * The placeholder mapping is accel = command * max_accel. */
static void test_closed_loop(void)
{
    for (int mode = 0; mode < 2; mode++)
    {
        balance_t b;
        balance_init(&b, mode);
        double x = 0, v = 0, om = 0, th = 0;
        const double G = 9.81, L = 0.10;
        int ever_cut = 0;
        for (int i = 0; i < 1600; i++)
        { /* 8 s */
            double t = i * (double)DT;
            if (fabs(t - 2.0) < DT / 2)
            {
                om += 40.0 * PI / 180.0; /* shove */
            }
            bal_input_t in = {(float)(th * 180.0 / PI), (float)(om * 180.0 / PI), (float)x};
            float cmd = balance_step(&b, &in, DT);
            double accel = cmd * BALANCE_MAX_ACCEL;
            om += (G * sin(th) - accel * cos(th)) / L * DT;
            th += om * DT;
            v += accel * DT;
            x += v * DT;
            if (b.falls > 0)
            {
                ever_cut = 1;
            }
        }
        CHECK(!ever_cut, mode ? "closed loop LQR: shove at 2 s never trips the cutoff"
                              : "closed loop PID: shove at 2 s never trips the cutoff");
        CHECK(b.state == BAL_RUNNING, mode ? "closed loop LQR: still running at 8 s"
                                           : "closed loop PID: still running at 8 s");
        CHECK(fabs(th * 180.0 / PI) < 0.1, mode ? "closed loop LQR: upright at 8 s"
                                                : "closed loop PID: upright at 8 s");
    }
}

int main(void)
{
    test_helpers();
    test_arming();
    test_running();
    test_cutoff();
    test_closed_loop();
    printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}