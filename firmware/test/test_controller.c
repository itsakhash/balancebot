/* Host-side test for the balance controllers. Build and run on your PC:
 *   gcc -Wall -Wextra -Iinclude test/test_controller.c src/controller.c -o /tmp/test_controller -lm && /tmp/test_controller
 *
 * It re-runs the pendulum simulation from the Python scripts (3 degree lean, a 40 deg/s
 * shove at 2 s, 200 Hz) with the C controllers and compares against numbers printed by
 * the Python versions (lqr_balance.py "Ideal sensors and motors" case). */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "controller.h"

#define G 9.81
#define L 0.10
#define DT 0.005
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

typedef struct
{
    double tilt_deg, speed, position;
} sample_t;

/* Runs the ideal simulation. use_lqr = 0 for PID, 1 for LQR. Records samples at
 * the given step indices. Returns the step at which it fell, or -1. */
static int simulate(int use_lqr, const int *at, sample_t *out, int count, int steps)
{
    static const float k[4] = {BALANCE_LQR_K0, BALANCE_LQR_K1, BALANCE_LQR_K2, BALANCE_LQR_K3};
    pid_ctrl_t pid;
    pid_init(&pid, BALANCE_PID_KP, BALANCE_PID_KI, BALANCE_PID_KD);

    double x = 0.0, v = 0.0, om = 0.0, th = 3.0 * PI / 180.0;
    int next = 0;
    for (int i = 0; i < steps; i++)
    {
        double t = i * DT;
        if (fabs(t - 2.0) < DT / 2)
        {
            om += 40.0 * PI / 180.0;
        }
        float accel;
        if (use_lqr)
        {
            accel = balance_lqr_accel(k, (float)x, (float)v, (float)th, (float)om, BALANCE_MAX_ACCEL);
        }
        else
        {
            accel = balance_pid_accel(&pid, (float)th, (float)om, (float)DT, BALANCE_MAX_ACCEL);
        }
        om += (G * sin(th) - accel * cos(th)) / L * DT;
        th += om * DT;
        v += accel * DT;
        x += v * DT;
        if (next < count && i == at[next])
        {
            out[next].tilt_deg = th * 180.0 / PI;
            out[next].speed = v;
            out[next].position = x;
            next++;
        }
        if (fabs(th) > PI / 2)
        {
            return i;
        }
    }
    return -1;
}

static void test_basics(void)
{
    pid_ctrl_t pid;
    pid_init(&pid, 2.0f, 1.0f, 0.5f);
    float out = pid_update(&pid, 1.0f, 2.0f, 0.1f);
    CHECK(near(out, 2.0 * 1.0 + 1.0 * 0.1 + 0.5 * 2.0, 1e-5), "pid: P + I + D terms add up");
    out = pid_update(&pid, 1.0f, 2.0f, 0.1f);
    CHECK(near(pid.integral, 0.2, 1e-5), "pid: integral accumulates error * dt");
    pid_reset(&pid);
    CHECK(pid.integral == 0.0f, "pid: reset clears the integral");

    pid_init(&pid, 30.0f, 0.0f, 1.5f);
    CHECK(balance_pid_accel(&pid, 0.01f, 0.0f, 0.005f, 5.0f) > 0.0f, "pid: leaning forward drives forward");
    CHECK(balance_pid_accel(&pid, -0.01f, 0.0f, 0.005f, 5.0f) < 0.0f, "pid: leaning back drives backward");
    CHECK(balance_pid_accel(&pid, 1.0f, 0.0f, 0.005f, 5.0f) == 5.0f, "pid: output clamps at +max_accel");
    CHECK(balance_pid_accel(&pid, -1.0f, 0.0f, 0.005f, 5.0f) == -5.0f, "pid: output clamps at -max_accel");

    const float k[4] = {BALANCE_LQR_K0, BALANCE_LQR_K1, BALANCE_LQR_K2, BALANCE_LQR_K3};
    CHECK(balance_lqr_accel(k, 0, 0, 0, 0, 5.0f) == 0.0f, "lqr: upright and still gives zero");
    CHECK(balance_lqr_accel(k, 0, 0, 0.01f, 0, 5.0f) > 0.0f, "lqr: leaning forward drives forward");
    CHECK(balance_lqr_accel(k, 0.1f, 0, 0, 0, 5.0f) > 0.0f, "lqr: position term has the sign from the simulation (k0 < 0)");
    CHECK(balance_lqr_accel(k, 0, 0, 1.0f, 0, 5.0f) == 5.0f, "lqr: output clamps at +max_accel");
    CHECK(near(balance_lqr_accel(k, 0, 0, 0.01f, 0, 50.0f), 29.71225976 * 0.01, 1e-4), "lqr: tilt gain matches K2");

    CHECK(accel_to_command(2.5f, 5.0f) == 0.5f, "command: half of max_accel is 0.5");
    CHECK(accel_to_command(-9.0f, 5.0f) == -1.0f, "command: clamps at -1");
    CHECK(accel_to_command(1.0f, 0.0f) == 0.0f, "command: zero max_accel gives zero, no divide by zero");

    CHECK(balance_should_cut(0.6f, 0.5f) == 1, "cutoff: leaning past the limit cuts");
    CHECK(balance_should_cut(-0.6f, 0.5f) == 1, "cutoff: works for negative tilt");
    CHECK(balance_should_cut(0.4f, 0.5f) == 0, "cutoff: inside the limit keeps running");
}

static void test_against_python(void)
{
    /* Reference values printed by lqr_balance.py's simulate() with ideal sensors. */
    const int at[] = {200, 400, 600, 800, 1200, 1598};
    sample_t s[6];

    int fell = simulate(0, at, s, 6, 1600);
    CHECK(fell == -1, "PID stays upright for 8 s");
    CHECK(near(s[0].tilt_deg, 0.000933, 0.005), "PID tilt at 1 s matches Python");
    CHECK(near(s[1].tilt_deg, 0.185000, 0.005), "PID tilt at 2 s matches Python (just after the shove)");
    CHECK(near(s[2].speed, 0.139444, 0.003), "PID speed at 3 s matches Python");
    CHECK(near(s[5].speed, 0.139485, 0.003), "PID ends with a steady drift speed of about 0.14 m/s");
    CHECK(near(s[5].position, 0.916049, 0.03), "PID position at 8 s matches Python (it rolls away)");

    fell = simulate(1, at, s, 6, 1600);
    CHECK(fell == -1, "LQR stays upright for 8 s");
    CHECK(near(s[1].tilt_deg, 0.240995, 0.02), "LQR tilt at 2 s matches Python");
    CHECK(near(s[2].tilt_deg, -0.287978, 0.02), "LQR tilt at 3 s matches Python");
    CHECK(near(s[4].tilt_deg, 0.013242, 0.02), "LQR tilt at 6 s matches Python");
    CHECK(fabs(s[5].speed) < 0.01, "LQR speed settles near zero (no drift)");
    CHECK(fabs(s[5].position) < 0.01, "LQR position settles near the start");
}

int main(void)
{
    test_basics();
    test_against_python();
    printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}