/* Host-side test for the tilt filter. Build and run on your PC (see README):
 *   gcc -Wall -Wextra -Iinclude test/test_tilt.c src/tilt.c -o test_tilt -lm && ./test_tilt
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "tilt.h"

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

/* Raw accelerometer values for a board tilted by `deg` about +Y (1 g = 16384). */
static void accel_for(float deg, int16_t *ax, int16_t *az)
{
    float r = deg * 3.14159265f / 180.0f;
    *ax = (int16_t)lroundf(-16384.0f * sinf(r));
    *az = (int16_t)lroundf(16384.0f * cosf(r));
}

static void test_accel_angle(void)
{
    int16_t ax, az;
    accel_for(0.0f, &ax, &az);
    CHECK(near(tilt_accel_angle_deg(ax, az), 0.0f, 0.05f), "flat board reads 0 deg");
    accel_for(30.0f, &ax, &az);
    CHECK(near(tilt_accel_angle_deg(ax, az), 30.0f, 0.05f), "+30 deg tilt reads +30 deg");
    accel_for(-45.0f, &ax, &az);
    CHECK(near(tilt_accel_angle_deg(ax, az), -45.0f, 0.05f), "-45 deg tilt reads -45 deg");
}

static void test_calibration(void)
{
    tilt_t t;
    tilt_init(&t, 0.98f, 0.005f);
    for (int i = 0; i < 200; i++)
    {
        tilt_calibrate_add(&t, (int16_t)(131 + (i % 3) - 1)); /* ~2.0 deg/s, tiny jitter */
    }
    tilt_calibrate_finish(&t);
    CHECK(near(t.gyro_bias_dps, 131.0f / 65.5f, 0.02f), "gyro bias averages to about 2 deg/s");

    tilt_t empty;
    tilt_init(&empty, 0.98f, 0.005f);
    tilt_calibrate_finish(&empty);
    CHECK(empty.gyro_bias_dps == 0.0f, "finishing with no samples leaves bias at 0");
}

static void test_first_sample_seeds_angle(void)
{
    tilt_t t;
    int16_t ax, az;
    tilt_init(&t, 0.98f, 0.005f);
    accel_for(20.0f, &ax, &az);
    float a = tilt_update(&t, ax, az, 0);
    CHECK(near(a, 20.0f, 0.05f), "first update starts at the accelerometer angle");
}

static void test_holds_still_with_bias(void)
{
    tilt_t t;
    int16_t ax, az;
    tilt_init(&t, 0.98f, 0.005f);
    for (int i = 0; i < 200; i++)
        tilt_calibrate_add(&t, 131);
    tilt_calibrate_finish(&t);
    accel_for(0.0f, &ax, &az);
    float a = 0;
    for (int i = 0; i < 4000; i++)
        a = tilt_update(&t, ax, az, 131); /* 20 s at rest */
    CHECK(near(a, 0.0f, 0.1f), "stays at 0 deg for 20 s at rest with calibrated bias");
}

static void test_tracks_rotation(void)
{
    /* Rotate +40 deg at 80 deg/s about +Y (0.5 s), then hold. */
    tilt_t t;
    int16_t ax, az;
    tilt_init(&t, 0.98f, 0.005f);
    accel_for(0.0f, &ax, &az);
    tilt_update(&t, ax, az, 0);
    float truth = 0.0f, est = 0.0f;
    for (int i = 0; i < 100; i++)
    {
        truth += 80.0f * 0.005f;
        accel_for(truth, &ax, &az);
        est = tilt_update(&t, ax, az, (int16_t)lroundf(80.0f * 65.5f));
    }
    CHECK(near(est, 40.0f, 1.0f), "tracks a +40 deg rotation (positive gyro rate = positive angle)");
    for (int i = 0; i < 1000; i++)
    {
        est = tilt_update(&t, ax, az, 0);
    }
    CHECK(near(est, 40.0f, 0.2f), "holds 40 deg afterward");
}

static void test_gyro_drift_is_corrected(void)
{
    /* Uncalibrated gyro offset of 3 deg/s: accel must pull the estimate back. */
    tilt_t t;
    int16_t ax, az;
    tilt_init(&t, 0.98f, 0.005f);
    accel_for(0.0f, &ax, &az);
    float a = 0;
    for (int i = 0; i < 4000; i++)
        a = tilt_update(&t, ax, az, (int16_t)lroundf(3.0f * 65.5f));
    CHECK(fabsf(a) < 1.5f, "3 deg/s uncorrected bias gives a bounded error (<1.5 deg)");
}

int main(void)
{
    test_accel_angle();
    test_calibration();
    test_first_sample_seeds_angle();
    test_holds_still_with_bias();
    test_tracks_rotation();
    test_gyro_drift_is_corrected();
    printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}