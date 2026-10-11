/* Host-side test for the motor and encoder math. Build and run on your PC:
 *   gcc -Wall -Wextra -Iinclude test/test_drive_math.c src/drive_math.c -o /tmp/test_drive_math -lm && /tmp/test_drive_math
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "drive_math.h"

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

static void test_motor_command(void)
{
    motor_drive_t d;

    motor_drive_from_command(0.0f, 0, 800, 0.0f, &d);
    CHECK(d.in1 == 0 && d.in2 == 0 && d.duty == 0, "zero command coasts");

    motor_drive_from_command(0.0005f, 0, 800, 0.0f, &d);
    CHECK(d.in1 == 0 && d.in2 == 0 && d.duty == 0, "tiny command coasts");

    motor_drive_from_command(1.0f, 0, 800, 0.0f, &d);
    CHECK(d.in1 == 1 && d.in2 == 0 && d.duty == 800, "full forward");

    motor_drive_from_command(-0.5f, 0, 800, 0.0f, &d);
    CHECK(d.in1 == 0 && d.in2 == 1 && d.duty == 400, "half reverse");

    motor_drive_from_command(5.0f, 0, 800, 0.0f, &d);
    CHECK(d.in1 == 1 && d.duty == 800, "command above +1 is clamped");

    motor_drive_from_command(-5.0f, 0, 800, 0.0f, &d);
    CHECK(d.in2 == 1 && d.duty == 800, "command below -1 is clamped");

    motor_drive_from_command(0.25f, 1, 800, 0.0f, &d);
    CHECK(d.in1 == 0 && d.in2 == 1 && d.duty == 200, "invert flips direction, same duty");

    motor_drive_from_command(0.5f, 0, 800, 0.2f, &d);
    CHECK(d.duty == 480, "deadband 0.2: command 0.5 maps to 60% duty (480)");

    motor_drive_from_command(0.01f, 0, 800, 0.2f, &d);
    CHECK(d.duty > 160 && d.duty < 170, "deadband 0.2: tiny command starts just above 20% duty");

    motor_drive_from_command(1.0f, 0, 800, 0.2f, &d);
    CHECK(d.duty == 800, "deadband does not reduce full power");

    motor_drive_from_command(0.5f, 0, 800, -1.0f, &d);
    CHECK(d.duty == 400, "negative deadband is treated as 0");
}

static void test_encoder_delta(void)
{
    CHECK(encoder_delta16(100, 110) == 10, "16-bit forward count");
    CHECK(encoder_delta16(110, 100) == -10, "16-bit backward count");
    CHECK(encoder_delta16(65535, 2) == 3, "16-bit forward across wraparound");
    CHECK(encoder_delta16(2, 65534) == -4, "16-bit backward across wraparound");
    CHECK(encoder_delta16(1234, 1234) == 0, "16-bit no change");

    CHECK(encoder_delta32(0xFFFFFFFEu, 3u) == 5, "32-bit forward across wraparound");
    CHECK(encoder_delta32(3u, 0xFFFFFFFEu) == -5, "32-bit backward across wraparound");
    CHECK(encoder_delta32(1000u, 1500u) == 500, "32-bit forward");
}

static void test_speed(void)
{
    float cpr = 11.0f * 4.0f * 21.3f; /* 937.2 */
    CHECK(near(counts_to_revs(1874, cpr), 2.0f, 0.001f), "1874 counts is about 2 wheel turns");
    CHECK(near(speed_rpm(937, cpr, 1.0f), 60.0f, 0.1f), "one turn in one second is 60 RPM");
    CHECK(near(speed_rpm(-469, cpr, 0.1f), -300.0f, 1.0f), "reverse speed is negative");
    CHECK(speed_rpm(100, cpr, 0.0f) == 0.0f, "zero dt gives zero speed, no divide by zero");
}

int main(void)
{
    test_motor_command();
    test_encoder_delta();
    test_speed();
    printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}