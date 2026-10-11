#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>

#include "controller.h"

/* The balance supervisor: arming, tip-over cutoff, and the controller call.
 * Pure C with no hardware access, so it is tested on the PC (test/test_balance.c). */

#define BALANCE_WHEEL_DIAMETER_M 0.065f
#define BALANCE_ARM_WINDOW_DEG 3.0f /* must be this close to upright to arm...        */
#define BALANCE_ARM_RATE_DPS 30.0f  /* ...and rotating slower than this               */
#define BALANCE_ARM_HOLD_S 1.0f     /* ...for this long                               */
#define BALANCE_CUT_DEG 35.0f       /* past this the motors are cut                   */
#define BALANCE_SPEED_LP 0.2f       /* speed filter: fraction of new data per step    */

typedef enum
{
    BAL_DISARMED = 0,
    BAL_RUNNING = 1
} bal_state_t;

typedef struct
{
    float tilt_deg;   /* from the complementary filter, + = +X side down          */
    float rate_dps;   /* gyro-Y rate with bias removed, + = tilt increasing       */
    float position_m; /* average of both wheels, forward = +                      */
} bal_input_t;

typedef struct
{
    /* settings (change after balance_init if needed) */
    int use_lqr;     /* 1 = LQR, 0 = PID                                   */
    float tilt_sign; /* +1 or -1: which way "lean forward" is on the board   */
    float trim_deg;  /* balance point offset (accelerometer error)           */
    float arm_window_deg;
    float arm_rate_dps;
    float arm_hold_s;
    float cut_deg;
    float max_accel;
    float k[4];
    pid_ctrl_t pid;

    /* state */
    bal_state_t state;
    float held_s; /* how long we have been upright and still             */
    float prev_pos_m;
    float pos_origin_m; /* position when we armed; the LQR position error is relative to this */
    float speed_mps;    /* filtered speed                                      */
    float tilt_rad;     /* last tilt after trim and sign, for printing         */
    float accel;        /* last wheel acceleration request                     */
    uint32_t falls;     /* how many times the cutoff has fired                 */
} balance_t;

void balance_init(balance_t *b, int use_lqr);

/* One control step at a steady dt. Returns the motor command in -1..+1
 * (always 0 while disarmed). */
float balance_step(balance_t *b, const bal_input_t *in, float dt);

/* Immediately disarm (for example after IMU read failures). */
void balance_force_disarm(balance_t *b);

/* Encoder counts to metres of wheel travel. */
float counts_to_meters(float counts, float counts_per_rev, float wheel_diameter_m);

/* Gyro reading in deg/s with the resting bias removed. */
float gyro_dps(int16_t raw, float lsb_per_dps, float bias_dps);

#endif