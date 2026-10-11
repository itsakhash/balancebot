#ifndef CONTROLLER_H
#define CONTROLLER_H

/* Balance controllers, ported from the Python simulation (pid_balance.py and
 * lqr_balance.py). Both return a wheel acceleration in m/s^2: positive means
 * drive the wheels forward. Angles are in radians, positive = leaning forward. */

/* Gains and limits used in the simulation. */
#define BALANCE_MAX_ACCEL 5.0f
#define BALANCE_PID_KP 30.0f
#define BALANCE_PID_KI 0.0f
#define BALANCE_PID_KD 1.5f
/* u = -K z with z = [position m, speed m/s, tilt rad, tilt rate rad/s] */
#define BALANCE_LQR_K0 (-3.16227766f)
#define BALANCE_LQR_K1 (-3.71901666f)
#define BALANCE_LQR_K2 (-29.71225976f)
#define BALANCE_LQR_K3 (-2.99663144f)

typedef struct
{
    float kp, ki, kd;
    float integral;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd);
void pid_reset(pid_ctrl_t *pid);
float pid_update(pid_ctrl_t *pid, float error, float error_rate, float dt);

/* Tilt-only controller (the step 6 PID). Returns clamped wheel acceleration. */
float balance_pid_accel(pid_ctrl_t *pid, float tilt, float tilt_rate, float dt, float max_accel);

/* Full state feedback (the step 22 LQR). k points to 4 gains. Returns clamped
 * wheel acceleration. */
float balance_lqr_accel(const float *k, float position, float speed, float tilt, float tilt_rate,
                        float max_accel);

/* Wheel acceleration -> motor command in -1..+1. PLACEHOLDER mapping: full command
 * equals max_accel. The real mapping comes from the bench test and the tuning
 * on the real robot. */
float accel_to_command(float accel, float max_accel);

/* 1 if the robot has tipped past the limit and the motors should be cut. */
int balance_should_cut(float tilt, float limit);

#endif