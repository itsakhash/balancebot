#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <stdint.h>

typedef enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} motor_id_t;

/* Sets up the TB6612 pins, TIM1 PWM, and the TIM2/TIM3 encoder counters.
 * Motors start disabled (STBY low) and coasting. */
void drivetrain_init(void);

/* 1 = driver on, 0 = driver in standby (both motors coast). */
void motors_enable(int on);

/* Command in -1..+1 (fraction of battery voltage). Positive = forward. */
void motor_set(motor_id_t id, float cmd);

/* Call at a steady rate (every loop pass). Reads both encoder timers and adds
 * the change since the last call to the running position. */
void encoders_update(void);

/* Total counts since init, and counts gained in the last encoders_update(). */
int32_t encoder_position(motor_id_t id);
int32_t encoder_last_delta(motor_id_t id);

#endif