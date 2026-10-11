#ifndef DRIVE_MATH_H
#define DRIVE_MATH_H

#include <stdint.h>

/* What to write to the TB6612 for one motor. in1/in2 are pin levels (0 or 1),
 * duty is the PWM compare value (0..period). in1 = in2 = 0 means coast. */
typedef struct
{
    uint8_t in1;
    uint8_t in2;
    uint16_t duty;
} motor_drive_t;

/* Turn a command in -1..+1 into pin levels and a duty.
 *  - commands outside -1..+1 are clamped
 *  - invert flips the direction
 *  - deadband (0..0.9) is the fraction of full power a tiny nonzero command starts at;
 *    the rest of the range is scaled linearly up to full power
 *  - a command smaller than 0.001 in magnitude means coast */
void motor_drive_from_command(float cmd, int invert, uint16_t period, float deadband,
                              motor_drive_t *out);

/* Signed number of counts between two readings of a free-running counter,
 * correct across wraparound. Use the 16-bit version for TIM3, 32-bit for TIM2. */
int32_t encoder_delta16(uint16_t previous, uint16_t current);
int32_t encoder_delta32(uint32_t previous, uint32_t current);

float counts_to_revs(int32_t counts, float counts_per_rev);

/* Wheel speed in RPM from a count change over dt seconds (0 if dt <= 0). */
float speed_rpm(int32_t delta_counts, float counts_per_rev, float dt_seconds);

#endif