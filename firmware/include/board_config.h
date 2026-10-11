#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* ---- TB6612FNG motor driver wiring (Nucleo-F446RE) -------------------------
 * Driver pin  -> Nucleo pin  (Arduino label)
 *   PWMA      -> PA8   (D7)   TIM1_CH1, left motor speed
 *   AIN1      -> PB4   (D5)
 *   AIN2      -> PB5   (D4)
 *   PWMB      -> PA9   (D8)   TIM1_CH2, right motor speed
 *   BIN1      -> PB10  (D6)
 *   BIN2      -> PC7   (D9)
 *   STBY      -> PB6   (D10)  low = motors off
 *   VCC       -> 3V3          logic supply
 *   VM        -> battery +    motor supply (7.4 V pack, after the fuse and switch)
 *   GND       -> GND          shared with the Nucleo and the battery
 *
 * Encoders (JGA25-370): blue = 3V3, black = GND (power the encoder from 3V3!)
 *   left  yellow/green -> PA0 (A0) / PA1 (A1)   TIM2 encoder mode
 *   right yellow/green -> PA6 (D12) / PA7 (D11) TIM3 encoder mode
 * ------------------------------------------------------------------------- */

#define LEFT_IN1_PORT GPIOB
#define LEFT_IN1_PIN GPIO_PIN_4
#define LEFT_IN2_PORT GPIOB
#define LEFT_IN2_PIN GPIO_PIN_5
#define RIGHT_IN1_PORT GPIOB
#define RIGHT_IN1_PIN GPIO_PIN_10
#define RIGHT_IN2_PORT GPIOC
#define RIGHT_IN2_PIN GPIO_PIN_7
#define STBY_PORT GPIOB
#define STBY_PIN GPIO_PIN_6

/* PWM: the timer runs at 16 MHz (HSI), so a period of 800 ticks gives 20 kHz,
 * above hearing range. A command of 1.0 corresponds to a duty of 800. */
#define MOTOR_PWM_PERIOD 800

/* Flip these if the bench test shows a wheel turning (or counting) backward.
 * The two motors are mounted as mirror images, so one side usually needs it. */
#define MOTOR_LEFT_INVERT 0
#define MOTOR_RIGHT_INVERT 1
#define ENCODER_LEFT_INVERT 0
#define ENCODER_RIGHT_INVERT 1

/* Smallest command (as a fraction of full power) that moves the wheel at all.
 * 0 for now; the bench test tells us the real value. */
#define MOTOR_DEADBAND 0.0f

/* Encoder geometry: 11 pulses per motor-shaft turn per channel, counted on both
 * edges of both channels (x4), then the gearbox ratio. The 280 RPM motor is 21.3:1.
 * Verify by turning a wheel exactly once by hand and reading the count. */
#define ENCODER_PPR 11.0f
#define GEAR_RATIO 21.3f
#define ENCODER_COUNTS_PER_REV (ENCODER_PPR * 4.0f * GEAR_RATIO)

#endif