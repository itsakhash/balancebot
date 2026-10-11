#include "drivetrain.h"

#include "board_config.h"
#include "drive_math.h"
#include "stm32f4xx_hal.h"

static TIM_HandleTypeDef htim1; /* PWM for both motors */
static TIM_HandleTypeDef htim2; /* left encoder, 32-bit */
static TIM_HandleTypeDef htim3; /* right encoder, 16-bit */

static int32_t position[2];
static int32_t last_delta[2];
static uint32_t prev_raw[2];

static void gpio_output(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gpio);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static void gpio_af(GPIO_TypeDef *port, uint16_t pins, uint32_t af, uint32_t pull)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = pins;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = pull;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = af;
    HAL_GPIO_Init(port, &gpio);
}

static void pwm_init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    gpio_af(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, GPIO_AF1_TIM1, GPIO_NOPULL);

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = MOTOR_PWM_PERIOD - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim1);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

static void encoder_timer_init(TIM_HandleTypeDef *h, TIM_TypeDef *tim, uint32_t period)
{
    h->Instance = tim;
    h->Init.Prescaler = 0;
    h->Init.CounterMode = TIM_COUNTERMODE_UP;
    h->Init.Period = period;
    h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    TIM_Encoder_InitTypeDef enc = {0};
    enc.EncoderMode = TIM_ENCODERMODE_TI12; /* count on both edges of both channels (x4) */
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 6; /* ignore glitches shorter than a few input-clock cycles */
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 6;
    HAL_TIM_Encoder_Init(h, &enc);
    HAL_TIM_Encoder_Start(h, TIM_CHANNEL_ALL);
}

static void encoders_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    gpio_af(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_AF1_TIM2, GPIO_PULLUP);
    gpio_af(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_AF2_TIM3, GPIO_PULLUP);
    encoder_timer_init(&htim2, TIM2, 0xFFFFFFFFu);
    encoder_timer_init(&htim3, TIM3, 0xFFFFu);
    prev_raw[MOTOR_LEFT] = __HAL_TIM_GET_COUNTER(&htim2);
    prev_raw[MOTOR_RIGHT] = __HAL_TIM_GET_COUNTER(&htim3);
}

void drivetrain_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Standby first, so nothing can move while the rest is being set up. */
    gpio_output(STBY_PORT, STBY_PIN);
    gpio_output(LEFT_IN1_PORT, LEFT_IN1_PIN);
    gpio_output(LEFT_IN2_PORT, LEFT_IN2_PIN);
    gpio_output(RIGHT_IN1_PORT, RIGHT_IN1_PIN);
    gpio_output(RIGHT_IN2_PORT, RIGHT_IN2_PIN);

    pwm_init();
    encoders_init();
}

void motors_enable(int on)
{
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_set(motor_id_t id, float cmd)
{
    motor_drive_t d;
    if (id == MOTOR_LEFT)
    {
        motor_drive_from_command(cmd, MOTOR_LEFT_INVERT, MOTOR_PWM_PERIOD, MOTOR_DEADBAND, &d);
        HAL_GPIO_WritePin(LEFT_IN1_PORT, LEFT_IN1_PIN, d.in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_IN2_PORT, LEFT_IN2_PIN, d.in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d.duty);
    }
    else
    {
        motor_drive_from_command(cmd, MOTOR_RIGHT_INVERT, MOTOR_PWM_PERIOD, MOTOR_DEADBAND, &d);
        HAL_GPIO_WritePin(RIGHT_IN1_PORT, RIGHT_IN1_PIN, d.in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_IN2_PORT, RIGHT_IN2_PIN, d.in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d.duty);
    }
}

void encoders_update(void)
{
    uint32_t left = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t right = __HAL_TIM_GET_COUNTER(&htim3);

    int32_t dl = encoder_delta32(prev_raw[MOTOR_LEFT], left);
    int32_t dr = encoder_delta16((uint16_t)prev_raw[MOTOR_RIGHT], (uint16_t)right);
    prev_raw[MOTOR_LEFT] = left;
    prev_raw[MOTOR_RIGHT] = right;

    if (ENCODER_LEFT_INVERT)
        dl = -dl;
    if (ENCODER_RIGHT_INVERT)
        dr = -dr;

    last_delta[MOTOR_LEFT] = dl;
    last_delta[MOTOR_RIGHT] = dr;
    position[MOTOR_LEFT] += dl;
    position[MOTOR_RIGHT] += dr;
}

int32_t encoder_position(motor_id_t id)
{
    return position[id];
}

int32_t encoder_last_delta(motor_id_t id)
{
    return last_delta[id];
}