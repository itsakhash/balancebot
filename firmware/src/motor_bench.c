/* Motor + encoder bench test. Build with:  pio run -e motor_bench
 *
 * Phase 1 (15 s, motors OFF): turn each wheel by hand exactly one revolution and
 *   read the counts. Expect about 937 per wheel revolution. Forward (the way the
 *   wheel turns when the robot drives forward) should count UP.
 * Phase 2: runs each motor at a gentle command, forward then reverse, then both.
 *   Prints cmd, total counts and RPM for each motor every 100 ms.
 *
 * SAFETY: first run with the wheels OFF THE GROUND (robot on a stand or the motors
 * on the bench) and the bench supply current limit set to 1 A. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "board_config.h"
#include "drive_math.h"
#include "drivetrain.h"

#define BENCH_CMD 0.30f     /* fraction of supply voltage; raise slowly */
#define BENCH_MAX_CMD 0.60f /* hard clamp for this test program */
#define SAMPLE_MS 100
#define STEP_MS 2000 /* each move lasts 2 s, each stop 1 s */
#define STOP_MS 1000

static UART_HandleTypeDef huart2;

void SysTick_Handler(void)
{
    HAL_IncTick();
}

static void uart_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

static void print(const char *text)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

/* Print one status line: command, running count, and RPM from the last interval. */
static void print_sample(float cmd_l, float cmd_r)
{
    int32_t dl = encoder_last_delta(MOTOR_LEFT);
    int32_t dr = encoder_last_delta(MOTOR_RIGHT);
    float dt = SAMPLE_MS / 1000.0f;
    int rpm_l = (int)speed_rpm(dl, ENCODER_COUNTS_PER_REV, dt);
    int rpm_r = (int)speed_rpm(dr, ENCODER_COUNTS_PER_REV, dt);
    char line[128];
    snprintf(line, sizeof line, "L cmd=%+d%% cnt=%ld rpm=%d | R cmd=%+d%% cnt=%ld rpm=%d\r\n",
             (int)(cmd_l * 100.0f), (long)encoder_position(MOTOR_LEFT), rpm_l,
             (int)(cmd_r * 100.0f), (long)encoder_position(MOTOR_RIGHT), rpm_r);
    print(line);
}

static void run_for(uint32_t ms, float cmd_l, float cmd_r)
{
    if (cmd_l > BENCH_MAX_CMD)
        cmd_l = BENCH_MAX_CMD;
    if (cmd_l < -BENCH_MAX_CMD)
        cmd_l = -BENCH_MAX_CMD;
    if (cmd_r > BENCH_MAX_CMD)
        cmd_r = BENCH_MAX_CMD;
    if (cmd_r < -BENCH_MAX_CMD)
        cmd_r = -BENCH_MAX_CMD;
    motor_set(MOTOR_LEFT, cmd_l);
    motor_set(MOTOR_RIGHT, cmd_r);

    uint32_t end = HAL_GetTick() + ms;
    uint32_t next = HAL_GetTick() + SAMPLE_MS;
    while ((int32_t)(end - HAL_GetTick()) > 0)
    {
        /* encoders_update() must run more often than the counters can wrap:
         * TIM3 is 16-bit, so polling every 100 ms is fine at these speeds. */
        if ((int32_t)(HAL_GetTick() - next) >= 0)
        {
            next += SAMPLE_MS;
            encoders_update();
            print_sample(cmd_l, cmd_r);
        }
    }
}

int main(void)
{
    HAL_Init();
    uart_init();
    drivetrain_init();

    print("\r\nbalancebot: motor + encoder bench test\r\n");
    char line[96];
    snprintf(line, sizeof line, "counts per wheel rev (expected) = %d\r\n", (int)ENCODER_COUNTS_PER_REV);
    print(line);

    print("Phase 1: motors OFF. Turn each wheel forward exactly 1 revolution by hand.\r\n");
    for (int i = 0; i < 150; i++) /* 15 s */
    {
        HAL_Delay(SAMPLE_MS);
        encoders_update();
        print_sample(0.0f, 0.0f);
    }

    print("Phase 2: motors ON (wheels must be off the ground).\r\n");
    motors_enable(1);
    while (1)
    {
        print("-- left forward\r\n");
        run_for(STEP_MS, BENCH_CMD, 0.0f);
        run_for(STOP_MS, 0.0f, 0.0f);
        print("-- left reverse\r\n");
        run_for(STEP_MS, -BENCH_CMD, 0.0f);
        run_for(STOP_MS, 0.0f, 0.0f);
        print("-- right forward\r\n");
        run_for(STEP_MS, 0.0f, BENCH_CMD);
        run_for(STOP_MS, 0.0f, 0.0f);
        print("-- right reverse\r\n");
        run_for(STEP_MS, 0.0f, -BENCH_CMD);
        run_for(STOP_MS, 0.0f, 0.0f);
        print("-- both forward\r\n");
        run_for(STEP_MS, BENCH_CMD, BENCH_CMD);
        run_for(STOP_MS, 0.0f, 0.0f);
        print("-- both reverse\r\n");
        run_for(STEP_MS, -BENCH_CMD, -BENCH_CMD);
        run_for(STOP_MS, 0.0f, 0.0f);
    }
}