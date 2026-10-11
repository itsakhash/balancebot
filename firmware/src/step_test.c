/* Motor step-response test. Build with:  pio run -e motor_step
 *
 * For each motor and each command in STEP_CMDS, the motor is stopped, then stepped
 * to that command while the encoder is read every 5 ms for 1 s. From each record the
 * program finds the steady speed, the dead time and the lag time constant, then fits
 * a line through speed versus command. It prints a summary and the numbers to use for
 * the motor model in the balance firmware.
 *
 * Read the printout from the serial monitor when it says DONE (about 20 s).
 * Press the black RESET button to run it again.
 *
 * SAFETY: wheels OFF THE GROUND (motors on the bench or the robot on its stand), bench
 * supply current limit at 1 A. The wheels have no load here, so the lag you measure is
 * the best case; a robot on the floor will be a little slower. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "balance.h"
#include "board_config.h"
#include "drive_math.h"
#include "drivetrain.h"
#include "step_fit.h"

#define STEP_DT_MS 5
#define STEP_SAMPLES 200 /* 1 s */
#define SETTLE_MS 800    /* coast to a stop before each step */
#define SMOOTH 4
#define RAW_CMD_INDEX 2 /* print the raw record for this command (0.40) */
#define MAX_CMD 0.60f   /* hard clamp for this test program */
#define NUM_CMDS 5
#define MIN_TAU_CMD 0.35f /* only steps at or above this command are averaged for tau */

static const float STEP_CMDS[NUM_CMDS] = {0.20f, 0.30f, 0.40f, 0.50f, 0.60f};

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
    HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 200);
}

/* newlib-nano printf has no float support, so show values as integers. */
static int to_tenths(float x)
{
    return (int)(x * 10.0f + (x >= 0.0f ? 0.5f : -0.5f));
}

static int to_thousandths(float x)
{
    return (int)(x * 1000.0f + (x >= 0.0f ? 0.5f : -0.5f));
}

static void print_tenths(const char *label, float x, const char *unit)
{
    int t = to_tenths(x);
    int a = t < 0 ? -t : t;
    char line[64];
    snprintf(line, sizeof line, " %s=%s%d.%d%s", label, t < 0 ? "-" : "", a / 10, a % 10, unit);
    print(line);
}

/* Step one motor from stopped to cmd and record the counts gained in every 5 ms. */
static void record_step(motor_id_t id, float cmd, int16_t *record)
{
    if (cmd > MAX_CMD)
        cmd = MAX_CMD;
    if (cmd < -MAX_CMD)
        cmd = -MAX_CMD;

    motor_set(MOTOR_LEFT, 0.0f);
    motor_set(MOTOR_RIGHT, 0.0f);
    HAL_Delay(SETTLE_MS);
    encoders_update(); /* baseline; this delta is thrown away */

    uint32_t t0 = HAL_GetTick();
    motor_set(id, cmd);
    for (int k = 0; k < STEP_SAMPLES; k++)
    {
        uint32_t target = t0 + (uint32_t)((k + 1) * STEP_DT_MS);
        while ((int32_t)(HAL_GetTick() - target) < 0)
        {
        }
        encoders_update();
        record[k] = (int16_t)encoder_last_delta(id);
    }
    motor_set(id, 0.0f);
}

static void print_raw(const int16_t *record)
{
    print("  raw counts per 5 ms, 20 per line:\r\n");
    char num[12];
    for (int k = 0; k < STEP_SAMPLES; k++)
    {
        snprintf(num, sizeof num, "%d ", (int)record[k]);
        print(num);
        if (k % 20 == 19)
            print("\r\n");
    }
}

static float cps_to_mps(float cps)
{
    return counts_to_meters(cps, ENCODER_COUNTS_PER_REV, BALANCE_WHEEL_DIAMETER_M);
}

static void test_motor(motor_id_t id, const char *name)
{
    static int16_t record[STEP_SAMPLES];
    float cmds[NUM_CMDS], speeds[NUM_CMDS];
    int used = 0;
    float tau_sum = 0.0f, delay_sum = 0.0f;
    int tau_count = 0;
    char line[96];

    snprintf(line, sizeof line, "== %s motor ==\r\n", name);
    print(line);

    for (int c = 0; c < NUM_CMDS; c++)
    {
        record_step(id, STEP_CMDS[c], record);

        step_fit_t fit;
        step_fit(record, STEP_SAMPLES, STEP_DT_MS / 1000.0f, SMOOTH, &fit);

        snprintf(line, sizeof line, "%s cmd=%d%%: steady=%d rpm", name, (int)(STEP_CMDS[c] * 100.0f + 0.5f),
                 (int)(fit.steady_cps / ENCODER_COUNTS_PER_REV * 60.0f));
        print(line);
        if (fit.ok)
        {
            print_tenths("delay", fit.delay_s * 1000.0f, "ms");
            print_tenths("tau", fit.tau_s * 1000.0f, "ms");
            print("\r\n");
            if (STEP_CMDS[c] >= MIN_TAU_CMD)
            {
                tau_sum += fit.tau_s;
                delay_sum += fit.delay_s;
                tau_count++;
            }
        }
        else
        {
            print("  NO FIT (motor did not move or never settled)\r\n");
        }
        if (fit.steady_cps >= 40.0f)
        {
            cmds[used] = STEP_CMDS[c];
            speeds[used] = fit.steady_cps;
            used++;
        }
        if (c == RAW_CMD_INDEX)
            print_raw(record);
    }

    float slope = 0.0f, intercept = 0.0f;
    if (fit_line(cmds, speeds, used, &slope, &intercept) && slope > 0.0f)
    {
        float gain_mps = cps_to_mps(slope);
        snprintf(line, sizeof line, "%s line: %d rpm per 100%% command", name,
                 (int)(slope / ENCODER_COUNTS_PER_REV * 60.0f));
        print(line);
        print_tenths(" starts moving at", (-intercept / slope) * 100.0f, "% command");
        print("\r\n");
        if (tau_count > 0)
        {
            float tau = tau_sum / (float)tau_count;
            float delay = delay_sum / (float)tau_count;
            snprintf(line, sizeof line, "%s MODEL: gain=%d.%03d m/s per 1.0 cmd", name,
                     to_thousandths(gain_mps) / 1000, to_thousandths(gain_mps) % 1000);
            print(line);
            print_tenths(" tau", tau * 1000.0f, "ms");
            print_tenths(" delay", delay * 1000.0f, "ms");
            print_tenths(" full-power accel", gain_mps / tau, "m/s^2 (no load)");
            print("\r\n");
        }
    }
    else
    {
        print(name);
        print(" line fit failed (fewer than two steps moved the motor)\r\n");
    }
}

int main(void)
{
    HAL_Init();
    uart_init();
    drivetrain_init();

    print("\r\nbalancebot: motor step response (wheels off the ground!)\r\n");
    HAL_Delay(2000);
    motors_enable(1);

    test_motor(MOTOR_LEFT, "L");
    test_motor(MOTOR_RIGHT, "R");

    motors_enable(0);
    print("DONE. Motors off. Press RESET to run again.\r\n");
    while (1)
    {
    }
}