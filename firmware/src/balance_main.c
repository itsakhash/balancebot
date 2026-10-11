/* Balance loop. Build with:  pio run -e balance
 *
 * DRY_RUN = 1 (default): everything runs except the motors. The motor driver stays in
 *   standby. Tilt the board by hand and read what the robot WOULD command:
 *   hold it upright and still for 1 s -> state goes RUN; lean it forward -> cmd goes
 *   positive; tip it past 35 degrees -> state goes OFF.
 * DRY_RUN = 0: the motors are driven. Only do this on the stand, wheels off the ground,
 *   after the motor_bench test, with the bench supply current limit set.
 *
 * Wiring: the IMU wiring from main.c plus the motor and encoder wiring in board_config.h. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "board_config.h"
#include "balance.h"
#include "drivetrain.h"
#include "mpu6050.h"
#include "tilt.h"

#define DRY_RUN 1
#define USE_LQR 1 /* 1 = LQR, 0 = PID */

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5
#define LOOP_MS 5 /* 200 Hz */
#define CAL_SAMPLES 200
#define PRINT_EVERY 20 /* 10 lines per second */
#define FILTER_ALPHA 0.98f
#define MAX_IMU_FAILS 5 /* this many failed reads in a row disarms the robot */

static I2C_HandleTypeDef hi2c1;
static UART_HandleTypeDef huart2;

void SysTick_Handler(void)
{
    HAL_IncTick();
}

static void led_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);
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

static void i2c_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void print(const char *text)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

/* newlib-nano printf has no float support, so print values as text like +12.34. */
static void format_deg(char *out, size_t size, float deg)
{
    int32_t hundredths = (int32_t)(deg * 100.0f + (deg >= 0.0f ? 0.5f : -0.5f));
    char sign = '+';
    if (hundredths < 0)
    {
        sign = '-';
        hundredths = -hundredths;
    }
    snprintf(out, size, "%c%ld.%02ld", sign, (long)(hundredths / 100), (long)(hundredths % 100));
}

int main(void)
{
    HAL_Init();
    led_init();
    uart_init();
    i2c_init();
    drivetrain_init(); /* motors stay in standby until the loop enables them */

    print(DRY_RUN ? "\r\nbalancebot: balance loop (DRY RUN, motors off)\r\n"
                  : "\r\nbalancebot: balance loop (MOTORS LIVE)\r\n");

    uint8_t who_am_i = 0;
    while (mpu6050_init(&hi2c1, &who_am_i) != HAL_OK)
    {
        print("MPU-6050 not found at 0x68. Check SCL/SDA/VCC/GND wiring.\r\n");
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(1000);
    }

    tilt_t tilt;
    tilt_init(&tilt, FILTER_ALPHA, LOOP_MS / 1000.0f);
    print("Keep the board still. Calibrating gyro...\r\n");
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    for (int i = 0; i < CAL_SAMPLES; i++)
    {
        mpu6050_raw_t imu;
        if (mpu6050_read(&hi2c1, &imu) == HAL_OK)
        {
            tilt_calibrate_add(&tilt, imu.gy);
        }
        HAL_Delay(LOOP_MS);
    }
    tilt_calibrate_finish(&tilt);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

    balance_t bal;
    balance_init(&bal, USE_LQR);
    print("Hold the robot upright and still for 1 s to arm.\r\n");

    uint32_t next_tick = HAL_GetTick() + LOOP_MS;
    uint32_t loops = 0;
    int imu_fails = 0;
    float angle = 0.0f;
    float cmd = 0.0f;
    while (1)
    {
        mpu6050_raw_t imu;
        if (mpu6050_read(&hi2c1, &imu) == HAL_OK)
        {
            imu_fails = 0;
            angle = tilt_update(&tilt, imu.ax, imu.az, imu.gy);
            encoders_update();
            float wheel_counts = 0.5f * ((float)encoder_position(MOTOR_LEFT) + (float)encoder_position(MOTOR_RIGHT));

            bal_input_t in;
            in.tilt_deg = angle;
            in.rate_dps = gyro_dps(imu.gy, TILT_GYRO_LSB_PER_DPS, tilt.gyro_bias_dps);
            in.position_m = counts_to_meters(wheel_counts, ENCODER_COUNTS_PER_REV, BALANCE_WHEEL_DIAMETER_M);
            cmd = balance_step(&bal, &in, LOOP_MS / 1000.0f);
        }
        else
        {
            imu_fails++;
            if (imu_fails >= MAX_IMU_FAILS)
            {
                balance_force_disarm(&bal);
            }
            cmd = 0.0f;
        }

        int running = (bal.state == BAL_RUNNING);
        if (!DRY_RUN && running)
        {
            motors_enable(1);
            motor_set(MOTOR_LEFT, cmd);
            motor_set(MOTOR_RIGHT, cmd);
        }
        else
        {
            motor_set(MOTOR_LEFT, 0.0f);
            motor_set(MOTOR_RIGHT, 0.0f);
            motors_enable(0);
        }
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, running ? GPIO_PIN_SET : GPIO_PIN_RESET);

        loops++;
        if (loops % PRINT_EVERY == 0)
        {
            char a[16];
            char line[96];
            format_deg(a, sizeof a, angle);
            snprintf(line, sizeof line, "state=%s angle=%s cmd=%+d%% falls=%lu\r\n",
                     running ? "RUN" : "OFF", a, (int)(cmd * 100.0f), (unsigned long)bal.falls);
            print(line);
        }

        next_tick += LOOP_MS;
        uint32_t now = HAL_GetTick();
        if ((int32_t)(next_tick - now) < 0)
        {
            next_tick = now;
        }
        else
        {
            while ((int32_t)(next_tick - HAL_GetTick()) > 0)
            {
            }
        }
    }
}