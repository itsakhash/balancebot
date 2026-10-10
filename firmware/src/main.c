#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "tilt.h"

/* Wiring (Nucleo-F446RE):
 *   MPU-6050 SCL -> PB8 (D15)     MPU-6050 SDA -> PB9 (D14)
 *   MPU-6050 VCC -> 3.3 V         MPU-6050 GND -> GND
 * Serial output goes to the ST-Link's USB virtual COM port (USART2, PA2/PA3).
 *
 * Tilt convention: angle = atan2(-ax, az), rotation about the sensor's Y axis.
 * Hold the board component-side up (az positive) so the angle is near 0.
 * Positive angle = the +X side of the board tipped downward. */

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5

#define LOOP_MS 5          /* 200 Hz filter rate */
#define CAL_SAMPLES 200    /* 200 samples * 5 ms = 1 s of gyro calibration */
#define PRINT_EVERY 20     /* print every 20th sample = 10 lines per second */
#define FILTER_ALPHA 0.98f /* time constant about 245 ms at 200 Hz */

static I2C_HandleTypeDef hi2c1;
static UART_HandleTypeDef huart2;

/* HAL_Delay() relies on this 1 ms tick; without it HAL_Delay() never returns. */
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
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3; /* PA2 = TX, PA3 = RX */
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
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9; /* PB8 = SCL, PB9 = SDA */
    gpio.Mode = GPIO_MODE_AF_OD;        /* I2C lines are open-drain */
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000; /* 100 kHz to start; 400 kHz later */
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

/* newlib-nano printf has no float support, so print degrees as text like +12.34. */
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

    print("\r\nbalancebot: tilt estimator\r\n");

    uint8_t who_am_i = 0;
    while (mpu6050_init(&hi2c1, &who_am_i) != HAL_OK)
    {
        print("MPU-6050 not found at 0x68. Check SCL/SDA/VCC/GND wiring.\r\n");
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(1000);
    }

    char line[96];
    snprintf(line, sizeof line, "WHO_AM_I = 0x%02X (a genuine MPU-6050 reports 0x68)\r\n", who_am_i);
    print(line);

    /* Gyro calibration: the board must be still. LED stays on while it runs. */
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

    char bias[16];
    format_deg(bias, sizeof bias, tilt.gyro_bias_dps);
    snprintf(line, sizeof line, "Gyro-Y bias = %s deg/s. Running.\r\n", bias);
    print(line);

    uint32_t next_tick = HAL_GetTick() + LOOP_MS;
    uint32_t loops = 0;
    while (1)
    {
        mpu6050_raw_t imu;
        if (mpu6050_read(&hi2c1, &imu) == HAL_OK)
        {
            float angle = tilt_update(&tilt, imu.ax, imu.az, imu.gy);
            loops++;
            if (loops % PRINT_EVERY == 0)
            {
                char filtered[16];
                char accel_only[16];
                format_deg(filtered, sizeof filtered, angle);
                format_deg(accel_only, sizeof accel_only, tilt_accel_angle_deg(imu.ax, imu.az));
                snprintf(line, sizeof line, "angle=%s accel=%s\r\n", filtered, accel_only);
                print(line);
            }
        }
        if (loops % 100 == 0)
        {
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN); /* slow blink = loop is running */
        }

        /* Wait for the next 5 ms slot. If a pass ran long, resync instead of racing. */
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