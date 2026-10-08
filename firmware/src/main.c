#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "mpu6050.h"

/* Wiring (Nucleo-F446RE):
 *   MPU-6050 SCL -> PB8 (D15)     MPU-6050 SDA -> PB9 (D14)
 *   MPU-6050 VCC -> 3.3 V         MPU-6050 GND -> GND
 * Serial output goes to the ST-Link's USB virtual COM port (USART2, PA2/PA3). */

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5

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

int main(void)
{
    HAL_Init();
    led_init();
    uart_init();
    i2c_init();

    print("\r\nbalancebot: MPU-6050 test\r\n");

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

    while (1)
    {
        mpu6050_raw_t imu;
        if (mpu6050_read(&hi2c1, &imu) == HAL_OK)
        {
            snprintf(line, sizeof line,
                     "ax=%6d ay=%6d az=%6d | gx=%6d gy=%6d gz=%6d\r\n",
                     imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz);
            print(line);
        }
        else
        {
            print("read failed\r\n");
        }
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(100);
    }
}