#include "stm32f4xx_hal.h"

/* On the Nucleo-64 boards, the green user LED (LD2) is on pin PA5. */
#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5

/* HAL_Delay() relies on this 1 ms tick; without it HAL_Delay() never returns. */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

int main(void)
{
    HAL_Init();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);

    while (1)
    {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(500);
    }
}