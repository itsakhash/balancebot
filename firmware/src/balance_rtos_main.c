/* Balance loop on FreeRTOS. Build with:  pio run -e balance_rtos
 *
 * The same sensing, tilt filter, supervisor and controller as balance_main.c, but the 200 Hz
 * loop is a FreeRTOS task woken by the kernel tick instead of a busy-wait. Three tasks:
 *
 *   balance     priority 3   every 5 ms: read IMU, filter, controller, motors. Measures its own
 *                            period and run time with the CPU cycle counter (DWT).
 *   supervisor  priority 2   every 100 ms: checks the balance task is still running and only
 *                            then feeds the hardware watchdog. If it is not, the motors are cut
 *                            and the watchdog resets the chip.
 *   logger      priority 1   prints the angle 10 times a second and a timing line once a
 *                            second, so slow printing can never delay the control loop.
 *
 * DRY_RUN = 1 (default): motors off, exactly as in balance_main.c. Read the timing line to see
 * the real loop period and jitter. DRY_RUN = 0 drives the motors: same rules as balance_main.c
 * (stand, wheels off the ground, PID, current limit set).
 *
 * Wiring: identical to balance_main.c. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "board_config.h"
#include "balance.h"
#include "drivetrain.h"
#include "mpu6050.h"
#include "tilt.h"
#include "timing_stats.h"

#define DRY_RUN 1
#define USE_LQR 1 /* 1 = LQR, 0 = PID. Use 0 with the wheels in the air (see balance_main.c). */

/* Bring-up settings. Same meaning as in balance_main.c. */
#define BAL_TILT_SIGN 1.0f
#define BAL_TRIM_DEG 0.0f
#define BAL_CUT_DEG 35.0f

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5
#define LOOP_MS 5 /* 200 Hz */
#define CAL_SAMPLES 200
#define FILTER_ALPHA 0.98f
#define MAX_IMU_FAILS 5
#define ANGLE_EVERY 20     /* loop passes between angle lines = 0.1 s */
#define TIMING_EVERY 200   /* loop passes per timing window and timing line = 1 s */
#define LATE_LIMIT_US 5500 /* a pass that starts more than 0.5 ms after the previous one + 5 ms counts as late */
#define WATCHDOG_MS 500    /* nominal; the internal oscillator is only accurate to about +-40% */

#define PRIO_BALANCE 3
#define PRIO_SUPERVISOR 2
#define PRIO_LOGGER 1
#define STACK_BALANCE 384 /* words */
#define STACK_SUPERVISOR 128
#define STACK_LOGGER 512

typedef struct
{
    int full; /* 1 = also print the timing line */
    timing_window_t window;
    float angle_deg;
    float cmd;
    int running;
    uint32_t falls;
    uint32_t total_late;
    uint32_t total_missed;
    uint32_t total_imu_fails;
    uint32_t stack_free_balance;
    uint32_t stack_free_supervisor;
    uint32_t stack_free_logger;
} snapshot_t;

static I2C_HandleTypeDef hi2c1;
static UART_HandleTypeDef huart2;

static QueueHandle_t snapshot_queue;
static TaskHandle_t balance_handle;
static TaskHandle_t supervisor_handle;
static TaskHandle_t logger_handle;
static volatile uint32_t heartbeat; /* the balance task adds 1 every pass */

/* ---- fail-safe ---------------------------------------------------------------------------- */

/* Motors to standby, then stop. The watchdog resets the chip about half a second later. */
void rtos_fail_safe(void)
{
    __disable_irq();
    motors_enable(0);
    for (;;)
    {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    rtos_fail_safe();
}

void vApplicationMallocFailedHook(void)
{
    rtos_fail_safe();
}

void HardFault_Handler(void)
{
    rtos_fail_safe();
}

/* ---- tick: the kernel and the HAL share SysTick (both 1 kHz) ------------------------------- */

extern void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}

/* ---- hardware setup ----------------------------------------------------------------------- */

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

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Independent watchdog: LSI / 32 is about 1 kHz, so the reload value is about milliseconds. */
static void watchdog_start(void)
{
    IWDG->KR = 0xCCCC; /* start first: this also turns on the LSI clock the register updates need */
    IWDG->KR = 0x5555; /* unlock PR and RLR */
    IWDG->PR = 3;
    IWDG->RLR = WATCHDOG_MS;
    uint32_t guard = 0;
    while (IWDG->SR != 0 && guard++ < 1000000)
    {
    }
    IWDG->KR = 0xAAAA; /* load the reload value */
}

static void watchdog_feed(void)
{
    IWDG->KR = 0xAAAA;
}

/* ---- tasks -------------------------------------------------------------------------------- */

static void balance_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(LOOP_MS);
    TickType_t wake = xTaskGetTickCount();

    tilt_t tilt;
    tilt_init(&tilt, FILTER_ALPHA, LOOP_MS / 1000.0f);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    for (int i = 0; i < CAL_SAMPLES; i++)
    {
        mpu6050_raw_t imu;
        if (mpu6050_read(&hi2c1, &imu) == HAL_OK)
        {
            tilt_calibrate_add(&tilt, imu.gy);
        }
        heartbeat++;
        xTaskDelayUntil(&wake, period);
    }
    tilt_calibrate_finish(&tilt);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

    balance_t bal;
    balance_init(&bal, USE_LQR);
    bal.tilt_sign = BAL_TILT_SIGN;
    bal.trim_deg = BAL_TRIM_DEG;
    bal.cut_deg = BAL_CUT_DEG;

    timing_window_t window;
    timing_reset(&window);
    uint32_t prev_start = 0;
    int have_prev = 0;
    uint32_t passes = 0;
    uint32_t total_late = 0;
    uint32_t total_missed = 0;
    uint32_t total_imu_fails = 0;
    int imu_fails = 0;
    float angle = 0.0f;
    float cmd = 0.0f;

    for (;;)
    {
        BaseType_t on_time = xTaskDelayUntil(&wake, period);
        uint32_t start = DWT->CYCCNT;

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
            total_imu_fails++;
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
        uint32_t end = DWT->CYCCNT;
        heartbeat++;

        /* Timing is recorded after the motor command, so it does not add to the delay. */
        if (!on_time)
        {
            total_missed++;
        }
        if (have_prev)
        {
            uint32_t period_us = cycles_to_us(start - prev_start, SystemCoreClock);
            if (period_us > LATE_LIMIT_US)
            {
                total_late++;
            }
            timing_record(&window, period_us, cycles_to_us(end - start, SystemCoreClock), LATE_LIMIT_US);
        }
        prev_start = start;
        have_prev = 1;

        passes++;
        if (passes % ANGLE_EVERY == 0)
        {
            snapshot_t snap;
            snap.full = (passes % TIMING_EVERY == 0);
            snap.window = window;
            snap.angle_deg = angle;
            snap.cmd = cmd;
            snap.running = running;
            snap.falls = bal.falls;
            snap.total_late = total_late;
            snap.total_missed = total_missed;
            snap.total_imu_fails = total_imu_fails;
            snap.stack_free_balance = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
            snap.stack_free_supervisor = (uint32_t)uxTaskGetStackHighWaterMark(supervisor_handle);
            snap.stack_free_logger = (uint32_t)uxTaskGetStackHighWaterMark(logger_handle);
            xQueueOverwrite(snapshot_queue, &snap);
            if (snap.full)
            {
                timing_reset(&window);
            }
        }
    }
}

/* Feeds the watchdog only while the balance task keeps counting. */
static void supervisor_task(void *arg)
{
    (void)arg;
    uint32_t last = heartbeat;
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
        uint32_t now = heartbeat;
        if (now != last)
        {
            last = now;
            watchdog_feed();
        }
        else
        {
            motors_enable(0); /* control loop stalled: cut the motors; no feed, so the watchdog resets */
        }
    }
}

static void logger_task(void *arg)
{
    (void)arg;
    for (;;)
    {
        snapshot_t s;
        if (xQueueReceive(snapshot_queue, &s, pdMS_TO_TICKS(2000)) != pdPASS)
        {
            print("no data from the balance task for 2 s\r\n");
            continue;
        }
        char a[16];
        char line[200];
        format_deg(a, sizeof a, s.angle_deg);
        snprintf(line, sizeof line, "%s angle=%s cmd=%+d%%\r\n", s.running ? "RUN" : "OFF", a,
                 (int)(s.cmd * 100.0f));
        print(line);
        if (!s.full)
        {
            continue;
        }
        snprintf(line, sizeof line,
                 "  timing: period us min/mean/max=%lu/%lu/%lu exec us mean/max=%lu/%lu"
                 " | late=%lu missed=%lu imufail=%lu | stack free (words) bal/sup/log=%lu/%lu/%lu\r\n",
                 (unsigned long)s.window.period_min_us, (unsigned long)timing_period_mean_us(&s.window),
                 (unsigned long)s.window.period_max_us, (unsigned long)timing_exec_mean_us(&s.window),
                 (unsigned long)s.window.exec_max_us, (unsigned long)s.total_late,
                 (unsigned long)s.total_missed, (unsigned long)s.total_imu_fails,
                 (unsigned long)s.stack_free_balance, (unsigned long)s.stack_free_supervisor,
                 (unsigned long)uxTaskGetStackHighWaterMark(NULL));
        print(line);
    }
}

int main(void)
{
    HAL_Init();
    led_init();
    uart_init();
    i2c_init();
    drivetrain_init(); /* motors stay in standby until the balance task enables them */
    dwt_init();

    print(DRY_RUN ? "\r\nbalancebot: balance loop on FreeRTOS (DRY RUN, motors off)\r\n"
                  : "\r\nbalancebot: balance loop on FreeRTOS (MOTORS LIVE)\r\n");

    uint8_t who_am_i = 0;
    while (mpu6050_init(&hi2c1, &who_am_i) != HAL_OK)
    {
        print("MPU-6050 not found at 0x68. Check SCL/SDA/VCC/GND wiring.\r\n");
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(1000);
    }
    print("Keep the board still for 1 s (gyro calibration), then hold it upright and still for 1 s to arm.\r\n");

    snapshot_queue = xQueueCreate(1, sizeof(snapshot_t));
    if (snapshot_queue == NULL ||
        xTaskCreate(balance_task, "balance", STACK_BALANCE, NULL, PRIO_BALANCE, &balance_handle) != pdPASS ||
        xTaskCreate(supervisor_task, "supervisor", STACK_SUPERVISOR, NULL, PRIO_SUPERVISOR, &supervisor_handle) != pdPASS ||
        xTaskCreate(logger_task, "logger", STACK_LOGGER, NULL, PRIO_LOGGER, &logger_handle) != pdPASS)
    {
        rtos_fail_safe();
    }

    watchdog_start();
    vTaskStartScheduler();
    rtos_fail_safe(); /* only reached if the scheduler could not start */
    return 0;
}