#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* FreeRTOS settings for the balancebot (STM32F446RE, 16 MHz HSI).
 * The firmware is built with software floating point (the PlatformIO default for this board), so the
 * Cortex-M3 port is used: it is correct on a Cortex-M4 as long as no code touches the FPU.
 * Only used by the balance_rtos program. Each setting not listed here keeps the FreeRTOS default. */

#if defined(__VFP_FP__) && !defined(__SOFTFP__)
#error "Hard-float build: the Cortex-M3 port does not save the FPU registers. Use the ARM_CM4F port instead."
#endif

#include <stdint.h>
extern uint32_t SystemCoreClock;

/* Scheduler */
#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 0 /* equal-priority tasks only switch when they block */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ ((TickType_t)1000) /* 1 ms tick, the same rate the HAL tick uses */
#define configMAX_PRIORITIES 5
#define configMINIMAL_STACK_SIZE ((uint16_t)128) /* in 4-byte words */
#define configMAX_TASK_NAME_LEN 12
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1

/* Memory: static pool in heap_4.c, nothing is freed once the tasks exist */
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configSUPPORT_STATIC_ALLOCATION 0
#define configTOTAL_HEAP_SIZE ((size_t)(24 * 1024))

/* Features. Everything else is off to keep the build small. */
#define configUSE_MUTEXES 0
#define configUSE_RECURSIVE_MUTEXES 0
#define configUSE_COUNTING_SEMAPHORES 0
#define configUSE_QUEUE_SETS 0
#define configUSE_TIMERS 0
#define configUSE_TRACE_FACILITY 0
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_MALLOC_FAILED_HOOK 1
#define configCHECK_FOR_STACK_OVERFLOW 2 /* checks the stack fill pattern as well as the pointer */
#define configUSE_NEWLIB_REENTRANT 0     /* only the logger task calls printf */
#define configQUEUE_REGISTRY_SIZE 0
#define configGENERATE_RUN_TIME_STATS 0

/* Functions to include */
#define INCLUDE_vTaskDelay 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_vTaskDelete 0
#define INCLUDE_vTaskSuspend 0
#define INCLUDE_xTaskGetCurrentTaskHandle 0

/* Interrupt priorities. The F446 has 4 priority bits and the HAL sets 4 bits of preemption
 * priority. Interrupts numerically below the max-syscall priority (0 to 4) must never call
 * FreeRTOS functions; this program has no interrupt that does. The kernel itself runs at the lowest. */
#define configPRIO_BITS 4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Failed-assert handler: stop everything and let the watchdog reset the chip. */
void rtos_fail_safe(void);
#define configASSERT(x)       \
    do                        \
    {                         \
        if ((x) == 0)         \
        {                     \
            rtos_fail_safe(); \
        }                     \
    } while (0)

/* The port's handlers use the startup file's vector names. SysTick is not mapped here:
 * balance_rtos_main.c defines SysTick_Handler so it can keep HAL_Delay running. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler

#endif