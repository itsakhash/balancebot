#include "timing_stats.h"

void timing_reset(timing_window_t *w)
{
    w->count = 0;
    w->period_min_us = 0xFFFFFFFFu;
    w->period_max_us = 0;
    w->period_sum_us = 0;
    w->exec_max_us = 0;
    w->exec_sum_us = 0;
    w->late = 0;
}

void timing_record(timing_window_t *w, uint32_t period_us, uint32_t exec_us, uint32_t late_limit_us)
{
    w->count++;
    if (period_us < w->period_min_us)
        w->period_min_us = period_us;
    if (period_us > w->period_max_us)
        w->period_max_us = period_us;
    w->period_sum_us += period_us;
    if (exec_us > w->exec_max_us)
        w->exec_max_us = exec_us;
    w->exec_sum_us += exec_us;
    if (period_us > late_limit_us)
        w->late++;
}

static uint32_t mean_of(uint32_t sum, uint32_t count)
{
    if (count == 0)
        return 0;
    return (uint32_t)(((uint64_t)sum + count / 2) / count);
}

uint32_t timing_period_mean_us(const timing_window_t *w)
{
    return mean_of(w->period_sum_us, w->count);
}

uint32_t timing_exec_mean_us(const timing_window_t *w)
{
    return mean_of(w->exec_sum_us, w->count);
}

uint32_t cycles_to_us(uint32_t cycles, uint32_t clock_hz)
{
    if (clock_hz == 0)
        return 0;
    return (uint32_t)(((uint64_t)cycles * 1000000u + clock_hz / 2) / clock_hz);
}