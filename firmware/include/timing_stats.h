#ifndef TIMING_STATS_H
#define TIMING_STATS_H

#include <stdint.h>

/* Loop-timing statistics for one reporting window (for example 100 ms of a 200 Hz loop).
 * All times are in microseconds. Pure C with no hardware access, tested on the PC
 * (test/test_timing_stats.c). */
typedef struct
{
    uint32_t count;         /* number of loop passes recorded in this window          */
    uint32_t period_min_us; /* shortest time between two loop starts                  */
    uint32_t period_max_us; /* longest time between two loop starts                   */
    uint32_t period_sum_us; /* for the mean                                           */
    uint32_t exec_max_us;   /* longest time from loop start to the motor command      */
    uint32_t exec_sum_us;   /* for the mean                                           */
    uint32_t late;          /* passes whose period exceeded the late limit            */
} timing_window_t;

/* Empty the window. */
void timing_reset(timing_window_t *w);

/* Add one loop pass. period_us is the time since the previous pass started, exec_us is
 * how long this pass took, late_limit_us is the period above which a pass counts as late
 * (a skipped or delayed cycle). */
void timing_record(timing_window_t *w, uint32_t period_us, uint32_t exec_us, uint32_t late_limit_us);

/* Means over the window, rounded to the nearest microsecond; 0 for an empty window. */
uint32_t timing_period_mean_us(const timing_window_t *w);
uint32_t timing_exec_mean_us(const timing_window_t *w);

/* Convert a processor cycle count to microseconds, rounded to the nearest one. */
uint32_t cycles_to_us(uint32_t cycles, uint32_t clock_hz);

#endif