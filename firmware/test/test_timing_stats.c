/* Host-side test for the loop-timing statistics. Build and run on your PC (from firmware/):
 *   gcc -Wall -Wextra -Iinclude test/test_timing_stats.c src/timing_stats.c -o /tmp/test_timing_stats && /tmp/test_timing_stats
 */
#include <stdint.h>
#include <stdio.h>

#include "timing_stats.h"

static int failures = 0;

#define CHECK(cond, msg)               \
    do                                 \
    {                                  \
        if (!(cond))                   \
        {                              \
            printf("FAIL: %s\n", msg); \
            failures++;                \
        }                              \
        else                           \
        {                              \
            printf("ok:   %s\n", msg); \
        }                              \
    } while (0)

#define LATE 7500u

static void test_empty_and_reset(void)
{
    timing_window_t w;
    timing_reset(&w);
    CHECK(w.count == 0, "empty window has count 0");
    CHECK(timing_period_mean_us(&w) == 0, "empty window period mean is 0, no division by zero");
    CHECK(timing_exec_mean_us(&w) == 0, "empty window exec mean is 0, no division by zero");
    CHECK(w.period_max_us == 0 && w.exec_max_us == 0 && w.late == 0, "empty window maxima and late count are 0");

    timing_record(&w, 5000, 1500, LATE);
    timing_reset(&w);
    CHECK(w.count == 0 && w.late == 0 && w.period_sum_us == 0, "reset clears a used window");
    timing_record(&w, 5100, 900, LATE);
    CHECK(w.period_min_us == 5100, "after reset the minimum starts fresh");
}

static void test_statistics(void)
{
    timing_window_t w;
    timing_reset(&w);
    timing_record(&w, 5000, 1500, LATE);
    timing_record(&w, 4990, 1600, LATE);
    timing_record(&w, 5010, 1400, LATE);

    CHECK(w.count == 3, "count is 3");
    CHECK(w.period_min_us == 4990, "period minimum");
    CHECK(w.period_max_us == 5010, "period maximum");
    CHECK(timing_period_mean_us(&w) == 5000, "period mean is 5000");
    CHECK(w.exec_max_us == 1600, "exec maximum");
    CHECK(timing_exec_mean_us(&w) == 1500, "exec mean is 1500");
    CHECK(w.late == 0, "no late passes");
}

static void test_late(void)
{
    timing_window_t w;
    timing_reset(&w);
    timing_record(&w, 5000, 1500, LATE);
    timing_record(&w, 7500, 1500, LATE);
    CHECK(w.late == 0, "a period exactly at the limit is not late");
    timing_record(&w, 7501, 1500, LATE);
    timing_record(&w, 10000, 6000, LATE);
    CHECK(w.late == 2, "two periods over the limit are counted late");
    CHECK(w.period_max_us == 10000, "late period shows in the maximum");
}

static void test_rounding_and_range(void)
{
    timing_window_t w;
    timing_reset(&w);
    timing_record(&w, 5000, 1000, LATE);
    timing_record(&w, 5001, 1001, LATE);
    CHECK(timing_period_mean_us(&w) == 5001, "mean 5000.5 rounds up to 5001");
    CHECK(timing_exec_mean_us(&w) == 1001, "mean 1000.5 rounds up to 1001");

    timing_reset(&w);
    for (int i = 0; i < 1000; i++)
        timing_record(&w, 5000, 1500, LATE);
    CHECK(timing_period_mean_us(&w) == 5000, "one thousand passes keep the exact mean");
}

static void test_cycles_to_us(void)
{
    CHECK(cycles_to_us(16, 16000000u) == 1, "16 cycles at 16 MHz is 1 us");
    CHECK(cycles_to_us(80000, 16000000u) == 5000, "80000 cycles at 16 MHz is 5000 us");
    CHECK(cycles_to_us(180, 180000000u) == 1, "180 cycles at 180 MHz is 1 us");
    CHECK(cycles_to_us(8, 16000000u) == 1, "half a microsecond rounds up to 1");
    CHECK(cycles_to_us(7, 16000000u) == 0, "under half a microsecond rounds down to 0");
    CHECK(cycles_to_us(4000000000u, 16000000u) == 250000000u, "large counts do not overflow");
    CHECK(cycles_to_us(100, 0) == 0, "zero clock gives 0, no division by zero");
}

int main(void)
{
    test_empty_and_reset();
    test_statistics();
    test_late();
    test_rounding_and_range();
    test_cycles_to_us();

    if (failures == 0)
        printf("\nALL TESTS PASSED (0 failures)\n");
    else
        printf("\n%d FAILURE(S)\n", failures);
    return failures == 0 ? 0 : 1;
}