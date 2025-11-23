/**
 * @file    telemetry_ut5_obstacle_state.c
 * @brief   Unit test for telemetry_obstacle_state_to_string()
 * @details Verifies correct string mappings for all ObstacleState enum values
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "telemetry.h"

static int g_pass_count = 0;
static int g_fail_count = 0;

/**
 * @brief Run a single test of obstacle-state-to-string mappings
 * @return true if all checks pass, false otherwise
 */
static bool run_single_test(void)
{
    bool all_ok = true;
    const char *s = NULL;

    s = telemetry_obstacle_state_to_string(OBSTACLE_STATE_NONE);
    if (strcmp(s, "NONE") != 0)
    {
        all_ok = false;
    }

    s = telemetry_obstacle_state_to_string(OBSTACLE_STATE_DETECTED);
    if (strcmp(s, "DETECTED") != 0)
    {
        all_ok = false;
    }

    s = telemetry_obstacle_state_to_string(OBSTACLE_STATE_SCANNING);
    if (strcmp(s, "SCANNING") != 0)
    {
        all_ok = false;
    }

    s = telemetry_obstacle_state_to_string(OBSTACLE_STATE_AVOIDING);
    if (strcmp(s, "AVOIDING") != 0)
    {
        all_ok = false;
    }

    s = telemetry_obstacle_state_to_string(OBSTACLE_STATE_RETURNING);
    if (strcmp(s, "RETURNING") != 0)
    {
        all_ok = false;
    }

    s = telemetry_obstacle_state_to_string((ObstacleState)123);
    if (strcmp(s, "UNKNOWN") != 0)
    {
        all_ok = false;
    }

    return all_ok;
}

/**
 * @brief Entry point for this unit test executable
 */
int main(void)
{
    int i = 0;
    const int iterations = 100;
    const char *test_name = "telemetry_ut5_obstacle_state";

    printf("Running %s...\n", test_name);

    for (i = 0; i < iterations; i++)
    {
        if (run_single_test())
        {
            g_pass_count++;
        }
        else
        {
            g_fail_count++;
            printf(" Iteration %d failed.\n", i);
        }
    }

    printf("Result (%s): %d passed, %d failed (iterations=%d)\n",
           test_name,
           g_pass_count,
           g_fail_count,
           iterations);

    return 0;
}
