/**
 * @file    telemetry_ut6_avoidance_dir.c
 * @brief   Unit test for telemetry_avoidance_dir_to_string()
 * @details Verifies correct string mappings for AvoidanceDirection enum values
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "telemetry.h"

static int g_pass_count = 0;
static int g_fail_count = 0;

/**
 * @brief Run a single test of avoidance-direction-to-string mappings
 * @return true if all checks pass, false otherwise
 */
static bool run_single_test(void)
{
    bool all_ok = true;
    const char *s = NULL;

    s = telemetry_avoidance_dir_to_string(AVOID_LEFT);
    if (strcmp(s, "LEFT") != 0)
    {
        all_ok = false;
    }

    s = telemetry_avoidance_dir_to_string(AVOID_RIGHT);
    if (strcmp(s, "RIGHT") != 0)
    {
        all_ok = false;
    }

    s = telemetry_avoidance_dir_to_string(AVOID_NONE);
    if (strcmp(s, "NONE") != 0)
    {
        all_ok = false;
    }

    s = telemetry_avoidance_dir_to_string((AvoidanceDirection)77);
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
    const char *test_name = "telemetry_ut6_avoidance_dir";

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
