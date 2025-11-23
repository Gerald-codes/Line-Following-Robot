/** @file telemetry_ut1_line_state.c
 *
 * @brief Unit tests for telemetry_line_state_to_string().
 *
 * This module verifies that the telemetry_line_state_to_string() helper
 * returns the expected strings for each known LineFollowState value and
 * "UNKNOWN" for out-of-range values.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "telemetry.h"   /* LineFollowState, telemetry_line_state_to_string() */

/* Local counters for success rate. */
static int g_pass_count = 0;
static int g_fail_count = 0;

/*!
 * @brief Run a single test of line-follow-state-to-string mappings.
 *
 * @return true if all checks pass, false otherwise.
 */
static bool
run_single_test (void)
{
    bool         all_ok = true;
    const char * s      = NULL;

    /* These enum names must exist in telemetry.h. */
    s = telemetry_line_state_to_string(LINE_FOLLOW_CENTERED);
    if (strcmp(s, "CENTERED") != 0)
    {
        all_ok = false;
    }

    s = telemetry_line_state_to_string(LINE_FOLLOW_LEFT);
    if (strcmp(s, "LEFT") != 0)
    {
        all_ok = false;
    }

    s = telemetry_line_state_to_string(LINE_FOLLOW_RIGHT);
    if (strcmp(s, "RIGHT") != 0)
    {
        all_ok = false;
    }

    s = telemetry_line_state_to_string(LINE_FOLLOW_FAR_LEFT);
    if (strcmp(s, "FAR_LEFT") != 0)
    {
        all_ok = false;
    }

    s = telemetry_line_state_to_string(LINE_FOLLOW_FAR_RIGHT);
    if (strcmp(s, "FAR_RIGHT") != 0)
    {
        all_ok = false;
    }

    s = telemetry_line_state_to_string(LINE_FOLLOW_LOST);
    if (strcmp(s, "LOST") != 0)
    {
        all_ok = false;
    }

    /* Out-of-range value must map to "UNKNOWN". */
    s = telemetry_line_state_to_string((LineFollowState) 99);
    if (strcmp(s, "UNKNOWN") != 0)
    {
        all_ok = false;
    }

    return all_ok;
}

/*!
 * @brief Entry point for this unit test executable.
 */
int
main (void)
{
    int         i          = 0;
    const int   iterations = 100;
    const char *test_name  = "telemetry_ut1_line_state";

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
            printf("  Iteration %d failed.\n", i);
        }
    }

    printf("Result (%s): %d passed, %d failed (iterations=%d)\n",
           test_name,
           g_pass_count,
           g_fail_count,
           iterations);

    return 0;
}

/*** end of file ***/
