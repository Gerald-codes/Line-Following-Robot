/** @file telemetry_ut4_robot_state.c
 *
 * @brief Unit test for telemetry_robot_state_to_string().
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "telemetry.h"   /* RobotState, telemetry_robot_state_to_string() */

static int g_pass_count = 0;
static int g_fail_count = 0;

/*!
 * @brief Run a single test of robot-state-to-string mappings.
 *
 * @return true if all checks pass, false otherwise.
 */
static bool
run_single_test (void)
{
    bool all_ok = true;
    const char * s = NULL;

    s = telemetry_robot_state_to_string(ROBOT_STATE_IDLE);
    if (strcmp(s, "IDLE") != 0)
    {
        all_ok = false;
    }

    s = telemetry_robot_state_to_string(ROBOT_STATE_FOLLOWING);
    if (strcmp(s, "FOLLOWING") != 0)
    {
        all_ok = false;
    }

    s = telemetry_robot_state_to_string(ROBOT_STATE_TURNING);
    if (strcmp(s, "TURNING") != 0)
    {
        all_ok = false;
    }

    s = telemetry_robot_state_to_string(ROBOT_STATE_STOPPED);
    if (strcmp(s, "STOPPED") != 0)
    {
        all_ok = false;
    }

    s = telemetry_robot_state_to_string(ROBOT_STATE_SCANNING);
    if (strcmp(s, "SCANNING") != 0)
    {
        all_ok = false;
    }

    s = telemetry_robot_state_to_string(ROBOT_STATE_AVOIDING);
    if (strcmp(s, "AVOIDING") != 0)
    {
        all_ok = false;
    }

    s = telemetry_robot_state_to_string(ROBOT_STATE_RETURNING);
    if (strcmp(s, "RETURNING") != 0)
    {
        all_ok = false;
    }

    s = telemetry_robot_state_to_string(ROBOT_STATE_LOST);
    if (strcmp(s, "LOST") != 0)
    {
        all_ok = false;
    }

    s = telemetry_robot_state_to_string(ROBOT_STATE_ERROR);
    if (strcmp(s, "ERROR") != 0)
    {
        all_ok = false;
    }

    /* Out-of-range value must map to "UNKNOWN". */
    s = telemetry_robot_state_to_string((RobotState) 99);
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
    int i = 0;
    const int iterations = 100;

    printf("Running telemetry_ut4_robot_state...\n");

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

    printf("Result: %d passed, %d failed (iterations=%d)\n",
           g_pass_count, g_fail_count, iterations);

    return 0;
}

/*** end of file ***/
