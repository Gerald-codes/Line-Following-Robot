/** @file telemetry_ut2_encoder.c
 *
 * @brief Unit tests for telemetry_publish_encoder() JSON formatting.
 *
 * This module verifies that telemetry_publish_encoder() calls mqtt_publish()
 * exactly once and that the JSON payload contains the expected fields and
 * values.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "telemetry.h"   /* telemetry_publish_encoder() */

/* Local type definitions. */
typedef enum
{
    MQTT_DISCONNECTED = 0,
    MQTT_CONNECTING,
    MQTT_CONNECTED,
    MQTT_ERROR
} mqtt_status_t;

/* Local data. */
static char g_last_topic[128];
static char g_last_payload[600];
static int  g_mqtt_publish_call_count = 0;

static int g_pass_count = 0;
static int g_fail_count = 0;

/*!
 * @brief Reset the MQTT spy state before each test.
 */
static void
reset_mqtt_spy (void)
{
    g_mqtt_publish_call_count = 0;
    g_last_topic[0]           = '\0';
    g_last_payload[0]         = '\0';
}

/*!
 * @brief Mock implementation of mqtt_publish() for unit testing.
 *
 * This overrides the real mqtt_publish() when mqtt_client.c is not linked.
 */
bool
mqtt_publish (const char * topic, const char * payload, uint8_t qos)
{
    (void) qos;  /* Unused in this unit test. */

    g_mqtt_publish_call_count++;

    /* Store copies for inspection. */
    if (topic != NULL)
    {
        strncpy(g_last_topic, topic, sizeof(g_last_topic) - 1U);
        g_last_topic[sizeof(g_last_topic) - 1U] = '\0';
    }

    if (payload != NULL)
    {
        strncpy(g_last_payload, payload, sizeof(g_last_payload) - 1U);
        g_last_payload[sizeof(g_last_payload) - 1U] = '\0';
    }

    return true; /* Pretend publish always succeeds. */
}

/*!
 * @brief Stub for mqtt_get_status() used by telemetry_is_ready().
 */
mqtt_status_t
mqtt_get_status (void)
{
    return MQTT_CONNECTED;
}

/*!
 * @brief Verify basic behaviour of telemetry_publish_encoder().
 *
 * @return true if all checks pass, false otherwise.
 */
static bool
run_single_test (void)
{
    bool    all_ok       = true;
    bool    ok           = false;
    float   speed_mm_s   = 123.4f;
    float   distance_mm  = 56.7f;
    int32_t left_count   = 10;
    int32_t right_count  = -5;

    reset_mqtt_spy();

    ok = telemetry_publish_encoder(
             speed_mm_s,
             distance_mm,
             left_count,
             right_count);

    /* Should report success. */
    if (!ok)
    {
        all_ok = false;
    }

    /* Exactly one MQTT publish. */
    if (g_mqtt_publish_call_count != 1)
    {
        all_ok = false;
    }

    /*
     * Check that payload contains expected fields and values.
     * We do not care about spacing or field order, only the substrings.
     */
    if (strstr(g_last_payload, "\"speed\":123.4") == NULL)
    {
        all_ok = false;
    }

    if (strstr(g_last_payload, "\"distance\":56.7") == NULL)
    {
        all_ok = false;
    }

    if (strstr(g_last_payload, "\"L_enc\":10") == NULL)
    {
        all_ok = false;
    }

    if (strstr(g_last_payload, "\"R_enc\":-5") == NULL)
    {
        all_ok = false;
    }

    /* Topic check: verify it is non-empty without depending on exact name. */
    if (g_last_topic[0] == '\0')
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
    const char *test_name  = "telemetry_ut2_encoder";

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
