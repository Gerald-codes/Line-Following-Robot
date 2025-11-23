/** @file telemetry_ut7_status_basic.c
 *
 * @brief Unit test for telemetry_publish_status() normal behaviour.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "telemetry.h"   /* telemetry_publish_status() */

/* Local MQTT status type for mocking. */
typedef enum
{
    MQTT_DISCONNECTED = 0,
    MQTT_CONNECTING,
    MQTT_CONNECTED,
    MQTT_ERROR
} mqtt_status_t;

/* MQTT spy data. */
static char g_last_topic[128];
static char g_last_payload[600];
static int  g_mqtt_publish_call_count = 0;
static mqtt_status_t g_mock_status = MQTT_CONNECTED;

static int g_pass_count = 0;
static int g_fail_count = 0;

/*!
 * @brief Reset MQTT spy state.
 */
static void
reset_mqtt_spy (void)
{
    g_last_topic[0] = '\0';
    g_last_payload[0] = '\0';
    g_mqtt_publish_call_count = 0;
}

/*!
 * @brief Mock of mqtt_publish() used by telemetry.c.
 */
bool
mqtt_publish (const char * topic, const char * payload, uint8_t qos)
{
    (void) qos;

    g_mqtt_publish_call_count++;

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

    return true;
}

/*!
 * @brief Mock of mqtt_get_status() used by telemetry_is_ready().
 */
mqtt_status_t
mqtt_get_status (void)
{
    return g_mock_status;
}

/*!
 * @brief Run one basic status publish test.
 *
 * @return true if all checks pass, false otherwise.
 */
static bool
run_single_test (void)
{
    bool ok = false;
    bool all_ok = true;

    reset_mqtt_spy();
    g_mock_status = MQTT_CONNECTED;

    ok = telemetry_publish_status("OK");

    if (!ok)
    {
        all_ok = false;
    }

    if (g_mqtt_publish_call_count != 1)
    {
        all_ok = false;
    }

    if (strstr(g_last_payload, "{\"status\":\"OK\"}") == NULL)
    {
        all_ok = false;
    }

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
    int i = 0;
    const int iterations = 100;

    printf("Running telemetry_ut7_status_basic...\n");

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
