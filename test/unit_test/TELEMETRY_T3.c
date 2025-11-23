/**
 * @file    telemetry_ut3_status_overflow.c
 * @brief   Unit tests for telemetry_publish_status() buffer protection
 * @details Verifies that telemetry_publish_status() fails gracefully when
 *          the formatted JSON message would exceed the telemetry JSON
 *          buffer size, and that mqtt_publish() is not called in that case
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "telemetry.h"

typedef enum
{
    MQTT_DISCONNECTED = 0,
    MQTT_CONNECTING,
    MQTT_CONNECTED,
    MQTT_ERROR
} mqtt_status_t;

static int g_mqtt_publish_call_count = 0;
static char g_last_topic[128];
static char g_last_payload[600];
static int g_pass_count = 0;
static int g_fail_count = 0;

/**
 * @brief Reset the MQTT spy state before each test
 */
static void reset_mqtt_spy(void)
{
    g_mqtt_publish_call_count = 0;
    g_last_topic[0] = '\0';
    g_last_payload[0] = '\0';
}

/**
 * @brief Mock implementation of mqtt_publish() for unit testing
 * @param topic MQTT topic string
 * @param payload Message payload
 * @param qos Quality of Service level
 * @return Always true for testing
 */
bool mqtt_publish(const char *topic, const char *payload, uint8_t qos)
{
    (void)qos;
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

/**
 * @brief Stub for mqtt_get_status() used by telemetry_is_ready()
 * @return Always MQTT_CONNECTED for testing
 */
mqtt_status_t mqtt_get_status(void)
{
    return MQTT_CONNECTED;
}

/**
 * @brief Verify that overly long status messages are rejected safely
 * @return true if all checks pass, false otherwise
 */
static bool run_single_test(void)
{
    bool ok = false;
    bool all_ok = true;
    char big_msg[600];

    reset_mqtt_spy();

    memset(big_msg, 'X', sizeof(big_msg) - 1U);
    big_msg[sizeof(big_msg) - 1U] = '\0';

    ok = telemetry_publish_status(big_msg);

    if (ok)
    {
        all_ok = false;
    }

    if (g_mqtt_publish_call_count != 0)
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
    const char *test_name = "telemetry_ut3_status_overflow";

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
