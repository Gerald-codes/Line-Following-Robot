/**
 * @file    telemetry_ut10_barcode_basic.c
 * @brief   Unit test for telemetry_publish_barcode() basic JSON structure
 * @details Verifies that telemetry_publish_barcode() correctly formats
 *          barcode command and character data for MQTT publishing
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

static char g_last_topic[128];
static char g_last_payload[600];
static int g_mqtt_publish_call_count = 0;
static mqtt_status_t g_mock_status = MQTT_CONNECTED;
static int g_pass_count = 0;
static int g_fail_count = 0;

/**
 * @brief Reset MQTT spy state
 */
static void reset_mqtt_spy(void)
{
    g_last_topic[0] = '\0';
    g_last_payload[0] = '\0';
    g_mqtt_publish_call_count = 0;
}

/**
 * @brief Mock of mqtt_publish() used by telemetry.c
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
 * @brief Mock of mqtt_get_status() used by telemetry_is_ready()
 * @return Current mock MQTT status
 */
mqtt_status_t mqtt_get_status(void)
{
    return g_mock_status;
}

/**
 * @brief Run one basic barcode publish test
 * @return true if all checks pass, false otherwise
 */
static bool run_single_test(void)
{
    bool ok = true;
    bool rc = false;

    reset_mqtt_spy();
    g_mock_status = MQTT_CONNECTED;

    rc = telemetry_publish_barcode((BarcodeCommand)0, 'A');

    if (!rc)
    {
        ok = false;
    }

    if (g_mqtt_publish_call_count != 1)
    {
        ok = false;
    }

    if (strstr(g_last_payload, "\"command\"") == NULL)
    {
        ok = false;
    }

    if (strstr(g_last_payload, "\"char\":\"A\"") == NULL)
    {
        ok = false;
    }

    if (g_last_topic[0] == '\0')
    {
        ok = false;
    }

    return ok;
}

/**
 * @brief Entry point for this unit test executable
 */
int main(void)
{
    int i = 0;
    const int iterations = 100;
    const char *test_name = "telemetry_ut10_barcode_basic";

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
