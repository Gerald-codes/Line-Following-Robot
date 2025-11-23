/**
 * @file    telemetry_ut9_is_ready.c
 * @brief   Unit test for telemetry_is_ready() dependency on MQTT status
 * @details Verifies that telemetry_is_ready() correctly reflects
 *          initialization state and MQTT connection status
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

static mqtt_status_t g_mock_status = MQTT_DISCONNECTED;
static bool g_mock_connect_ok = true;
static int g_pass_count = 0;
static int g_fail_count = 0;

/**
 * @brief Stub for mqtt_init() so telemetry_init() links
 * @param broker_ip MQTT broker IP address
 * @param port MQTT broker port
 * @param client_id Client identifier
 */
void mqtt_init(const char *broker_ip, uint16_t port, const char *client_id)
{
    (void)broker_ip;
    (void)port;
    (void)client_id;
}

/**
 * @brief Stub for mqtt_connect() used by telemetry_init()
 * @return Mock connection result
 */
bool mqtt_connect(void)
{
    return g_mock_connect_ok;
}

/**
 * @brief Mock for mqtt_get_status() used by telemetry_is_ready()
 * @return Mock MQTT status
 */
mqtt_status_t mqtt_get_status(void)
{
    return g_mock_status;
}

/**
 * @brief Stub for mqtt_process() (not needed for this test)
 */
void mqtt_process(void)
{
}

/**
 * @brief Run one test of telemetry_is_ready() logic
 * @return true if all checks pass, false otherwise
 */
static bool run_single_test(void)
{
    bool ok = true;
    bool ready = false;

    g_mock_connect_ok = true;
    g_mock_status = MQTT_CONNECTED;

    if (!telemetry_init("127.0.0.1", 1883U, "ut9_client"))
    {
        ok = false;
    }

    ready = telemetry_is_ready();
    if (!ready)
    {
        ok = false;
    }

    g_mock_status = MQTT_DISCONNECTED;
    ready = telemetry_is_ready();
    if (ready)
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
    const int iterations = 50;
    const char *test_name = "telemetry_ut9_is_ready";

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
