/** @file telemetry_ut9_is_ready.c
 *
 * @brief Unit test for telemetry_is_ready() dependency on MQTT status.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "telemetry.h"   /* telemetry_init(), telemetry_is_ready() */

typedef enum
{
    MQTT_DISCONNECTED = 0,
    MQTT_CONNECTING,
    MQTT_CONNECTED,
    MQTT_ERROR
} mqtt_status_t;

static mqtt_status_t g_mock_status = MQTT_DISCONNECTED;
static bool          g_mock_connect_ok = true;

static int g_pass_count = 0;
static int g_fail_count = 0;

/* Minimal stubs so telemetry_init() links but does nothing real. */

void
mqtt_init (const char * broker_ip, uint16_t port, const char * client_id)
{
    (void) broker_ip;
    (void) port;
    (void) client_id;
}

bool
mqtt_connect (void)
{
    return g_mock_connect_ok;
}

mqtt_status_t
mqtt_get_status (void)
{
    return g_mock_status;
}

void
mqtt_process (void)
{
    /* Not needed for this unit test. */
}

/*!
 * @brief Run one test of telemetry_is_ready() logic.
 *
 * @return true if all checks pass, false otherwise.
 */
static bool
run_single_test (void)
{
    bool ok = true;
    bool ready = false;

    g_mock_connect_ok = true;
    g_mock_status = MQTT_CONNECTED;

    /* This will set telemetry internal flag to "initialized". */
    if (!telemetry_init("127.0.0.1", 1883U, "ut9_client"))
    {
        ok = false;
    }

    ready = telemetry_is_ready();
    if (!ready)
    {
        ok = false;
    }

    /* Now simulate MQTT disconnection. */
    g_mock_status = MQTT_DISCONNECTED;

    ready = telemetry_is_ready();
    if (ready)
    {
        ok = false;
    }

    return ok;
}

int
main (void)
{
    int i = 0;
    const int iterations = 50;

    printf("Running telemetry_ut9_is_ready...\n");

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
