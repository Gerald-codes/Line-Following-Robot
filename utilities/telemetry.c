// utilities/telemetry.c
#include "telemetry.h"

#include <stdio.h>
#include <string.h>

// Only include WiFi headers if building for Pico W

#include "pico/cyw43_arch.h"          // Pico W Wi-Fi
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"

// ===== Tunables =====
#ifndef TELEMETRY_KEEPALIVE_SEC
#define TELEMETRY_KEEPALIVE_SEC 30
#endif
#ifndef TELEMETRY_QOS
#define TELEMETRY_QOS 0
#endif
#ifndef TELEMETRY_RETAIN
#define TELEMETRY_RETAIN 0
#endif

static mqtt_client_t *g_client = NULL;
static volatile bool g_mqtt_connected = false;
static volatile bool g_wifi_ready = false;

// ---------- MQTT callbacks ----------
static void mqtt_conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void)client; (void)arg;
    g_mqtt_connected = (status == MQTT_CONNECT_ACCEPTED);
    if (g_mqtt_connected) {
        printf("[MQTT] Connected (status=%d)\n", status);
    } else {
        printf("[MQTT] Disconnected (status=%d)\n", status);
    }
}

static void mqtt_pub_cb(void *arg, err_t result) {
    (void)arg;
    if (result != ERR_OK) {
        printf("[MQTT] Publish result=%d\n", result);
    }
}

// ---------- Internal helpers ----------
static inline bool client_ready(void) {
    return g_client && g_mqtt_connected && mqtt_client_is_connected(g_client);
}

bool telemetry_is_connected(void) {
    return client_ready();
}

static inline TelemetryStatus publish_json(const char *topic, const char *json) {
    if (!client_ready()) return TELEMETRY_ERROR_NOT_CONNECTED;
    err_t err = mqtt_publish(g_client, topic, json, (u16_t)strlen(json),
                             TELEMETRY_QOS, TELEMETRY_RETAIN, mqtt_pub_cb, NULL);
    return (err == ERR_OK) ? TELEMETRY_SUCCESS : TELEMETRY_ERROR_PUBLISH;
}

static TelemetryStatus mqtt_connect_now(void) {
    if (!g_client) {
        g_client = mqtt_client_new();
        if (!g_client) {
            printf("[MQTT] Out of memory creating client\n");
            return TELEMETRY_ERROR_PUBLISH;
        }
    }

    ip_addr_t ip;
    if (!ipaddr_aton(MQTT_BROKER, &ip)) {
        // If you need DNS: replace this with dns_gethostbyname()
        printf("[MQTT] Invalid broker IP: %s\n", MQTT_BROKER);
        return TELEMETRY_ERROR_INVALID_PARAM;
    }

    struct mqtt_connect_client_info_t ci = {0};
    ci.client_id  = MQTT_CLIENTID;
    ci.keep_alive = TELEMETRY_KEEPALIVE_SEC;

    err_t err = mqtt_client_connect(g_client, &ip, MQTT_PORT, mqtt_conn_cb, NULL, &ci);
    if (err != ERR_OK) {
        printf("[MQTT] mqtt_client_connect err=%d\n", err);
        return TELEMETRY_ERROR_PUBLISH;
    }
    return TELEMETRY_SUCCESS; // async; mqtt_conn_cb will flip g_mqtt_connected
}

// ---------- Public API ----------
TelemetryStatus telemetry_begin(void) {
    if (cyw43_arch_init()) {
        printf("[WIFI] cyw43_arch_init failed\n");
        return TELEMETRY_ERROR_PUBLISH;
    }
    cyw43_arch_enable_sta_mode();

    printf("[WIFI] Connecting to SSID \"%s\" ...\n", WIFI_SSID);
    int rc = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 15000);
    if (rc) {
        printf("[WIFI] Connect failed rc=%d\n", rc);
        // We still return SUCCESS so the app can run; publishes will simply no-op.
        // Call telemetry_begin() again later if you want a retry.
        g_wifi_ready = false;
    } else {
        printf("[WIFI] Connected\n");
        g_wifi_ready = true;
    }

    if (!g_wifi_ready) return TELEMETRY_ERROR_NOT_CONNECTED;

    TelemetryStatus st = mqtt_connect_now();
    if (st != TELEMETRY_SUCCESS) {
        printf("[MQTT] Initial connect failed (%d); will keep running\n", st);
        return st;
    }
    return TELEMETRY_SUCCESS;
}

TelemetryStatus telemetry_publish_heading(float heading_raw_deg, float heading_ema_deg) {
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"raw_deg\":%.2f,\"ema_deg\":%.2f}",
             heading_raw_deg, heading_ema_deg);
    return publish_json(TOPIC_HEADING, payload);
}

TelemetryStatus telemetry_publish_speed(float left_speed_mm_s, float right_speed_mm_s) {
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"left_mm_s\":%.2f,\"right_mm_s\":%.2f}",
             left_speed_mm_s, right_speed_mm_s);
    return publish_json(TOPIC_SPEED, payload);
}

TelemetryStatus telemetry_publish_distance(float left_distance_mm, float right_distance_mm) {
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"left_mm\":%.2f,\"right_mm\":%.2f}",
             left_distance_mm, right_distance_mm);
    return publish_json(TOPIC_DISTANCE, payload);
}

