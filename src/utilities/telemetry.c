#include "telemetry.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include <stdio.h>
#include <string.h>

// MQTT client instance
static mqtt_client_t* mqtt_client = NULL;
static bool is_connected = false;
static ip_addr_t broker_ip;

// WiFi credentials - Update these with your network details
#define WIFI_SSID "Javiersphone"
#define WIFI_PASSWORD "imcool123"

// MQTT Broker IP - Update with your broker's IP
#define MQTT_BROKER_IP "10.193.42.160"  // Change to your broker IP
#define MQTT_BROKER_PORT 1883

// Internal helper functions
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
static void mqtt_pub_request_cb(void *arg, err_t err);

/**
 * @brief MQTT connection callback
 */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[TELEMETRY] Connected to MQTT broker\n");
        is_connected = true;
    } else {
        printf("[TELEMETRY] Connection failed, status: %d\n", status);
        is_connected = false;
    }
}

/**
 * @brief MQTT publish callback
 */
static void mqtt_pub_request_cb(void *arg, err_t err) {
    if (err != ERR_OK) {
        printf("[TELEMETRY] Publish error: %d\n", err);
    }
}

/**
 * @brief Initialize WiFi connection
 */
static TelemetryStatus init_wifi(void) {
    if (cyw43_arch_init()) {
        printf("[TELEMETRY] Failed to initialize WiFi\n");
        return TELEMETRY_ERROR_CONNECTION;
    }

    cyw43_arch_enable_sta_mode();
    printf("[TELEMETRY] Connecting to WiFi: %s\n", WIFI_SSID);

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, 
                                           CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("[TELEMETRY] Failed to connect to WiFi\n");
        return TELEMETRY_ERROR_CONNECTION;
    }

    printf("[TELEMETRY] WiFi connected successfully\n");
    return TELEMETRY_SUCCESS;
}

TelemetryStatus telemetry_init(const char* broker_address, const char* client_id) {
    // Initialize WiFi
    TelemetryStatus status = init_wifi();
    if (status != TELEMETRY_SUCCESS) {
        return status;
    }

    // Create MQTT client
    mqtt_client = mqtt_client_new();
    if (mqtt_client == NULL) {
        printf("[TELEMETRY] Failed to create MQTT client\n");
        return TELEMETRY_ERROR_CONNECTION;
    }

    // Parse broker IP address
    if (!ip4addr_aton(MQTT_BROKER_IP, &broker_ip)) {
        printf("[TELEMETRY] Invalid broker IP address\n");
        return TELEMETRY_ERROR_CONNECTION;
    }

    // MQTT client info
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = client_id;
    ci.keep_alive = 60;

    // Connect to broker
    printf("[TELEMETRY] Connecting to MQTT broker at %s:%d\n", 
           MQTT_BROKER_IP, MQTT_BROKER_PORT);
    
    err_t err = mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT,
                                   mqtt_connection_cb, NULL, &ci);
    
    if (err != ERR_OK) {
        printf("[TELEMETRY] Failed to connect to broker: %d\n", err);
        return TELEMETRY_ERROR_CONNECTION;
    }

    // Wait for connection (with timeout)
    int timeout = 100; // 10 seconds
    while (!is_connected && timeout > 0) {
        sleep_ms(100);
        timeout--;
    }

    if (!is_connected) {
        printf("[TELEMETRY] Connection timeout\n");
        return TELEMETRY_ERROR_CONNECTION;
    }

    printf("[TELEMETRY] Telemetry system initialized successfully\n");
    return TELEMETRY_SUCCESS;
}

TelemetryStatus telemetry_publish_obstacle_width(int obstacle_id, float width, float smoothed_width) {
    if (!is_connected) {
        return TELEMETRY_ERROR_NOT_CONNECTED;
    }

    char payload[128];
    snprintf(payload, sizeof(payload), 
             "{\"id\":%d,\"width\":%.2f,\"smoothed_width\":%.2f}", 
             obstacle_id, width, smoothed_width);

    err_t err = mqtt_publish(mqtt_client, TOPIC_OBSTACLE_WIDTH, payload, 
                            strlen(payload), MQTT_QOS, 0, 
                            mqtt_pub_request_cb, NULL);

    if (err != ERR_OK) {
        printf("[TELEMETRY] Failed to publish width: %d\n", err);
        return TELEMETRY_ERROR_PUBLISH;
    }

    printf("[TELEMETRY] Published: %s -> %s\n", TOPIC_OBSTACLE_WIDTH, payload);
    return TELEMETRY_SUCCESS;
}

TelemetryStatus telemetry_publish_obstacle_count(int count) {
    if (!is_connected) {
        return TELEMETRY_ERROR_NOT_CONNECTED;
    }

    char payload[64];
    snprintf(payload, sizeof(payload), "{\"count\":%d}", count);

    err_t err = mqtt_publish(mqtt_client, TOPIC_OBSTACLE_COUNT, payload, 
                            strlen(payload), MQTT_QOS, 0, 
                            mqtt_pub_request_cb, NULL);

    if (err != ERR_OK) {
        printf("[TELEMETRY] Failed to publish count: %d\n", err);
        return TELEMETRY_ERROR_PUBLISH;
    }

    printf("[TELEMETRY] Published: %s -> %s\n", TOPIC_OBSTACLE_COUNT, payload);
    return TELEMETRY_SUCCESS;
}

TelemetryStatus telemetry_publish_obstacle_distance(int obstacle_id, uint64_t distance) {
    if (!is_connected) {
        return TELEMETRY_ERROR_NOT_CONNECTED;
    }

    char payload[128];
    snprintf(payload, sizeof(payload), 
             "{\"id\":%d,\"distance\":%llu}", 
             obstacle_id, distance);

    err_t err = mqtt_publish(mqtt_client, TOPIC_OBSTACLE_DISTANCE, payload, 
                            strlen(payload), MQTT_QOS, 0, 
                            mqtt_pub_request_cb, NULL);

    if (err != ERR_OK) {
        printf("[TELEMETRY] Failed to publish distance: %d\n", err);
        return TELEMETRY_ERROR_PUBLISH;
    }

    printf("[TELEMETRY] Published: %s -> %s\n", TOPIC_OBSTACLE_DISTANCE, payload);
    return TELEMETRY_SUCCESS;
}

TelemetryStatus telemetry_publish_obstacle_angles(int obstacle_id, int angle_start, 
                                                   int angle_end, int angle_span) {
    if (!is_connected) {
        return TELEMETRY_ERROR_NOT_CONNECTED;
    }

    char payload[256];
    snprintf(payload, sizeof(payload), 
             "{\"id\":%d,\"angle_start\":%d,\"angle_end\":%d,\"angle_span\":%d}", 
             obstacle_id, angle_start, angle_end, angle_span);

    err_t err = mqtt_publish(mqtt_client, TOPIC_OBSTACLE_ANGLES, payload, 
                            strlen(payload), MQTT_QOS, 0, 
                            mqtt_pub_request_cb, NULL);

    if (err != ERR_OK) {
        printf("[TELEMETRY] Failed to publish angles: %d\n", err);
        return TELEMETRY_ERROR_PUBLISH;
    }

    printf("[TELEMETRY] Published: %s -> %s\n", TOPIC_OBSTACLE_ANGLES, payload);
    return TELEMETRY_SUCCESS;
}

TelemetryStatus telemetry_publish_obstacle(const ObstacleTelemetry* obstacle) {
    if (!is_connected) {
        return TELEMETRY_ERROR_NOT_CONNECTED;
    }

    if (obstacle == NULL) {
        return TELEMETRY_ERROR_INVALID_PARAM;
    }

    char payload[512];
    snprintf(payload, sizeof(payload), 
             "{\"id\":%d,\"angle_start\":%d,\"angle_end\":%d,\"angle_span\":%d,"
             "\"min_distance\":%llu,\"width\":%.2f,\"smoothed_width\":%.2f}",
             obstacle->obstacle_id, obstacle->angle_start, obstacle->angle_end,
             obstacle->angle_span, obstacle->min_distance, 
             obstacle->width, obstacle->smoothed_width);

    err_t err = mqtt_publish(mqtt_client, TOPIC_OBSTACLE_ALL, payload, 
                            strlen(payload), MQTT_QOS, 0, 
                            mqtt_pub_request_cb, NULL);

    if (err != ERR_OK) {
        printf("[TELEMETRY] Failed to publish obstacle: %d\n", err);
        return TELEMETRY_ERROR_PUBLISH;
    }

    printf("[TELEMETRY] Published: %s -> %s\n", TOPIC_OBSTACLE_ALL, payload);
    return TELEMETRY_SUCCESS;
}

TelemetryStatus telemetry_publish_scan_results(const ScanTelemetry* scan_data) {
    if (!is_connected) {
        return TELEMETRY_ERROR_NOT_CONNECTED;
    }

    if (scan_data == NULL) {
        return TELEMETRY_ERROR_INVALID_PARAM;
    }

    // First publish the obstacle count
    telemetry_publish_obstacle_count(scan_data->obstacle_count);

    // Then publish each obstacle individually
    for (int i = 0; i < scan_data->obstacle_count; i++) {
        telemetry_publish_obstacle(&scan_data->obstacles[i]);
        sleep_ms(10); // Small delay between publishes
    }

    // Publish scan complete message
    char payload[256];
    snprintf(payload, sizeof(payload), 
             "{\"obstacle_count\":%d,\"timestamp\":%llu,\"status\":\"complete\"}",
             scan_data->obstacle_count, scan_data->scan_timestamp);

    err_t err = mqtt_publish(mqtt_client, TOPIC_SCAN_COMPLETE, payload, 
                            strlen(payload), MQTT_QOS, 0, 
                            mqtt_pub_request_cb, NULL);

    if (err != ERR_OK) {
        printf("[TELEMETRY] Failed to publish scan complete: %d\n", err);
        return TELEMETRY_ERROR_PUBLISH;
    }

    printf("[TELEMETRY] Scan results published successfully\n");
    return TELEMETRY_SUCCESS;
}

TelemetryStatus telemetry_publish_status(const char* message) {
    if (!is_connected) {
        return TELEMETRY_ERROR_NOT_CONNECTED;
    }

    if (message == NULL) {
        return TELEMETRY_ERROR_INVALID_PARAM;
    }

    char payload[512];
    snprintf(payload, sizeof(payload), "{\"status\":\"%s\"}", message);

    err_t err = mqtt_publish(mqtt_client, TOPIC_STATUS, payload, 
                            strlen(payload), MQTT_QOS, 0, 
                            mqtt_pub_request_cb, NULL);

    if (err != ERR_OK) {
        printf("[TELEMETRY] Failed to publish status: %d\n", err);
        return TELEMETRY_ERROR_PUBLISH;
    }

    printf("[TELEMETRY] Published: %s -> %s\n", TOPIC_STATUS, payload);
    return TELEMETRY_SUCCESS;
}

bool telemetry_is_connected(void) {
    return is_connected && mqtt_client_is_connected(mqtt_client);
}

TelemetryStatus telemetry_process(void) {
    // Process any pending network events
    // This should be called periodically in the main loop
    if (is_connected) {
        cyw43_arch_poll();
    }
    return TELEMETRY_SUCCESS;
}

void telemetry_cleanup(void) {
    if (mqtt_client != NULL) {
        if (is_connected) {
            mqtt_disconnect(mqtt_client);
        }
        mqtt_client_free(mqtt_client);
        mqtt_client = NULL;
    }
    
    cyw43_arch_deinit();
    is_connected = false;
    
    printf("[TELEMETRY] Telemetry system cleaned up\n");
}