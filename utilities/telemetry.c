/**
 * telemetry.c
 * MQTT-Based Telemetry System with WiFi Support
 * 
 * This implementation provides both MQTT telemetry and serial fallback.
 * For Demo 1, it publishes real-time speed, distance, and heading data.
 * 
 * NOTE: This is a TEMPLATE implementation. You'll need to:
 * 1. Add appropriate Pico W WiFi libraries (pico_cyw43_arch, pico_lwip)
 * 2. Add MQTT library (e.g., paho-mqtt-embedded-c or lwip MQTT)
 * 3. Update CMakeLists.txt with required libraries
 * 
 * The structure is designed to work with the Pico W's WiFi capabilities.
 */

#include "telemetry.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// For Pico W WiFi - uncomment when you have the libraries
// #include "pico/cyw43_arch.h"
// #include "lwip/apps/mqtt.h"
// #include "lwip/dns.h"

// ============================================================================
// INTERNAL STATE
// ============================================================================

typedef struct {
    ConnectionStatus status;
    bool telemetry_enabled;
    bool wifi_connected;
    bool mqtt_connected;
    
    uint32_t last_publish_time;
    uint32_t last_reconnect_attempt;
    uint32_t start_time;
    
    uint32_t packets_sent;
    uint32_t packets_failed;
    float last_latency_ms;
    
    // MQTT client handle (will be set when using actual MQTT library)
    void *mqtt_client;
    
    // Command callback
    void (*command_callback)(const char *topic, const char *payload);
    
} TelemetryState;

static TelemetryState state = {
    .status = CONN_STATUS_DISCONNECTED,
    .telemetry_enabled = false,
    .wifi_connected = false,
    .mqtt_connected = false,
    .last_publish_time = 0,
    .last_reconnect_attempt = 0,
    .start_time = 0,
    .packets_sent = 0,
    .packets_failed = 0,
    .last_latency_ms = 0.0f,
    .mqtt_client = NULL,
    .command_callback = NULL
};

// ============================================================================
// FORWARD DECLARATIONS - MQTT CALLBACKS
// ============================================================================

// These will be implemented when you add the MQTT library
static void mqtt_connection_callback(void *arg, int status);
static void mqtt_publish_callback(void *arg, int err);
static void mqtt_incoming_data_callback(void *arg, const char *topic, 
                                       uint32_t topic_len, const void *data, 
                                       uint32_t data_len);

// ============================================================================
// INITIALIZATION
// ============================================================================

void telemetry_init(void) {
    state.start_time = to_ms_since_boot(get_absolute_time());
    state.last_publish_time = state.start_time;
    state.telemetry_enabled = true;
    
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║           MQTT TELEMETRY SYSTEM INITIALIZING                  ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    printf("Configuration:\n");
    printf("  Broker: %s:%d\n", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    printf("  Client ID: %s\n", MQTT_CLIENT_ID);
    printf("  Publish Interval: %d ms\n", TELEMETRY_PUBLISH_INTERVAL_MS);
    printf("  Serial Fallback: %s\n", TELEMETRY_ENABLE_SERIAL_FALLBACK ? "Enabled" : "Disabled");
    printf("\n");
    
    // Initialize WiFi (Pico W specific - uncomment when ready)
    /*
    if (cyw43_arch_init()) {
        printf("✗ WiFi initialization failed!\n");
        state.status = CONN_STATUS_ERROR;
        return;
    }
    printf("✓ WiFi hardware initialized\n");
    */
    
    printf("✓ Telemetry system ready (WiFi/MQTT not yet connected)\n");
    printf("  Call telemetry_connect_wifi() to connect\n\n");
}

// ============================================================================
// WIFI CONNECTION
// ============================================================================

bool telemetry_connect_wifi(const char *ssid, const char *password) {
    printf("Connecting to WiFi: %s\n", ssid);
    state.status = CONN_STATUS_WIFI_CONNECTING;
    
    /* Pico W WiFi connection - uncomment when libraries are added
    cyw43_arch_enable_sta_mode();
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    int result = cyw43_arch_wifi_connect_timeout_ms(ssid, password, 
                                                     CYW43_AUTH_WPA2_AES_PSK, 
                                                     WIFI_CONNECT_TIMEOUT_MS);
    
    if (result != 0) {
        printf("✗ WiFi connection failed (error %d)\n", result);
        state.status = CONN_STATUS_ERROR;
        state.wifi_connected = false;
        return false;
    }
    
    uint32_t connect_time = to_ms_since_boot(get_absolute_time()) - start_time;
    printf("✓ WiFi connected in %lu ms\n", connect_time);
    
    // Print IP address
    extern cyw43_t cyw43_state;
    uint32_t ip = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
    printf("  IP Address: %lu.%lu.%lu.%lu\n", 
           ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, ip >> 24);
    */
    
    // Simulation for testing without WiFi hardware
    printf("✓ WiFi connected (simulated)\n");
    state.wifi_connected = true;
    state.status = CONN_STATUS_WIFI_CONNECTED;
    
    return true;
}

// ============================================================================
// MQTT CONNECTION
// ============================================================================

bool telemetry_connect_mqtt(void) {
    if (!state.wifi_connected) {
        printf("✗ Cannot connect MQTT: WiFi not connected\n");
        return false;
    }
    
    printf("Connecting to MQTT broker: %s:%d\n", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    state.status = CONN_STATUS_MQTT_CONNECTING;
    
    /* MQTT connection - uncomment when MQTT library is added
    mqtt_client_t *client = mqtt_client_new();
    if (client == NULL) {
        printf("✗ Failed to create MQTT client\n");
        state.status = CONN_STATUS_ERROR;
        return false;
    }
    
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = MQTT_CLIENT_ID;
    ci.client_user = MQTT_USERNAME[0] ? MQTT_USERNAME : NULL;
    ci.client_pass = MQTT_PASSWORD[0] ? MQTT_PASSWORD : NULL;
    ci.keep_alive = MQTT_KEEPALIVE_SEC;
    
    // Connect to broker
    ip_addr_t broker_ip;
    err_t err = dns_gethostbyname(MQTT_BROKER_HOST, &broker_ip, NULL, NULL);
    if (err != ERR_OK) {
        printf("✗ DNS lookup failed for %s\n", MQTT_BROKER_HOST);
        state.status = CONN_STATUS_ERROR;
        return false;
    }
    
    err = mqtt_client_connect(client, &broker_ip, MQTT_BROKER_PORT, 
                             mqtt_connection_callback, NULL, &ci);
    if (err != ERR_OK) {
        printf("✗ MQTT connection failed (error %d)\n", err);
        state.status = CONN_STATUS_ERROR;
        return false;
    }
    
    state.mqtt_client = client;
    */
    
    // Simulation for testing without MQTT library
    printf("✓ MQTT connected (simulated)\n");
    state.mqtt_connected = true;
    state.status = CONN_STATUS_MQTT_CONNECTED;
    
    return true;
}

// ============================================================================
// MQTT CALLBACKS (for real implementation)
// ============================================================================

static void mqtt_connection_callback(void *arg, int status) {
    if (status == 0) {
        printf("✓ MQTT connection established\n");
        state.mqtt_connected = true;
        state.status = CONN_STATUS_MQTT_CONNECTED;
        
        // Subscribe to command topic
        telemetry_subscribe(TOPIC_COMMAND, QOS_COMMAND);
    } else {
        printf("✗ MQTT connection failed: %d\n", status);
        state.mqtt_connected = false;
        state.status = CONN_STATUS_ERROR;
    }
}

static void mqtt_publish_callback(void *arg, int err) {
    if (err == 0) {
        state.packets_sent++;
    } else {
        state.packets_failed++;
        if (TELEMETRY_ENABLE_SERIAL_FALLBACK) {
            printf("⚠ MQTT publish failed (error %d)\n", err);
        }
    }
}

static void mqtt_incoming_data_callback(void *arg, const char *topic, 
                                       uint32_t topic_len, const void *data, 
                                       uint32_t data_len) {
    if (state.command_callback != NULL) {
        // Create null-terminated strings
        char topic_str[64];
        char payload_str[128];
        
        uint32_t copy_len = topic_len < 63 ? topic_len : 63;
        memcpy(topic_str, topic, copy_len);
        topic_str[copy_len] = '\0';
        
        copy_len = data_len < 127 ? data_len : 127;
        memcpy(payload_str, data, copy_len);
        payload_str[copy_len] = '\0';
        
        state.command_callback(topic_str, payload_str);
    }
}

// ============================================================================
// DISCONNECT AND RECONNECT
// ============================================================================

void telemetry_disconnect(void) {
    printf("Disconnecting telemetry...\n");
    
    /* Uncomment when using real MQTT
    if (state.mqtt_client != NULL) {
        mqtt_disconnect(state.mqtt_client);
        state.mqtt_client = NULL;
    }
    */
    
    state.mqtt_connected = false;
    state.wifi_connected = false;
    state.status = CONN_STATUS_DISCONNECTED;
    
    printf("✓ Telemetry disconnected\n");
}

bool telemetry_reconnect(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Rate limit reconnection attempts
    if (current_time - state.last_reconnect_attempt < MQTT_RECONNECT_DELAY_MS) {
        return false;
    }
    
    state.last_reconnect_attempt = current_time;
    printf("Attempting reconnection...\n");
    
    if (!state.wifi_connected) {
        if (!telemetry_connect_wifi(WIFI_SSID, WIFI_PASSWORD)) {
            return false;
        }
    }
    
    if (!state.mqtt_connected) {
        if (!telemetry_connect_mqtt()) {
            return false;
        }
    }
    
    return true;
}

// ============================================================================
// PUBLISHING FUNCTIONS - DEMO 1 SPECIFIC
// ============================================================================

void telemetry_publish_speed(float left_speed, float right_speed) {
    if (!state.telemetry_enabled) return;
    
    float avg_speed = (left_speed + right_speed) / 2.0f;
    char buffer[64];
    
    // Publish to individual topics
    snprintf(buffer, sizeof(buffer), "%.1f", left_speed);
    telemetry_publish(TOPIC_SPEED_LEFT, buffer, QOS_TELEMETRY);
    
    snprintf(buffer, sizeof(buffer), "%.1f", right_speed);
    telemetry_publish(TOPIC_SPEED_RIGHT, buffer, QOS_TELEMETRY);
    
    snprintf(buffer, sizeof(buffer), "%.1f", avg_speed);
    telemetry_publish(TOPIC_SPEED_AVG, buffer, QOS_TELEMETRY);
    
    // Serial fallback
    if (TELEMETRY_ENABLE_SERIAL_FALLBACK || !state.mqtt_connected) {
        printf("[SPEED] L: %6.1f mm/s | R: %6.1f mm/s | Avg: %6.1f mm/s\n", 
               left_speed, right_speed, avg_speed);
    }
}

void telemetry_publish_heading(float filtered_heading, float raw_heading) {
    if (!state.telemetry_enabled) return;
    
    char buffer[64];
    
    snprintf(buffer, sizeof(buffer), "%.2f", filtered_heading);
    telemetry_publish(TOPIC_HEADING_FILTERED, buffer, QOS_TELEMETRY);
    
    snprintf(buffer, sizeof(buffer), "%.2f", raw_heading);
    telemetry_publish(TOPIC_HEADING_RAW, buffer, QOS_TELEMETRY);
    
    // Serial fallback
    if (TELEMETRY_ENABLE_SERIAL_FALLBACK || !state.mqtt_connected) {
        printf("[HEADING] Filtered: %6.1f° | Raw: %6.1f°\n", 
               filtered_heading, raw_heading);
    }
}

void telemetry_publish_distance(float left_distance, float right_distance) {
    if (!state.telemetry_enabled) return;
    
    float total_distance = (left_distance + right_distance) / 2.0f;
    char buffer[64];
    
    snprintf(buffer, sizeof(buffer), "%.1f", left_distance);
    telemetry_publish(TOPIC_DISTANCE_LEFT, buffer, QOS_TELEMETRY);
    
    snprintf(buffer, sizeof(buffer), "%.1f", right_distance);
    telemetry_publish(TOPIC_DISTANCE_RIGHT, buffer, QOS_TELEMETRY);
    
    snprintf(buffer, sizeof(buffer), "%.1f", total_distance);
    telemetry_publish(TOPIC_DISTANCE_TOTAL, buffer, QOS_TELEMETRY);
    
    // Serial fallback
    if (TELEMETRY_ENABLE_SERIAL_FALLBACK || !state.mqtt_connected) {
        printf("[DISTANCE] L: %7.1f mm | R: %7.1f mm | Total: %7.1f mm\n", 
               left_distance, right_distance, total_distance);
    }
}

void telemetry_publish_imu_data(float heading, float gyro_z, 
                                float accel_x, float accel_y) {
    if (!state.telemetry_enabled) return;
    
    char buffer[64];
    
    snprintf(buffer, sizeof(buffer), "%.2f", gyro_z);
    telemetry_publish(TOPIC_IMU_GYRO_Z, buffer, QOS_TELEMETRY);
    
    snprintf(buffer, sizeof(buffer), "%.3f", accel_x);
    telemetry_publish(TOPIC_IMU_ACCEL_X, buffer, QOS_TELEMETRY);
    
    snprintf(buffer, sizeof(buffer), "%.3f", accel_y);
    telemetry_publish(TOPIC_IMU_ACCEL_Y, buffer, QOS_TELEMETRY);
    
    // Serial fallback
    if (TELEMETRY_ENABLE_SERIAL_FALLBACK || !state.mqtt_connected) {
        printf("[IMU] Heading: %6.1f° | Gyro Z: %+6.1f°/s | Accel X: %+5.2fg Y: %+5.2fg\n",
               heading, gyro_z, accel_x, accel_y);
    }
}

void telemetry_publish_motor_output(float left_output, float right_output) {
    if (!state.telemetry_enabled) return;
    
    char buffer[64];
    
    snprintf(buffer, sizeof(buffer), "%.1f", left_output);
    telemetry_publish(TOPIC_MOTOR_LEFT_PWM, buffer, QOS_TELEMETRY);
    
    snprintf(buffer, sizeof(buffer), "%.1f", right_output);
    telemetry_publish(TOPIC_MOTOR_RIGHT_PWM, buffer, QOS_TELEMETRY);
    
    // Serial fallback
    if (TELEMETRY_ENABLE_SERIAL_FALLBACK || !state.mqtt_connected) {
        printf("[MOTOR PWM] L: %+6.1f%% | R: %+6.1f%%\n", left_output, right_output);
    }
}

void telemetry_publish_state(const char *state_name) {
    if (!state.telemetry_enabled) return;
    
    telemetry_publish(TOPIC_STATUS, state_name, QOS_STATUS);
    
    if (TELEMETRY_ENABLE_SERIAL_FALLBACK || !state.mqtt_connected) {
        printf("[STATE] %s\n", state_name);
    }
}

void telemetry_publish_error(const char *error_message) {
    if (!state.telemetry_enabled) return;
    
    telemetry_publish(TOPIC_ERROR, error_message, QOS_ERROR);
    
    printf("[ERROR] %s\n", error_message);
}

// ============================================================================
// STRUCTURED DATA PUBLISHING
// ============================================================================

bool telemetry_publish_speed_packet(const SpeedTelemetry *packet) {
    if (!state.telemetry_enabled || packet == NULL) return false;
    
    // Create JSON payload
    char json[TELEMETRY_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"left\":%.1f,\"right\":%.1f,\"avg\":%.1f,\"ts\":%lu}",
             packet->left_speed_mm_s,
             packet->right_speed_mm_s,
             packet->average_speed_mm_s,
             packet->timestamp_ms);
    
    return telemetry_publish_json("robot/demo1/speed/data", json, QOS_TELEMETRY);
}

bool telemetry_publish_distance_packet(const DistanceTelemetry *packet) {
    if (!state.telemetry_enabled || packet == NULL) return false;
    
    char json[TELEMETRY_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"left\":%.1f,\"right\":%.1f,\"total\":%.1f,\"ts\":%lu}",
             packet->left_distance_mm,
             packet->right_distance_mm,
             packet->total_distance_mm,
             packet->timestamp_ms);
    
    return telemetry_publish_json("robot/demo1/distance/data", json, QOS_TELEMETRY);
}

bool telemetry_publish_heading_packet(const HeadingTelemetry *packet) {
    if (!state.telemetry_enabled || packet == NULL) return false;
    
    char json[TELEMETRY_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"filtered\":%.2f,\"raw\":%.2f,\"error\":%.2f,\"target\":%.2f,\"ts\":%lu}",
             packet->filtered_heading_deg,
             packet->raw_heading_deg,
             packet->heading_error_deg,
             packet->target_heading_deg,
             packet->timestamp_ms);
    
    return telemetry_publish_json("robot/demo1/heading/data", json, QOS_TELEMETRY);
}

bool telemetry_publish_imu_packet(const IMUTelemetry *packet) {
    if (!state.telemetry_enabled || packet == NULL) return false;
    
    char json[TELEMETRY_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"gyro_z\":%.2f,\"accel_x\":%.3f,\"accel_y\":%.3f,\"filter\":%s,\"ts\":%lu}",
             packet->gyro_z_deg_s,
             packet->accel_x_g,
             packet->accel_y_g,
             packet->filter_active ? "true" : "false",
             packet->timestamp_ms);
    
    return telemetry_publish_json("robot/demo1/imu/data", json, QOS_TELEMETRY);
}

bool telemetry_publish_system_status(const SystemStatus *status) {
    if (!state.telemetry_enabled || status == NULL) return false;
    
    char json[TELEMETRY_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"status\":\"%s\",\"uptime\":%lu,\"sent\":%lu,\"failed\":%lu,"
             "\"latency\":%.1f,\"wifi\":%s,\"mqtt\":%s}",
             telemetry_connection_status_string(status->connection_status),
             status->uptime_sec,
             status->packets_sent,
             status->packets_failed,
             status->mqtt_latency_ms,
             status->wifi_connected ? "true" : "false",
             status->mqtt_connected ? "true" : "false");
    
    return telemetry_publish_json("robot/demo1/system/status", json, QOS_STATUS);
}

// ============================================================================
// GENERIC MQTT PUBLISH
// ============================================================================

bool telemetry_publish(const char *topic, const char *payload, MQTTQoS qos) {
    if (!state.telemetry_enabled) return false;
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    /* Real MQTT publish - uncomment when library is added
    if (state.mqtt_connected && state.mqtt_client != NULL) {
        err_t err = mqtt_publish(state.mqtt_client, topic, payload, 
                                strlen(payload), qos, 0, 
                                mqtt_publish_callback, NULL);
        
        uint32_t end_time = to_ms_since_boot(get_absolute_time());
        state.last_latency_ms = (float)(end_time - start_time);
        
        if (err != ERR_OK) {
            state.packets_failed++;
            return false;
        }
        
        state.packets_sent++;
        return true;
    }
    */
    
    // Simulated publish for testing
    if (state.mqtt_connected) {
        state.packets_sent++;
        return true;
    }
    
    state.packets_failed++;
    return false;
}

bool telemetry_publish_float(const char *topic, float value, MQTTQoS qos) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.3f", value);
    return telemetry_publish(topic, buffer, qos);
}

bool telemetry_publish_json(const char *topic, const char *json, MQTTQoS qos) {
    return telemetry_publish(topic, json, qos);
}

// ============================================================================
// SUBSCRIPTION (for receiving commands)
// ============================================================================

bool telemetry_subscribe(const char *topic, MQTTQoS qos) {
    if (!state.mqtt_connected) return false;
    
    /* Real MQTT subscribe - uncomment when library is added
    if (state.mqtt_client != NULL) {
        err_t err = mqtt_subscribe(state.mqtt_client, topic, qos, 
                                  mqtt_incoming_data_callback, NULL);
        if (err != ERR_OK) {
            printf("✗ Failed to subscribe to %s\n", topic);
            return false;
        }
        printf("✓ Subscribed to %s\n", topic);
        return true;
    }
    */
    
    printf("✓ Subscribed to %s (simulated)\n", topic);
    return true;
}

void telemetry_set_command_callback(void (*callback)(const char *topic, const char *payload)) {
    state.command_callback = callback;
}

// ============================================================================
// STATUS AND CONTROL
// ============================================================================

ConnectionStatus telemetry_get_status(void) {
    return state.status;
}

bool telemetry_is_connected(void) {
    return state.wifi_connected && state.mqtt_connected;
}

bool telemetry_wifi_is_connected(void) {
    return state.wifi_connected;
}

bool telemetry_mqtt_is_connected(void) {
    return state.mqtt_connected;
}

uint32_t telemetry_get_packets_sent(void) {
    return state.packets_sent;
}

uint32_t telemetry_get_packets_failed(void) {
    return state.packets_failed;
}

float telemetry_get_latency_ms(void) {
    return state.last_latency_ms;
}

bool telemetry_should_publish(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    if (current_time - state.last_publish_time >= TELEMETRY_PUBLISH_INTERVAL_MS) {
        state.last_publish_time = current_time;
        return true;
    }
    
    return false;
}

void telemetry_enable(bool enable) {
    state.telemetry_enabled = enable;
    if (enable) {
        printf("✓ Telemetry enabled\n");
    } else {
        printf("✗ Telemetry disabled\n");
    }
}

bool telemetry_is_enabled(void) {
    return state.telemetry_enabled;
}

void telemetry_update(void) {
    // Check connection and attempt reconnection if needed
    if (!telemetry_is_connected()) {
        telemetry_reconnect();
    }
    
    // Process any pending MQTT messages (when using real library)
    /* Uncomment when using real MQTT
    if (state.mqtt_client != NULL) {
        mqtt_client_poll(state.mqtt_client);
    }
    */
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void telemetry_print_status(void) {
    uint32_t uptime = (to_ms_since_boot(get_absolute_time()) - state.start_time) / 1000;
    
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    TELEMETRY STATUS                           ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    printf("Connection Status: %s\n", telemetry_connection_status_string(state.status));
    printf("WiFi:  %s\n", state.wifi_connected ? "✓ Connected" : "✗ Disconnected");
    printf("MQTT:  %s\n", state.mqtt_connected ? "✓ Connected" : "✗ Disconnected");
    printf("\n");
    printf("Uptime:         %lu seconds\n", uptime);
    printf("Packets Sent:   %lu\n", state.packets_sent);
    printf("Packets Failed: %lu\n", state.packets_failed);
    printf("Success Rate:   %.1f%%\n", 
           state.packets_sent > 0 ? 
           (100.0f * state.packets_sent / (state.packets_sent + state.packets_failed)) : 0);
    printf("Last Latency:   %.1f ms\n", state.last_latency_ms);
    printf("\n");
}

void telemetry_print_separator(void) {
    printf("───────────────────────────────────────────────────────────────\n");
}

void telemetry_print_header(const char *title) {
    printf("\n╔═══════════════════════════════════════════════════════════╗\n");
    printf("║ %-57s ║\n", title);
    printf("╚═══════════════════════════════════════════════════════════╝\n\n");
}

const char* telemetry_connection_status_string(ConnectionStatus status) {
    switch (status) {
        case CONN_STATUS_DISCONNECTED:    return "Disconnected";
        case CONN_STATUS_WIFI_CONNECTING: return "WiFi Connecting";
        case CONN_STATUS_WIFI_CONNECTED:  return "WiFi Connected";
        case CONN_STATUS_MQTT_CONNECTING: return "MQTT Connecting";
        case CONN_STATUS_MQTT_CONNECTED:  return "MQTT Connected";
        case CONN_STATUS_ERROR:           return "Error";
        default:                          return "Unknown";
    }
}

// ============================================================================
// LEGACY COMPATIBILITY
// ============================================================================

void telemetry_publish_line_position(int32_t position) {
    if (!state.telemetry_enabled) return;
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%ld", position);
    telemetry_publish("robot/demo2/line/position", buffer, QOS_TELEMETRY);
    
    if (TELEMETRY_ENABLE_SERIAL_FALLBACK || !state.mqtt_connected) {
        printf("[LINE] Position: %ld\n", position);
    }
}

void telemetry_publish_barcode(const char *command) {
    if (!state.telemetry_enabled) return;
    telemetry_publish("robot/demo2/barcode", command, QOS_STATUS);
    
    if (TELEMETRY_ENABLE_SERIAL_FALLBACK || !state.mqtt_connected) {
        printf("[BARCODE] Command: %s\n", command);
    }
}

void telemetry_publish_obstacle(float distance, float width) {
    if (!state.telemetry_enabled) return;
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "{\"distance\":%.1f,\"width\":%.1f}", distance, width);
    telemetry_publish("robot/demo3/obstacle", buffer, QOS_STATUS);
    
    if (TELEMETRY_ENABLE_SERIAL_FALLBACK || !state.mqtt_connected) {
        printf("[OBSTACLE] Distance: %.1f mm | Width: %.1f mm\n", distance, width);
    }
}