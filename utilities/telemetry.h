#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>

// ===== Wi-Fi / MQTT configuration (edit these as needed) =====
#define WIFI_SSID     "Javiersphone"
#define WIFI_PASS     "imcool123"
#define MQTT_BROKER   "10.194.254.160"  // set to your Mosquitto host (e.g., 10.132.221.160)
#define MQTT_PORT     1883
#define MQTT_CLIENTID "pico-demo1"

// ===== MQTT topics =====
#define TOPIC_HEADING  "robot/heading"
#define TOPIC_SPEED    "robot/speed"
#define TOPIC_DISTANCE "robot/distance"

// ===== Status codes =====
typedef enum {
    TELEMETRY_SUCCESS = 0,
    TELEMETRY_ERROR_NOT_CONNECTED,
    TELEMETRY_ERROR_PUBLISH,
    TELEMETRY_ERROR_INVALID_PARAM
} TelemetryStatus;

// ===== Public API =====

// Bring up Wi-Fi (Pico W) and connect to MQTT broker.
// Safe to call once at startup; non-fatal if Wi-Fi/MQTT is temporarily down.
TelemetryStatus telemetry_begin(void);

// Quick check if MQTT session is live
bool telemetry_is_connected(void);

// Publishers (JSON payloads):
//   robot/heading  -> {"raw_deg":..,"ema_deg":..}
TelemetryStatus telemetry_publish_heading(float heading_raw_deg, float heading_ema_deg);

//   robot/speed    -> {"left_mm_s":..,"right_mm_s":..}
TelemetryStatus telemetry_publish_speed(float left_speed_mm_s, float right_speed_mm_s);

//   robot/distance -> {"left_mm":..,"right_mm":..}
TelemetryStatus telemetry_publish_distance(float left_distance_mm, float right_distance_mm);

#endif
