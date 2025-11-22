/**
 * mqtt_client.h
 * Simple MQTT client using lwIP TCP raw API
 */

#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MQTT_DISCONNECTED = 0,
    MQTT_CONNECTING,
    MQTT_CONNECTED,
    MQTT_ERROR
} MQTTStatus;

void mqtt_init(const char *broker_ip, uint16_t port, const char *client_id);
bool mqtt_connect(void);
void mqtt_disconnect(void);
bool mqtt_is_connected(void);
MQTTStatus mqtt_get_status(void);
void mqtt_process(void);  // Add this line
bool mqtt_publish(const char *topic, const char *message, uint8_t qos);

#endif // MQTT_CLIENT_H