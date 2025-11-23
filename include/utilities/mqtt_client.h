/**
 * @file    mqtt_client.h
 * @brief   MQTT client interface
 * @details Minimal MQTT 3.1.1 client implementation using lwIP TCP
 *          raw API for embedded systems
 */

#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MQTT_DISCONNECTED = 0,
    MQTT_CONNECTING,
    MQTT_CONNECTED,
    MQTT_ERROR
} MQTTStatus;

/**
 * @brief Initialize MQTT client
 * @param broker_ip MQTT broker IP address string
 * @param port MQTT broker port (typically 1883)
 * @param client_id Client identifier string
 */
void mqtt_init(const char *broker_ip, uint16_t port, const char *client_id);

/**
 * @brief Connect to MQTT broker
 * @return true if connection successful
 */
bool mqtt_connect(void);

/**
 * @brief Disconnect from MQTT broker
 */
void mqtt_disconnect(void);

/**
 * @brief Check if MQTT client is connected
 * @return true if connected
 */
bool mqtt_is_connected(void);

/**
 * @brief Get current MQTT connection status
 * @return Current MQTTStatus
 */
MQTTStatus mqtt_get_status(void);

/**
 * @brief Process MQTT client tasks
 * @details Handles keepalive pings and reconnection attempts
 */
void mqtt_process(void);

/**
 * @brief Publish message to MQTT topic
 * @param topic Topic string
 * @param message Message payload
 * @param qos Quality of Service level (0, 1, or 2)
 * @return true if publish successful
 */
bool mqtt_publish(const char *topic, const char *message, uint8_t qos);

#endif /* MQTT_CLIENT_H */
