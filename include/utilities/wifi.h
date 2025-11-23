/**
 * @file    wifi.h
 * @brief   WiFi connection management interface
 * @details Provides WiFi connection management functions for Raspberry Pi
 *          Pico W including initialization, connection handling, and
 *          status monitoring
 */

#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>
#include <stddef.h>

typedef enum
{
    WIFI_DISCONNECTED = 0,
    WIFI_CONNECTING,
    WIFI_CONNECTED,
    WIFI_ERROR
} WiFiStatus;

/**
 * @brief Initialize WiFi subsystem
 */
void wifi_init(void);

/**
 * @brief Connect to WiFi network
 * @param ssid Network SSID
 * @param password Network password
 * @return true if connection successful
 */
bool wifi_connect(const char *ssid, const char *password);

/**
 * @brief Disconnect from WiFi network
 */
void wifi_disconnect(void);

/**
 * @brief Check if WiFi is connected
 * @return true if connected
 */
bool wifi_is_connected(void);

/**
 * @brief Get current WiFi status
 * @return Current WiFiStatus
 */
WiFiStatus wifi_get_status(void);

/**
 * @brief Get IP address string
 * @param buf Buffer to store IP address
 * @param len Buffer length (minimum 16 bytes)
 * @return true if IP address retrieved successfully
 */
bool wifi_get_ip(char *buf, size_t len);

#endif /* WIFI_H */
