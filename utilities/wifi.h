/**
 * wifi.h
 * WiFi connection management for Pico W
 */

#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>
#include <stddef.h>

typedef enum {
    WIFI_DISCONNECTED = 0,
    WIFI_CONNECTING,
    WIFI_CONNECTED,
    WIFI_ERROR
} WiFiStatus;

void wifi_init(void);
bool wifi_connect(const char *ssid, const char *password);
void wifi_disconnect(void);
bool wifi_is_connected(void);
WiFiStatus wifi_get_status(void);
bool wifi_get_ip(char *buf, size_t len);

#endif // WIFI_H