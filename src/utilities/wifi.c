/**
 * wifi.c
 * WiFi connection management implementation
 */

#include "wifi.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <string.h>

#define MAX_CONNECT_ATTEMPTS 4
#define CONNECT_RETRY_DELAY_MS 3000

static WiFiStatus wifi_status = WIFI_DISCONNECTED;

void wifi_init(void) {
    if (cyw43_arch_init()) {
        printf("Failed to initialize WiFi\n");
        wifi_status = WIFI_ERROR;
        return;
    }
    cyw43_arch_enable_sta_mode();
    wifi_status = WIFI_DISCONNECTED;
    printf("WiFi initialized\n");
}

bool wifi_connect(const char *ssid, const char *password) {
    if (wifi_status == WIFI_ERROR) {
        return false;
    }

    printf("Connecting to WiFi: %s\n", ssid);
    wifi_status = WIFI_CONNECTING;

    for (int attempt = 1; attempt <= MAX_CONNECT_ATTEMPTS; attempt++) {
        printf("Attempt %d/%d...\n", attempt, MAX_CONNECT_ATTEMPTS);
        
        int result = cyw43_arch_wifi_connect_timeout_ms(
            ssid, 
            password, 
            CYW43_AUTH_WPA2_AES_PSK,
            10000  // 10 second timeout per attempt
        );

        if (result == 0) {
            wifi_status = WIFI_CONNECTED;
            printf("WiFi connected!\n");
            
            char ip_str[16];
            if (wifi_get_ip(ip_str, sizeof(ip_str))) {
                printf("IP Address: %s\n", ip_str);
            }
            return true;
        }

        printf("Connection failed (code %d)\n", result);
        
        if (attempt < MAX_CONNECT_ATTEMPTS) {
            uint32_t delay = CONNECT_RETRY_DELAY_MS * (1 << (attempt - 1));
            printf("Retrying in %lums...\n", delay);
            sleep_ms(delay);
        }
    }

    printf("Failed to connect after %d attempts\n", MAX_CONNECT_ATTEMPTS);
    wifi_status = WIFI_ERROR;
    return false;
}

void wifi_disconnect(void) {
    cyw43_arch_deinit();
    wifi_status = WIFI_DISCONNECTED;
    printf("WiFi disconnected\n");
}

bool wifi_is_connected(void) {
    return wifi_status == WIFI_CONNECTED && 
           cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

WiFiStatus wifi_get_status(void) {
    return wifi_status;
}

bool wifi_get_ip(char *buf, size_t len) {
    if (!wifi_is_connected() || buf == NULL || len < 16) {
        return false;
    }

    uint32_t ip = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
    snprintf(buf, len, "%lu.%lu.%lu.%lu",
             ip & 0xFF,
             (ip >> 8) & 0xFF,
             (ip >> 16) & 0xFF,
             (ip >> 24) & 0xFF);
    
    return true;
}