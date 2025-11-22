/**
 * mqtt_client.c
 * Minimal MQTT 3.1.1 client using lwIP TCP raw API
 */
#include "pico/cyw43_arch.h"
#include "mqtt_client.h"
#include "pico/stdlib.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include <stdio.h>
#include <string.h>

#define MQTT_CONNECT_TIMEOUT_MS 10000
#define MQTT_KEEPALIVE_SEC 60
#define MQTT_TX_BUFFER_SIZE 512

// MQTT Control Packet types
#define MQTT_CONNECT    0x10
#define MQTT_CONNACK    0x20
#define MQTT_PUBLISH    0x30
#define MQTT_PINGREQ    0xC0
#define MQTT_PINGRESP   0xD0
#define MQTT_DISCONNECT 0xE0

typedef struct {
    struct tcp_pcb *pcb;
    ip_addr_t broker_addr;
    uint16_t broker_port;
    char client_id[32];
    MQTTStatus status;
    uint16_t packet_id;
    uint8_t tx_buffer[MQTT_TX_BUFFER_SIZE];
    uint32_t last_activity;
} mqtt_client_state_t;

static mqtt_client_state_t mqtt_state = {0};

// Helper: Write 2-byte length
static void write_length(uint8_t *buf, uint16_t len) {
    buf[0] = (len >> 8) & 0xFF;
    buf[1] = len & 0xFF;
}

// Helper: Write string with length prefix
static uint16_t write_string(uint8_t *buf, const char *str) {
    uint16_t len = strlen(str);
    write_length(buf, len);
    memcpy(buf + 2, str, len);
    return len + 2;
}

// Helper: Write remaining length (variable length encoding)
static uint8_t write_remaining_length(uint8_t *buf, uint32_t len) {
    uint8_t count = 0;
    do {
        uint8_t byte = len % 128;
        len /= 128;
        if (len > 0) {
            byte |= 0x80;
        }
        buf[count++] = byte;
    } while (len > 0);
    return count;
}

// TCP error callback
static void mqtt_tcp_err(void *arg, err_t err) {
    printf("MQTT TCP error: %d\n", err);
    mqtt_state.status = MQTT_ERROR;
    mqtt_state.pcb = NULL;
}

// TCP receive callback
static err_t mqtt_tcp_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        // Connection closed
        printf("MQTT connection closed by server\n");
        mqtt_state.status = MQTT_DISCONNECTED;
        tcp_close(pcb);
        mqtt_state.pcb = NULL;
        return ERR_OK;
    }

    // Process MQTT packets
    uint8_t *data = (uint8_t *)p->payload;
    uint8_t packet_type = data[0] & 0xF0;

    if (packet_type == MQTT_CONNACK) {
        // Check connection accepted (return code 0)
        if (p->tot_len >= 4 && data[3] == 0) {
            mqtt_state.status = MQTT_CONNECTED;
            printf("MQTT connected\n");
        } else {
            mqtt_state.status = MQTT_ERROR;
            printf("MQTT connection rejected\n");
        }
    }

    mqtt_state.last_activity = to_ms_since_boot(get_absolute_time());
    
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

// TCP sent callback
static err_t mqtt_tcp_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {
    mqtt_state.last_activity = to_ms_since_boot(get_absolute_time());
    return ERR_OK;
}

// TCP connected callback
static err_t mqtt_tcp_connected(void *arg, struct tcp_pcb *pcb, err_t err) {
    if (err != ERR_OK) {
        printf("MQTT TCP connect failed: %d\n", err);
        mqtt_state.status = MQTT_ERROR;
        return err;
    }

    // Build CONNECT packet
    uint8_t *buf = mqtt_state.tx_buffer;
    uint16_t pos = 0;

    // Fixed header
    buf[pos++] = MQTT_CONNECT;
    pos++; // Skip remaining length, fill later

    // Variable header
    pos += write_string(&buf[pos], "MQTT");  // Protocol name
    buf[pos++] = 0x04;  // Protocol level (3.1.1)
    buf[pos++] = 0x02;  // Connect flags (Clean Session)
    buf[pos++] = (MQTT_KEEPALIVE_SEC >> 8) & 0xFF;  // Keepalive MSB
    buf[pos++] = MQTT_KEEPALIVE_SEC & 0xFF;         // Keepalive LSB

    // Payload - Client ID
    pos += write_string(&buf[pos], mqtt_state.client_id);

    // Fill in remaining length
    uint32_t remaining_len = pos - 2;
    uint8_t len_bytes = write_remaining_length(&buf[1], remaining_len);
    
    // Shift data if remaining length > 1 byte
    if (len_bytes > 1) {
        memmove(&buf[1 + len_bytes], &buf[2], remaining_len);
        pos = 1 + len_bytes + remaining_len;
    }

    // Send CONNECT packet
    err_t result = tcp_write(pcb, buf, pos, TCP_WRITE_FLAG_COPY);
    if (result != ERR_OK) {
        printf("MQTT send CONNECT failed: %d\n", result);
        mqtt_state.status = MQTT_ERROR;
        return result;
    }

    tcp_output(pcb);
    return ERR_OK;
}




void mqtt_init(const char *broker_ip, uint16_t port, const char *client_id_str) {
    memset(&mqtt_state, 0, sizeof(mqtt_state));
    
    mqtt_state.broker_port = port;
    mqtt_state.packet_id = 1;
    
    if (client_id_str != NULL) {
        strncpy(mqtt_state.client_id, client_id_str, sizeof(mqtt_state.client_id) - 1);
    } else {
        strcpy(mqtt_state.client_id, "pico_robot");
    }

    if (!ip4addr_aton(broker_ip, &mqtt_state.broker_addr)) {
        printf("Invalid broker IP address\n");
        mqtt_state.status = MQTT_ERROR;
        return;
    }

    mqtt_state.status = MQTT_DISCONNECTED;
    printf("MQTT initialized (broker: %s:%u, client: %s)\n", 
           broker_ip, port, mqtt_state.client_id);
}

bool mqtt_connect(void) {
    if (mqtt_state.status == MQTT_ERROR) {
        return false;
    }

    if (mqtt_state.status == MQTT_CONNECTED) {
        return true;
    }

    // Create new TCP PCB
    mqtt_state.pcb = tcp_new();
    if (mqtt_state.pcb == NULL) {
        printf("Failed to create TCP PCB\n");
        mqtt_state.status = MQTT_ERROR;
        return false;
    }

    // Set up callbacks
    tcp_arg(mqtt_state.pcb, &mqtt_state);
    tcp_err(mqtt_state.pcb, mqtt_tcp_err);
    tcp_recv(mqtt_state.pcb, mqtt_tcp_recv);
    tcp_sent(mqtt_state.pcb, mqtt_tcp_sent);

    mqtt_state.status = MQTT_CONNECTING;
    printf("Connecting to MQTT broker...\n");

    // Connect to broker
    err_t err = tcp_connect(mqtt_state.pcb, &mqtt_state.broker_addr, 
                            mqtt_state.broker_port, mqtt_tcp_connected);
    if (err != ERR_OK) {
        printf("TCP connect failed: %d\n", err);
        tcp_close(mqtt_state.pcb);
        mqtt_state.pcb = NULL;
        mqtt_state.status = MQTT_ERROR;
        return false;
    }

    // Wait for connection
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (mqtt_state.status == MQTT_CONNECTING && 
           (to_ms_since_boot(get_absolute_time()) - start) < MQTT_CONNECT_TIMEOUT_MS) {
        sleep_ms(100);
    }

    return mqtt_state.status == MQTT_CONNECTED;
}

void mqtt_disconnect(void) {
    if (mqtt_state.pcb != NULL) {
        // Send DISCONNECT packet
        uint8_t disconnect[] = {MQTT_DISCONNECT, 0x00};
        tcp_write(mqtt_state.pcb, disconnect, sizeof(disconnect), TCP_WRITE_FLAG_COPY);
        tcp_output(mqtt_state.pcb);
        
        tcp_close(mqtt_state.pcb);
        mqtt_state.pcb = NULL;
    }
    mqtt_state.status = MQTT_DISCONNECTED;
    printf("MQTT disconnected\n");
}

bool mqtt_is_connected(void) {
    return mqtt_state.status == MQTT_CONNECTED && mqtt_state.pcb != NULL;
}

MQTTStatus mqtt_get_status(void) {
    return mqtt_state.status;
}

// Add this new function after mqtt_get_status()
void mqtt_process(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Check if connection is dead
    if (mqtt_state.pcb == NULL || mqtt_state.status != MQTT_CONNECTED) {
        // Try to reconnect every 5 seconds
        static uint32_t last_reconnect_attempt = 0;
        if ((now - last_reconnect_attempt) > 5000) {
            printf("MQTT: Connection lost, attempting reconnect...\n");
            mqtt_connect();
            last_reconnect_attempt = now;
        }
        return;
    }
    
    // Send PINGREQ every 20 seconds (more frequent = more stable)
    if ((now - mqtt_state.last_activity) > 20000) {
        // Send PINGREQ
        uint8_t pingreq[] = {MQTT_PINGREQ, 0x00};
        err_t err = tcp_write(mqtt_state.pcb, pingreq, sizeof(pingreq), TCP_WRITE_FLAG_COPY);
        if (err == ERR_OK) {
            tcp_output(mqtt_state.pcb);
            mqtt_state.last_activity = now;
            printf("MQTT: PINGREQ sent\n");
        } else {
            printf("MQTT: Failed to send PINGREQ, error %d\n", err);
            mqtt_state.status = MQTT_ERROR;
        }
    }
    
    // Detect connection timeout (no activity for 120 seconds = dead)
    if ((now - mqtt_state.last_activity) > 120000) {
        printf("MQTT: Connection timeout, forcing reconnect\n");
        mqtt_state.status = MQTT_DISCONNECTED;
        if (mqtt_state.pcb != NULL) {
            tcp_abort(mqtt_state.pcb);
            mqtt_state.pcb = NULL;
        }
    }
}



bool mqtt_publish(const char *topic, const char *payload, uint8_t qos) {
    if (mqtt_state.status != MQTT_CONNECTED || mqtt_state.pcb == NULL) {
        printf("ERROR: MQTT not connected\n");
        return false;
    }

    uint16_t topic_len = strlen(topic);
    uint16_t payload_len = strlen(payload);
    
    // Build PUBLISH packet
    uint8_t *p = mqtt_state.tx_buffer;
    *p++ = MQTT_PUBLISH | (qos << 1) | 0x01;  // Fixed header with RETAIN
    
    // Calculate remaining length
    uint16_t remaining_len = 2 + topic_len + payload_len;
    if (remaining_len > 127) {
        *p++ = (remaining_len & 0x7F) | 0x80;
        *p++ = remaining_len >> 7;
    } else {
        *p++ = remaining_len;
    }
    
    // Topic length + topic
    *p++ = (topic_len >> 8) & 0xFF;
    *p++ = topic_len & 0xFF;
    memcpy(p, topic, topic_len);
    p += topic_len;
    
    // Payload
    memcpy(p, payload, payload_len);
    p += payload_len;
    
    size_t total_len = p - mqtt_state.tx_buffer;
    
    // CRITICAL: Check if we can write
    if (tcp_sndbuf(mqtt_state.pcb) < total_len) {
        printf("ERROR: TCP send buffer full! Available=%u, needed=%zu\n", 
               tcp_sndbuf(mqtt_state.pcb), total_len);
        return false;
    }
    
    // Write to TCP buffer
    err_t err = tcp_write(mqtt_state.pcb, mqtt_state.tx_buffer, total_len, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        printf("ERROR: tcp_write failed! err=%d\n", err);
        return false;
    }
    
    // Force send
    err = tcp_output(mqtt_state.pcb);
    if (err != ERR_OK) {
        printf("ERROR: tcp_output failed! err=%d\n", err);
        // Don't return false yet - the write might still succeed
        // Connection might be temporarily busy
    }
    
    mqtt_state.last_activity = to_ms_since_boot(get_absolute_time());
    return true;  // Return true even if tcp_output fails (data is buffered)
}


