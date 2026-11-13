#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "esp_err.h"

typedef struct {
    uint32_t disconnects;       /* since boot */
    uint32_t restarts;          /* full Wi-Fi stack restarts since boot */
    uint8_t last_reason;        /* wifi_err_reason_t of the last disconnect */
} wifi_manager_stats_t;

esp_err_t wifi_manager_init(void);
esp_err_t wifi_manager_wait_connected(TickType_t timeout_ticks);
bool wifi_manager_is_connected(void);
int8_t wifi_manager_get_rssi(void);

/* Seconds since the link was lost (0 while connected). */
uint32_t wifi_manager_offline_seconds(void);
void wifi_manager_get_stats(wifi_manager_stats_t *stats);

/* Drop the association; the normal reconnect path takes over. */
void wifi_manager_force_reconnect(void);
/* Stop and start the Wi-Fi driver (last resort for a stuck link). */
void wifi_manager_restart(void);
