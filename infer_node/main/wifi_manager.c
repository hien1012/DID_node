#include "wifi_manager.h"

#include <string.h>

#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "app_config.h"

#define WIFI_CONNECTED_BIT BIT0

/*
 * Reconnect policy: every disconnect schedules esp_wifi_connect() after a
 * backoff that doubles from APP_WIFI_RECONNECT_MIN_MS up to
 * APP_WIFI_RECONNECT_MAX_MS and resets once an IP is obtained. Escalation
 * (forced reconnect, driver restart) is decided by node_health.c.
 */

static const char *TAG = "WIFI";

static EventGroupHandle_t s_wifi_events = NULL;
static esp_event_handler_instance_t s_wifi_handler;
static esp_event_handler_instance_t s_ip_handler;
static esp_timer_handle_t s_reconnect_timer = NULL;
static bool s_initialized = false;
static volatile bool s_stopping = false;
static uint32_t s_backoff_ms = APP_WIFI_RECONNECT_MIN_MS;
static volatile int64_t s_offline_since_us = 0;
static wifi_manager_stats_t s_stats;

static void reconnect_timer_callback(void *arg)
{
    (void)arg;
    if (!s_stopping) {
        esp_wifi_connect();
    }
}

static void schedule_reconnect(void)
{
    if (s_stopping || esp_timer_is_active(s_reconnect_timer)) {
        return;
    }
    ESP_LOGW(TAG, "Reconnecting in %u ms", (unsigned)s_backoff_ms);
    esp_timer_start_once(s_reconnect_timer, (uint64_t)s_backoff_ms * 1000ULL);
    s_backoff_ms = s_backoff_ms * 2U > APP_WIFI_RECONNECT_MAX_MS
                       ? APP_WIFI_RECONNECT_MAX_MS
                       : s_backoff_ms * 2U;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event =
            (const wifi_event_sta_disconnected_t *)event_data;
        const bool was_connected =
            (xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT) != 0;
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        s_stats.last_reason = event->reason;
        if (was_connected) {
            ++s_stats.disconnects;
            s_offline_since_us = esp_timer_get_time();
            ESP_LOGW(TAG, "Disconnected, reason=%u", (unsigned)event->reason);
        }
        schedule_reconnect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected, IP=" IPSTR, IP2STR(&event->ip_info.ip));
        s_backoff_ms = APP_WIFI_RECONNECT_MIN_MS;
        s_offline_since_us = 0;
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_manager_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_wifi_events = xEventGroupCreate();
    if (!s_wifi_events) {
        return ESP_ERR_NO_MEM;
    }
    const esp_timer_create_args_t timer_args = {
        .callback = reconnect_timer_callback,
        .name = "wifi_reconnect",
    };
    esp_err_t err = esp_timer_create(&timer_args, &s_reconnect_timer);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    if (!esp_netif_create_default_wifi_sta()) {
        return ESP_FAIL;
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_config);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, &s_wifi_handler);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, &s_ip_handler);
    if (err != ESP_OK) {
        return err;
    }

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, APP_WIFI_SSID,
            sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, APP_WIFI_PASSWORD,
            sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    s_offline_since_us = esp_timer_get_time();
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    }
    if (err == ESP_OK) {
        err = esp_wifi_start();
    }
    if (err == ESP_OK) {
        /* USB powered: no modem sleep, for upload throughput and latency. */
        err = esp_wifi_set_ps(WIFI_PS_NONE);
    }
    if (err != ESP_OK) {
        return err;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t wifi_manager_wait_connected(TickType_t timeout_ticks)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, timeout_ticks);
    return (bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool wifi_manager_is_connected(void)
{
    return s_initialized &&
           (xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT) != 0;
}

int8_t wifi_manager_get_rssi(void)
{
    wifi_ap_record_t ap_info = {0};
    if (!wifi_manager_is_connected() || esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
        return -127;
    }
    return ap_info.rssi;
}

uint32_t wifi_manager_offline_seconds(void)
{
    const int64_t since_us = s_offline_since_us;
    if (wifi_manager_is_connected() || since_us == 0) {
        return 0;
    }
    return (uint32_t)((esp_timer_get_time() - since_us) / 1000000LL);
}

void wifi_manager_get_stats(wifi_manager_stats_t *stats)
{
    if (stats) {
        *stats = s_stats;
    }
}

void wifi_manager_force_reconnect(void)
{
    if (!s_initialized) {
        return;
    }
    ESP_LOGW(TAG, "Forcing reconnect");
    /* The resulting STA_DISCONNECTED event schedules the reconnect. */
    esp_wifi_disconnect();
}

void wifi_manager_restart(void)
{
    if (!s_initialized) {
        return;
    }
    ESP_LOGW(TAG, "Restarting Wi-Fi driver");
    s_stopping = true;
    esp_timer_stop(s_reconnect_timer);
    esp_wifi_stop();
    s_stopping = false;
    s_backoff_ms = APP_WIFI_RECONNECT_MIN_MS;
    ++s_stats.restarts;
    /* STA_START connects again. */
    if (esp_wifi_start() == ESP_OK) {
        esp_wifi_set_ps(WIFI_PS_NONE);
    }
}
