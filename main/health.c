#include "health.h"

#include <stdint.h>
#include <string.h>
#include <time.h>

#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "app_config.h"
#include "device_identity.h"
#include "storage.h"
#include "wifi_manager.h"

typedef struct {
    uint32_t recordings_ok;
    uint32_t recordings_failed;
    uint32_t uploads_ok;
    uint32_t uploads_failed;
    int last_record_error;
    int last_upload_error;
    time_t last_recorded_at;
} health_state_t;

static const char *TAG = "HEALTH";
static SemaphoreHandle_t s_mutex = NULL;
static health_state_t s_state;

static esp_err_t send_heartbeat(void)
{
    health_state_t snapshot = {0};
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        snapshot = s_state;
        xSemaphoreGive(s_mutex);
    }

    size_t pending_files = 0;
    uint64_t sd_free_bytes = 0;
    (void)storage_count_done(&pending_files);
    (void)storage_get_free_bytes(&sd_free_bytes);

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddStringToObject(root, "device_id", device_identity_get());
    cJSON_AddNumberToObject(root, "reported_at", (double)time(NULL));
    cJSON_AddNumberToObject(root, "uptime_seconds",
                            (double)(esp_timer_get_time() / 1000000));
    cJSON_AddBoolToObject(root, "wifi_connected", wifi_manager_is_connected());
    cJSON_AddNumberToObject(root, "wifi_rssi", wifi_manager_get_rssi());
    cJSON_AddNumberToObject(root, "free_heap_bytes", esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "sd_free_bytes", (double)sd_free_bytes);
    cJSON_AddNumberToObject(root, "pending_files", (double)pending_files);
    cJSON_AddNumberToObject(root, "recordings_ok", snapshot.recordings_ok);
    cJSON_AddNumberToObject(root, "recordings_failed", snapshot.recordings_failed);
    cJSON_AddNumberToObject(root, "uploads_ok", snapshot.uploads_ok);
    cJSON_AddNumberToObject(root, "uploads_failed", snapshot.uploads_failed);
    cJSON_AddNumberToObject(root, "last_record_error", snapshot.last_record_error);
    cJSON_AddNumberToObject(root, "last_upload_error", snapshot.last_upload_error);
    cJSON_AddNumberToObject(root, "last_recorded_at", (double)snapshot.last_recorded_at);
    cJSON_AddNumberToObject(root, "reset_reason", esp_reset_reason());

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) {
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_config_t config = {
        .url = APP_SERVER_HEARTBEAT_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = APP_HTTP_TIMEOUT_MS,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        cJSON_free(json);
        return ESP_ERR_NO_MEM;
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json, (int)strlen(json));
    esp_err_t err = esp_http_client_perform(client);
    int status = err == ESP_OK ? esp_http_client_get_status_code(client) : 0;
    if (err == ESP_OK && status != 200) {
        err = ESP_FAIL;
    }
    esp_http_client_cleanup(client);
    cJSON_free(json);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Heartbeat acknowledged");
    } else {
        ESP_LOGW(TAG, "Heartbeat failed: %s, HTTP %d",
                 esp_err_to_name(err), status);
    }
    return err;
}

static void heartbeat_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        if (wifi_manager_is_connected()) {
            (void)send_heartbeat();
        } else {
            ESP_LOGW(TAG, "Heartbeat skipped: Wi-Fi disconnected");
        }
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_HEARTBEAT_PERIOD_MS));
    }
}

esp_err_t health_monitor_start(void)
{
    if (s_mutex) {
        return ESP_OK;
    }
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }
    memset(&s_state, 0, sizeof(s_state));
    if (xTaskCreate(heartbeat_task, "heartbeat", 6144, NULL, 4, NULL) != pdPASS) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void health_note_recording(const audio_record_result_t *result)
{
    if (!result || !s_mutex || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return;
    }
    if (result->success) {
        ++s_state.recordings_ok;
    } else {
        ++s_state.recordings_failed;
    }
    s_state.last_record_error = result->error;
    s_state.last_recorded_at = result->recorded_at_epoch;
    xSemaphoreGive(s_mutex);
}

void health_note_upload(const upload_summary_t *summary)
{
    if (!summary || !s_mutex || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return;
    }
    s_state.uploads_ok += (uint32_t)summary->uploaded_count;
    s_state.uploads_failed += (uint32_t)summary->failed_count;
    s_state.last_upload_error = summary->last_error;
    xSemaphoreGive(s_mutex);
}
