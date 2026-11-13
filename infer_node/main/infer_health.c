/*
 * Heartbeat + Wi-Fi watchdog (由 main/node_health.c 改寫)。
 *
 * 規則與 did_node 相同 (docs/heartbeat.md)：連續傳輸失敗重建連線、離線過久重啟 driver。
 * 差別：麥克風與相機一直在跑，所以 heartbeat 只在上傳時暫停；內容多了 inference 區塊。
 */
#include "infer_health.h"

#include <stdint.h>
#include <string.h>
#include <time.h>

#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "cam_infer.h"
#include "device_identity.h"
#include "infer_config.h"
#include "mic_infer.h"
#include "node_storage.h"
#include "result_report.h"
#include "sample_store.h"
#include "wifi_manager.h"

#define HEARTBEAT_FORMAT_VERSION  1
#define HEARTBEAT_TASK_STACK      6144U
#define HEARTBEAT_TASK_PRIORITY   4U
#define HEARTBEAT_TIMEOUT_MS      10000

typedef struct {
    infer_state_t state;
    bool has_upload;
    time_t last_upload_at;
    sample_upload_summary_t last_upload;
    uint32_t heartbeat_fail_streak;
} health_state_t;

static const char *TAG = "HEALTH";
static SemaphoreHandle_t s_mutex = NULL;
static health_state_t s_state;
static uint64_t s_sd_free_bytes = 0;

static const char *state_name(infer_state_t state)
{
    switch (state) {
    case INFER_STATE_RUNNING: return "running";
    case INFER_STATE_UPLOADING: return "uploading";
    default: return "booting";
    }
}

static const char *reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    default: return "other";
    }
}

static health_state_t snapshot_state(void)
{
    health_state_t snapshot = {0};
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        snapshot = s_state;
        xSemaphoreGive(s_mutex);
    }
    return snapshot;
}

static char *build_heartbeat_json(const health_state_t *snapshot)
{
    wifi_manager_stats_t wifi_stats = {0};
    wifi_manager_get_stats(&wifi_stats);

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return NULL;
    }
    cJSON_AddNumberToObject(root, "v", HEARTBEAT_FORMAT_VERSION);
    cJSON_AddStringToObject(root, "device_id", device_identity_get());
    cJSON_AddNumberToObject(root, "time", (double)time(NULL));
    cJSON_AddNumberToObject(root, "uptime_s", (double)(esp_timer_get_time() / 1000000));
    cJSON_AddStringToObject(root, "state", state_name(snapshot->state));
    cJSON_AddStringToObject(root, "reset_reason", reset_reason_name(esp_reset_reason()));

    cJSON *wifi = cJSON_AddObjectToObject(root, "wifi");
    cJSON_AddNumberToObject(wifi, "rssi", wifi_manager_get_rssi());
    cJSON_AddNumberToObject(wifi, "disconnects", wifi_stats.disconnects);
    cJSON_AddNumberToObject(wifi, "restarts", wifi_stats.restarts);
    cJSON_AddNumberToObject(wifi, "last_reason", wifi_stats.last_reason);
    cJSON_AddNumberToObject(wifi, "heartbeat_fail_streak", snapshot->heartbeat_fail_streak);

    const size_t pending_audio = sample_store_pending(SAMPLE_MEDIA_AUDIO);
    const size_t pending_video = sample_store_pending(SAMPLE_MEDIA_VIDEO);
    cJSON *storage = cJSON_AddObjectToObject(root, "storage");
    cJSON_AddNumberToObject(storage, "free_mb", (double)(s_sd_free_bytes / (1024ULL * 1024ULL)));
    cJSON_AddNumberToObject(storage, "pending_files", (double)(pending_audio + pending_video));
    cJSON_AddNumberToObject(storage, "broken_files", 0);

    cJSON *memory = cJSON_AddObjectToObject(root, "memory");
    cJSON_AddNumberToObject(memory, "heap_free_kb",
                            (double)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024U));
    cJSON_AddNumberToObject(memory, "heap_min_kb",
                            (double)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024U));
    cJSON_AddNumberToObject(memory, "psram_free_kb",
                            (double)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024U));

    mic_infer_stats_t mic = {0};
    mic_infer_get_stats(&mic);
    cam_infer_stats_t cam = {0};
    cam_infer_get_stats(&cam);
    cJSON *inference = cJSON_AddObjectToObject(root, "inference");
    cJSON_AddNumberToObject(inference, "threshold", (double)INFER_LOW_CONF_THRESHOLD);
    cJSON_AddNumberToObject(inference, "upload_after", INFER_UPLOAD_AFTER_SAMPLES);
    cJSON_AddNumberToObject(inference, "report_sent", result_report_sent());
    cJSON_AddNumberToObject(inference, "report_dropped", result_report_dropped());
    cJSON *pending = cJSON_AddObjectToObject(inference, "pending");
    cJSON_AddNumberToObject(pending, "audio", (double)pending_audio);
    cJSON_AddNumberToObject(pending, "video", (double)pending_video);
    cJSON *audio = cJSON_AddObjectToObject(inference, "audio");
    cJSON_AddBoolToObject(audio, "enabled", INFER_MIC_ENABLE);
    cJSON_AddNumberToObject(audio, "windows", mic.windows);
    cJSON_AddNumberToObject(audio, "dropped", mic.dropped_windows);
    cJSON_AddNumberToObject(audio, "low_conf", mic.low_conf);
    cJSON_AddNumberToObject(audio, "saved", mic.saved);
    cJSON_AddNumberToObject(audio, "overflows", mic.overflows);
    cJSON *video = cJSON_AddObjectToObject(inference, "video");
    cJSON_AddBoolToObject(video, "enabled", INFER_CAM_ENABLE);
    cJSON_AddNumberToObject(video, "frames", cam.frames);
    cJSON_AddNumberToObject(video, "dropped", cam.dropped);
    cJSON_AddNumberToObject(video, "rois", cam.rois);
    cJSON_AddNumberToObject(video, "low_conf", cam.low_conf);
    cJSON_AddNumberToObject(video, "saved", cam.saved);

    if (snapshot->has_upload) {
        const sample_upload_summary_t *upload = &snapshot->last_upload;
        cJSON *last_upload = cJSON_AddObjectToObject(root, "last_upload");
        cJSON_AddNumberToObject(last_upload, "at", (double)snapshot->last_upload_at);
        cJSON_AddNumberToObject(last_upload, "uploaded", (double)upload->uploaded_count);
        cJSON_AddNumberToObject(last_upload, "failed", (double)upload->failed_count);
        cJSON_AddNumberToObject(last_upload, "rejected", (double)upload->rejected_count);
    } else {
        cJSON_AddNullToObject(root, "last_upload");
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

/*
 * Returns ESP_OK when the server acknowledged. *transport_failed tells whether
 * the request got no HTTP response at all (link or server unreachable).
 */
static esp_err_t send_heartbeat(bool *transport_failed)
{
    *transport_failed = false;
    const health_state_t snapshot = snapshot_state();
    char *json = build_heartbeat_json(&snapshot);
    if (!json) {
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_config_t config = {
        .url = APP_SERVER_HEARTBEAT_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = HEARTBEAT_TIMEOUT_MS,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        cJSON_free(json);
        return ESP_ERR_NO_MEM;
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json, (int)strlen(json));
    esp_err_t err = esp_http_client_perform(client);
    int status = 0;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(client);
        if (status != 200) {
            err = ESP_FAIL;
        }
    } else {
        *transport_failed = true;
    }
    esp_http_client_cleanup(client);
    cJSON_free(json);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Heartbeat failed: %s, HTTP %d", esp_err_to_name(err), status);
    }
    return err;
}

static void heartbeat_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_restart_us = 0;

    while (true) {
        const infer_state_t state = snapshot_state().state;
        if (state == INFER_STATE_UPLOADING) {
            /* Leave the SD card and Wi-Fi to the upload. */
            ESP_LOGI(TAG, "Heartbeat paused while uploading");
        } else if (wifi_manager_is_connected()) {
            uint64_t free_bytes = 0;
            if (node_storage_get_free_bytes(&free_bytes) == ESP_OK) {
                s_sd_free_bytes = free_bytes;
            }
            bool transport_failed = false;
            const esp_err_t err = send_heartbeat(&transport_failed);
            uint32_t streak = 0;
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                if (err == ESP_OK || !transport_failed) {
                    s_state.heartbeat_fail_streak = 0;
                } else {
                    ++s_state.heartbeat_fail_streak;
                }
                streak = s_state.heartbeat_fail_streak;
                xSemaphoreGive(s_mutex);
            }
            if (streak >= APP_HEARTBEAT_FAILS_BEFORE_RECONNECT) {
                ESP_LOGW(TAG, "Server unreachable for %u heartbeats; rebuilding Wi-Fi link",
                         (unsigned)streak);
                wifi_manager_force_reconnect();
            }
        } else {
            const uint32_t offline_s = wifi_manager_offline_seconds();
            const int64_t now_us = esp_timer_get_time();
            const bool restart_due =
                last_restart_us == 0 ||
                now_us - last_restart_us >=
                    (int64_t)APP_WIFI_RESTART_AFTER_OFFLINE_S * 1000000LL;
            ESP_LOGW(TAG, "Heartbeat skipped: Wi-Fi offline for %u s", (unsigned)offline_s);
            if (offline_s >= APP_WIFI_RESTART_AFTER_OFFLINE_S && restart_due) {
                wifi_manager_restart();
                last_restart_us = now_us;
            }
        }
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_HEARTBEAT_PERIOD_MS));
    }
}

esp_err_t infer_health_start(void)
{
    if (s_mutex) {
        return ESP_OK;
    }
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(heartbeat_task, "heartbeat", HEARTBEAT_TASK_STACK, NULL,
                    HEARTBEAT_TASK_PRIORITY, NULL) != pdPASS) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void infer_health_set_state(infer_state_t state)
{
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        s_state.state = state;
        xSemaphoreGive(s_mutex);
    }
}

void infer_health_note_upload(const sample_upload_summary_t *summary)
{
    if (!summary || !s_mutex || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return;
    }
    s_state.has_upload = true;
    s_state.last_upload_at = time(NULL);
    s_state.last_upload = *summary;
    xSemaphoreGive(s_mutex);
}
