#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "cam_health.h"
#include "cam_record.h"
#include "cam_storage.h"
#include "cam_upload.h"
#include "device_identity.h"
#include "time_sync.h"
#include "wifi_manager.h"

static const char *TAG = "CAM_APP";

#define CAMERA_INIT_TASK_STACK_BYTES 8192U
#define CAMERA_INIT_TASK_PRIORITY    6U

typedef struct {
    TaskHandle_t caller;
    esp_err_t result;
} camera_init_context_t;

static void camera_init_on_core1(void *arg)
{
    camera_init_context_t *context = (camera_init_context_t *)arg;
    context->result = cam_record_init();
    xTaskNotifyGive(context->caller);
    vTaskDelete(NULL);
}

static esp_err_t init_camera_recorder(void)
{
    camera_init_context_t context = {
        .caller = xTaskGetCurrentTaskHandle(),
        .result = ESP_FAIL,
    };
    /* app_main 保持在 Core 0；感測器配置、暖機與 FPS 測試在 Core 1 執行。 */
#if CONFIG_FREERTOS_UNICORE
    const BaseType_t camera_core = 0;
#else
    const BaseType_t camera_core = 1;
#endif
    if (xTaskCreatePinnedToCore(camera_init_on_core1, "camera_init",
                                CAMERA_INIT_TASK_STACK_BYTES, &context,
                                CAMERA_INIT_TASK_PRIORITY, NULL,
                                camera_core) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return context.result;
}

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

static void wait_for_wifi(void)
{
    while (wifi_manager_wait_connected(pdMS_TO_TICKS(15000)) != ESP_OK) {
        ESP_LOGW(TAG, "Waiting for Wi-Fi...");
    }
}

static void synchronize_clock(void)
{
    while (!time_sync_is_valid()) {
        wait_for_wifi();
        esp_err_t err = time_sync_from_server();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Clock sync failed: %s; retrying",
                     esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(APP_UPLOAD_RETRY_MS));
        }
    }
}

static void upload_boot_backlog(void)
{
    while (true) {
        size_t pending = 0;
        esp_err_t err = cam_storage_count_done(&pending);
        if (err == ESP_OK && pending == 0) {
            ESP_LOGI(TAG, "Boot camera backlog is empty");
            return;
        }

        ESP_LOGI(TAG, "Boot camera backlog: %u file(s)", (unsigned)pending);
        wait_for_wifi();
        cam_upload_summary_t summary = {0};
        err = cam_upload_process_pending(&summary);
        cam_health_note_upload(&summary);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Backlog upload failed: %s; retrying",
                     esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(APP_UPLOAD_RETRY_MS));
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_nvs());
    ESP_ERROR_CHECK(device_identity_init(DEVICE_ID_CAMERA_PREFIX));
    ESP_LOGI(TAG, "Device ID: %s", device_identity_get());
    ESP_ERROR_CHECK(wifi_manager_init());
    time_sync_configure_timezone();
    synchronize_clock();
    ESP_ERROR_CHECK(cam_storage_init());
    ESP_ERROR_CHECK(cam_health_monitor_start());

    upload_boot_backlog();
    ESP_ERROR_CHECK(init_camera_recorder());
    /* 開機測試後到第一輪錄影前，不讓相機持續收集未使用的影格。 */
    ESP_ERROR_CHECK(cam_record_pause());
    ESP_LOGI(TAG, "Automatic camera schedule started: %u seconds every %u seconds",
             (unsigned)CAM_RECORD_DURATION_SEC,
             (unsigned)(CAM_RECORD_PERIOD_MS / 1000U));

    while (true) {
        TickType_t cycle_started = xTaskGetTickCount();

        cam_record_result_t record_result = {0};
        esp_err_t err = cam_record_once(&record_result);
        /* 早期失敗路徑也要停相機；正常路徑已在 cam_record_once() 停止。 */
        ESP_ERROR_CHECK(cam_record_pause());
        cam_health_note_recording(&record_result);
        if (err == ESP_OK && record_result.success) {
            ESP_LOGI(TAG, "Camera recording saved: %s, frames=%u, bytes=%u",
                     record_result.file_path,
                     (unsigned)record_result.saved_frames,
                     (unsigned)record_result.file_bytes);
        } else {
            ESP_LOGE(TAG, "Camera recording failed: %s",
                     esp_err_to_name(record_result.error));
        }

        if (wifi_manager_wait_connected(pdMS_TO_TICKS(15000)) == ESP_OK) {
            cam_upload_summary_t summary = {0};
            err = cam_upload_process_pending(&summary);
            cam_health_note_upload(&summary);
            ESP_LOGI(TAG, "Camera upload pass: ok=%u failed=%u",
                     (unsigned)summary.uploaded_count,
                     (unsigned)summary.failed_count);
        } else {
            ESP_LOGW(TAG, "Camera upload deferred: Wi-Fi unavailable");
        }

        TickType_t elapsed = xTaskGetTickCount() - cycle_started;
        TickType_t period = pdMS_TO_TICKS(CAM_RECORD_PERIOD_MS);
        if (elapsed < period) {
            vTaskDelay(period - elapsed);
        } else {
            ESP_LOGW(TAG, "Camera cycle exceeded its period; next capture starts now");
        }
    }
}
