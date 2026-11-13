/*
 * 推論韌體開機流程與背景迴圈 (ESP32 / TTGO T8 版，由 DID_NODE/infer_node/main/infer_app.c 移植)。
 *
 * 一塊板子只跑一條管線，相機或麥克風 (infer_config.h 的 INFER_NODE_ROLE)，並連續運作。
 * 這裡的迴圈只負責把累積滿的低信賴樣本上傳，並定期校時；上傳時擷取與推論照常進行。
 * 與 S3 版的差別：TTGO T8 沒有 RGB LED (拿掉 status_led)，Device ID 前綴依角色。
 */
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "cam_infer.h"
#include "device_identity.h"
#include "infer_config.h"
#include "infer_health.h"
#include "mic_infer.h"
#include "node_storage.h"
#include "result_report.h"
#include "sample_store.h"
#include "sample_upload.h"
#include "time_sync.h"
#include "wifi_manager.h"

static const char *TAG = "INFER_APP";

#define CAMERA_INIT_TASK_STACK_BYTES 8192U
#define CAMERA_INIT_TASK_PRIORITY    6U
#define NO_PIPELINE_RESTART_MS       30000U
#define WAIT_CHUNK_MS                1000U

/* 連續運作時 cam_writer 與上傳會同時進行，所以把共用 I/O 緩衝切成兩半 */
_Static_assert(CAM_INFER_STAGING_BYTES + SAMPLE_UPLOAD_BUFFER_BYTES <= NODE_STORAGE_IO_BUFFER_BYTES,
               "Camera staging and upload buffers must fit in the shared I/O buffer");

#if INFER_CAM_ENABLE
typedef struct {
    TaskHandle_t caller;
    esp_err_t result;
} camera_init_context_t;

static void camera_init_on_core1(void *arg)
{
    camera_init_context_t *context = (camera_init_context_t *)arg;
    context->result = cam_infer_init();
    xTaskNotifyGive(context->caller);
    vTaskDelete(NULL);
}

static esp_err_t init_camera_pipeline(void)
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
#endif /* INFER_CAM_ENABLE */

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

static void log_heap(const char *stage)
{
    ESP_LOGI(TAG, "HEAP [%s] internal free=%u largest=%u min=%u | PSRAM free=%u largest=%u",
             stage,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
}

/*
 * 檔名與排程都需要實際時間，開機等 Wi-Fi 與一次校時成功。
 * 時鐘看起來有效時也要做：軟體重置後會保留系統時間。
 */
static void synchronize_clock(void)
{
    while (true) {
        while (wifi_manager_wait_connected(pdMS_TO_TICKS(15000)) != ESP_OK) {
            ESP_LOGW(TAG, "Waiting for Wi-Fi...");
        }
        esp_err_t err = time_sync_from_server();
        if (err == ESP_OK) {
            return;
        }
        ESP_LOGW(TAG, "Clock sync failed: %s; retrying", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(APP_UPLOAD_RETRY_MS));
    }
}

static bool upload_media(sample_media_t media)
{
    sample_upload_summary_t summary = {0};
    const esp_err_t err = sample_upload_pending(media, &summary);
    infer_health_note_upload(&summary);
    ESP_LOGI(TAG, "Upload %s: ok=%u failed=%u rejected=%u left=%u gap_wait=%u ms gap_timeouts=%u",
             sample_store_media_name(media), (unsigned)summary.uploaded_count,
             (unsigned)summary.failed_count, (unsigned)summary.rejected_count,
             (unsigned)sample_store_pending(media), (unsigned)summary.gap_wait_ms,
             (unsigned)summary.gap_timeouts);
    return err == ESP_OK;
}

/* 累積滿 INFER_UPLOAD_AFTER_SAMPLES 的 media 才上傳；回傳 false = 失敗，稍後再試 */
static bool upload_ready_samples(void)
{
    if (!sample_store_any_ready(INFER_UPLOAD_AFTER_SAMPLES)) {
        return true;
    }
    if (!wifi_manager_is_connected()) {
        ESP_LOGW(TAG, "Upload deferred: Wi-Fi unavailable");
        return false;
    }
    infer_health_set_state(INFER_STATE_UPLOADING);
    log_heap("before upload");
    bool ok = true;
    for (int media = 0; media < SAMPLE_MEDIA_COUNT; ++media) {
        if (sample_store_pending((sample_media_t)media) >= INFER_UPLOAD_AFTER_SAMPLES) {
            ok = upload_media((sample_media_t)media) && ok;
        }
    }
    log_heap("after upload");
    infer_health_set_state(INFER_STATE_RUNNING);
    return ok;
}

static void resync_clock_if_due(int64_t *last_sync_us)
{
    const int64_t now_us = esp_timer_get_time();
    if (now_us - *last_sync_us < (int64_t)INFER_CLOCK_RESYNC_SEC * 1000000LL ||
        !wifi_manager_is_connected()) {
        return;
    }
    *last_sync_us = now_us;
    if (time_sync_from_server() != ESP_OK) {
        ESP_LOGW(TAG, "Clock resync failed; keeping local time");
    }
}

/* 樣本滿了就上傳，並定期校時；不會返回 */
static void run_background_loop(int64_t last_sync_us)
{
    int64_t retry_after_us = 0;
    while (true) {
        if (esp_timer_get_time() >= retry_after_us && !upload_ready_samples()) {
            retry_after_us = esp_timer_get_time() + (int64_t)APP_UPLOAD_RETRY_MS * 1000LL;
        }
        resync_clock_if_due(&last_sync_us);
        sample_store_wait_change(pdMS_TO_TICKS(WAIT_CHUNK_MS));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_nvs());
    ESP_ERROR_CHECK(device_identity_init(INFER_DEVICE_ID_PREFIX));
    ESP_LOGI(TAG, "Device ID: %s, role: %s", device_identity_get(),
             INFER_CAM_ENABLE ? "camera" : "microphone");
    time_sync_configure_timezone();

    log_heap("app start");
    ESP_ERROR_CHECK(wifi_manager_init());
    synchronize_clock();
    int64_t last_sync_us = esp_timer_get_time();
    log_heap("wifi connected");

    /*
     * SD is mounted after the camera powers up: on the breadboard build every
     * SD command timed out from the moment the camera powered on (PWDN low +
     * XCLK start), which looks like the card being reset by a supply glitch.
     */
#if INFER_MIC_ENABLE
    const esp_err_t mic_err = mic_infer_init();
    if (mic_err != ESP_OK) {
        ESP_LOGE(TAG, "Microphone unavailable: %s", esp_err_to_name(mic_err));
    }
#else
    const esp_err_t mic_err = ESP_ERR_NOT_SUPPORTED;
#endif
    log_heap("mic init");

#if INFER_CAM_ENABLE
    esp_err_t cam_err = init_camera_pipeline();
    if (cam_err == ESP_OK) {
        /* 開機測試後到 SD 與回報都準備好之前，不讓相機持續收集未使用的影格。 */
        cam_err = cam_infer_pause();
    }
    if (cam_err != ESP_OK) {
        ESP_LOGE(TAG, "Camera unavailable: %s", esp_err_to_name(cam_err));
    }
#else
    const esp_err_t cam_err = ESP_ERR_NOT_SUPPORTED;
#endif
    log_heap("camera init");

    if (mic_err != ESP_OK && cam_err != ESP_OK) {
        ESP_LOGE(TAG, "No pipeline available; restarting in %u s",
                 (unsigned)(NO_PIPELINE_RESTART_MS / 1000U));
        vTaskDelay(pdMS_TO_TICKS(NO_PIPELINE_RESTART_MS));
        esp_restart();
    }

    ESP_ERROR_CHECK(node_storage_init());
    ESP_ERROR_CHECK(sample_store_init());
    uint8_t *io_buffer = node_storage_io_buffer_acquire("infer");
    if (!io_buffer) {
        ESP_LOGE(TAG, "Shared I/O buffer unavailable");
        esp_restart();
    }
    /* 麥克風每 2 s 有空檔，上傳讀 SD 時避開推論與寫 WAV；相機連續擷取，不等 */
#if INFER_MIC_ENABLE
    const sample_upload_gap_fn upload_gap = mic_err == ESP_OK ? mic_infer_wait_gap : NULL;
#else
    const sample_upload_gap_fn upload_gap = NULL;
#endif
    ESP_ERROR_CHECK(sample_upload_init(io_buffer + CAM_INFER_STAGING_BYTES, upload_gap));
    ESP_ERROR_CHECK(result_report_start());
    ESP_ERROR_CHECK(infer_health_start());
    log_heap("storage + report");

    if (mic_err == ESP_OK) {
        ESP_ERROR_CHECK(mic_infer_start());
    }
    if (cam_err == ESP_OK) {
        ESP_ERROR_CHECK(cam_infer_start(io_buffer));
    }
    infer_health_set_state(INFER_STATE_RUNNING);
    log_heap("running");

    run_background_loop(last_sync_us);
}
