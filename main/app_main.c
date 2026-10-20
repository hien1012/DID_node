#include <stdbool.h>
#include <sys/time.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "cam_record.h"
#include "cam_stream.h"
#include "device_identity.h"
#include "mic_record.h"
#include "net_bench.h"
#include "node_health.h"
#include "node_storage.h"
#include "node_upload.h"
#include "rec_session.h"
#include "status_led.h"
#include "time_sync.h"
#include "timing_trace.h"
#include "wifi_manager.h"

static const char *TAG = "NODE_APP";

#define CAMERA_INIT_TASK_STACK_BYTES 8192U
#define CAMERA_INIT_TASK_PRIORITY    6U
/* Stop starting uploads this long before the next session's preparation. */
#define UPLOAD_GUARD_SEC             10U
#define NO_RECORDER_RESTART_MS       30000U

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

/*
 * File names and the schedule need real time, so boot waits for Wi-Fi and one
 * successful sync with the server. This runs even when the clock already looks
 * valid: the S3 keeps its system time across a software reset, which used to
 * skip both the Wi-Fi wait and the resync.
 */
static void synchronize_clock(void)
{
    while (true) {
        wait_for_wifi();
        esp_err_t err = time_sync_from_server();
        if (err == ESP_OK) {
            return;
        }
        ESP_LOGW(TAG, "Clock sync failed: %s; retrying", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(APP_UPLOAD_RETRY_MS));
    }
}

/*
 * The first session starts as soon as its files can be prepared after boot;
 * later sessions start one period after the previous start. If a cycle
 * overran its period, the next session starts as soon as possible.
 */
static time_t next_session_start(time_t previous_start, time_t lead_sec)
{
    const time_t earliest = time(NULL) + lead_sec;
    if (previous_start == 0) {
        return earliest;
    }
    const time_t scheduled = previous_start + (time_t)APP_RECORD_PERIOD_SEC;
    if (scheduled < earliest) {
        ESP_LOGW(TAG, "Previous cycle overran its period; starting now");
        return earliest;
    }
    return scheduled;
}

static void sleep_until(time_t epoch, uint32_t lead_ms)
{
    struct timeval now = {0};
    gettimeofday(&now, NULL);
    const int64_t now_ms = (int64_t)now.tv_sec * 1000LL + now.tv_usec / 1000;
    const int64_t wait_ms = (int64_t)epoch * 1000LL - (int64_t)lead_ms - now_ms;
    if (wait_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }
}

static void upload_pass(time_t deadline_epoch)
{
    if (!wifi_manager_is_connected()) {
        ESP_LOGW(TAG, "Upload deferred: Wi-Fi unavailable");
        return;
    }
    node_health_set_state(NODE_STATE_UPLOADING);
    net_bench_log_heap("before upload");
    node_upload_summary_t summary = {0};
    timing_trace_mark(TT_MARK_UPLOAD_START);
    (void)node_upload_process_pending(deadline_epoch, &summary);
    timing_trace_mark(TT_MARK_UPLOAD_END);
    node_health_note_upload(&summary);
    net_bench_log_heap("after upload");   /* min = 上傳期間內部 RAM 的最低點 */
    ESP_LOGI(TAG, "Upload pass: ok=%u failed=%u rejected=%u deferred=%s",
             (unsigned)summary.uploaded_count, (unsigned)summary.failed_count,
             (unsigned)summary.rejected_count, summary.deferred ? "yes" : "no");

    /* Keep the schedule aligned with the server clock. */
    if (wifi_manager_is_connected() && time_sync_from_server() != ESP_OK) {
        ESP_LOGW(TAG, "Clock resync failed; keeping local time");
    }
    timing_trace_mark(TT_MARK_RESYNC_DONE);
    node_health_set_state(NODE_STATE_IDLE);
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_nvs());
    (void)status_led_off();
    ESP_ERROR_CHECK(device_identity_init(APP_DEVICE_ID_PREFIX));
    ESP_LOGI(TAG, "Device ID: %s", device_identity_get());
    time_sync_configure_timezone();

    net_bench_log_heap("app start");
    ESP_ERROR_CHECK(wifi_manager_init());
    synchronize_clock();
    net_bench_log_heap("wifi connected");
    /* Debug: Wi-Fi/lwIP throughput before the camera takes its memory. */
    (void)net_bench_run("boot");

    /*
     * SD is mounted after the camera powers up: on the breadboard build every
     * SD command timed out from the moment the camera powered on (PWDN low +
     * XCLK start), which looks like the card being reset by a supply glitch.
     */
#if APP_MIC_ENABLE
    const esp_err_t mic_err = mic_record_init();
    if (mic_err != ESP_OK) {
        ESP_LOGE(TAG, "Microphone unavailable: %s", esp_err_to_name(mic_err));
    }
#else
    /* Debug: I2S 完全不啟動，排除 mic 時脈對相機影像的影響 (debug_sdcard #31) */
    const esp_err_t mic_err = ESP_ERR_NOT_SUPPORTED;
    ESP_LOGW(TAG, "Microphone disabled (APP_MIC_ENABLE=0): recording video only");
#endif
    net_bench_log_heap("mic init");
    esp_err_t cam_err = init_camera_recorder();
    net_bench_log_heap("camera init");
    if (cam_err == ESP_OK) {
        /* 開機測試後到第一輪錄影前，不讓相機持續收集未使用的影格。 */
        cam_err = cam_record_pause();
    }
    if (cam_err != ESP_OK) {
        ESP_LOGE(TAG, "Camera unavailable: %s", esp_err_to_name(cam_err));
    }
    if (mic_err != ESP_OK && cam_err != ESP_OK) {
        ESP_LOGE(TAG, "No recorder available; restarting in %u s",
                 (unsigned)(NO_RECORDER_RESTART_MS / 1000U));
        vTaskDelay(pdMS_TO_TICKS(NO_RECORDER_RESTART_MS));
        esp_restart();
    }
    ESP_ERROR_CHECK(node_storage_init());
    net_bench_log_heap("sd mounted");
    /* Debug: same test with camera, mic and SD initialized (upload conditions). */
    (void)net_bench_run("after_init");
    /* Debug: A/B camera interference on Wi-Fi. Stopping XCLK also stops the
     * OV7670's PCLK and data lines; the third run checks the result repeats. */
    if (cam_err == ESP_OK && cam_record_set_xclk_output(false) == ESP_OK) {
        (void)net_bench_run("xclk off");
        (void)cam_record_set_xclk_output(true);
        (void)net_bench_run("xclk on again");
    }
#if APP_CAM_STREAM_ENABLE
    if (cam_err == ESP_OK) {
        /* Debug: 只串流不錄影 (cam_stream.c)；只有相機無法恢復時才會返回 */
        const esp_err_t stream_err = cam_stream_run();
        ESP_LOGE(TAG, "Camera stream stopped: %s", esp_err_to_name(stream_err));
    }
#endif
    if (cam_err == ESP_OK) {
        /* 不錄影時 XCLK 保持停止；rec_session 在每次準備錄影時恢復 */
        (void)cam_record_set_xclk_output(false);
    }
    ESP_ERROR_CHECK(node_health_start());
    rec_session_init(cam_err == ESP_OK, mic_err == ESP_OK);
    node_health_set_state(NODE_STATE_IDLE);

    const time_t lead_sec = (time_t)((APP_PREPARE_LEAD_MS + 999U) / 1000U);

    /* Files left from before the reboot are uploaded after the first session. */
    time_t start_epoch = 0;
    while (true) {
        start_epoch = next_session_start(start_epoch, lead_sec);
        struct tm local_start = {0};
        localtime_r(&start_epoch, &local_start);
        ESP_LOGI(TAG, "Next session at %02d:%02d:%02d (%u s, every %u s)",
                 local_start.tm_hour, local_start.tm_min, local_start.tm_sec,
                 (unsigned)APP_RECORD_DURATION_SEC, (unsigned)APP_RECORD_PERIOD_SEC);
        sleep_until(start_epoch, APP_PREPARE_LEAD_MS);
        timing_trace_begin();
        timing_trace_mark(TT_MARK_WAKE);

        node_health_set_state(NODE_STATE_RECORDING);
        rec_session_result_t result;
        (void)rec_session_run(start_epoch, &result);
        node_upload_note_session_offsets(start_epoch,
                                         result.camera.first_frame_offset_us,
                                         result.mic.first_sample_offset_us);
        node_health_note_session(&result);
        node_health_set_state(NODE_STATE_IDLE);

        upload_pass(start_epoch + (time_t)APP_RECORD_PERIOD_SEC - lead_sec -
                    UPLOAD_GUARD_SEC);
        timing_trace_print();   /* 本輪結束：一次印出整輪時序 (APP_TIMING_TRACE) */
    }
}
