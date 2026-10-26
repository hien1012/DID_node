#include "rec_session.h"

#include <inttypes.h>
#include <string.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "la_probe.h"
#include "timing_trace.h"

/* Minimum distance between start() and the trigger, so both capture tasks are
 * running and the camera has resumed before the window opens. */
#define SESSION_START_MARGIN_US  300000LL

static const char *TAG = "REC_SESSION";
static bool s_camera_ready = false;
static bool s_mic_ready = false;

void rec_session_init(bool camera_ready, bool mic_ready)
{
    s_camera_ready = camera_ready;
    s_mic_ready = mic_ready;
}

static int64_t wall_clock_us(void)
{
    struct timeval now = {0};
    gettimeofday(&now, NULL);
    return (int64_t)now.tv_sec * 1000000LL + now.tv_usec;
}

esp_err_t rec_session_run(time_t start_epoch, rec_session_result_t *result)
{
    if (!result) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(result, 0, sizeof(*result));
    result->started_at = start_epoch;
    result->camera_enabled = s_camera_ready;
    result->mic_enabled = s_mic_ready;
    if (!s_camera_ready && !s_mic_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 1. 建立並預先配置兩個檔案 (camera 約 92 MB，需數秒) */
    const int64_t prepare_started_us = esp_timer_get_time();
    bool camera_prepared = false;
    bool mic_prepared = false;
    if (s_camera_ready) {
        /* 錄影之間 XCLK 是停止的；提早在準備階段恢復，讓感測器在觸發點前穩定。
         * mic 的 I2S 時脈由 mic_record_prepare/finish 自行開關。 */
        (void)cam_record_set_xclk_output(true);
        timing_trace_mark(TT_MARK_XCLK_ON);
        result->camera_prepare_error = cam_record_prepare(start_epoch);
        camera_prepared = result->camera_prepare_error == ESP_OK;
        timing_trace_mark(TT_MARK_CAM_PREPARED);
    }
    if (s_mic_ready) {
        result->mic_prepare_error = mic_record_prepare(start_epoch);
        mic_prepared = result->mic_prepare_error == ESP_OK;
        timing_trace_mark(TT_MARK_MIC_PREPARED);
    }
    ESP_LOGI(TAG, "Prepared in %" PRId64 " ms: camera=%s mic=%s",
             (esp_timer_get_time() - prepare_started_us) / 1000,
             camera_prepared ? "ok" : esp_err_to_name(result->camera_prepare_error),
             mic_prepared ? "ok" : esp_err_to_name(result->mic_prepare_error));

    /* 2. 把排程時間 (實際時間) 換算成 esp_timer 觸發點 */
    const int64_t now_us = esp_timer_get_time();
    int64_t trigger_us = now_us + ((int64_t)start_epoch * 1000000LL - wall_clock_us());
    if (trigger_us < now_us + SESSION_START_MARGIN_US) {
        result->start_late_ms = (now_us + SESSION_START_MARGIN_US - trigger_us) / 1000;
        trigger_us = now_us + SESSION_START_MARGIN_US;
        ESP_LOGW(TAG, "Session starts %" PRId64 " ms late", result->start_late_ms);
    }

    /* 3. 觸發點前 SESSION_START_MARGIN_US 才啟動任務，兩者共用同一個 trigger_us */
    const int64_t wait_us = trigger_us - SESSION_START_MARGIN_US - esp_timer_get_time();
    if (wait_us > 0) {
        vTaskDelay(pdMS_TO_TICKS(wait_us / 1000));
    }
    timing_trace_mark(TT_MARK_WAIT_DONE);
    timing_trace_mark_at(TT_MARK_TRIGGER, trigger_us);
    if (camera_prepared) {
        esp_err_t err = cam_record_start(trigger_us);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Camera start failed: %s", esp_err_to_name(err));
        }
        timing_trace_mark(TT_MARK_CAM_STARTED);
    }
    if (mic_prepared) {
        esp_err_t err = mic_record_start(trigger_us);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Mic start failed: %s", esp_err_to_name(err));
        }
        timing_trace_mark(TT_MARK_MIC_STARTED);
    }
    /* debug：LA task 腳改看錄製的 4 個 task (此時都已建立) */
    la_probe_set_phase(LA_PHASE_RECORD);

    /* 4. 等待兩邊結束，各自檢核後移到 done/ 或 broken/ */
    if (camera_prepared) {
        (void)cam_record_finish(&result->camera);
        timing_trace_mark(TT_MARK_CAM_FINISHED);
    } else {
        result->camera.error = s_camera_ready ? result->camera_prepare_error
                                              : ESP_ERR_INVALID_STATE;
    }
    if (s_camera_ready) {
        /* 相機已暫停：停止 XCLK 直到下一次錄影，避免干擾接下來的上傳 */
        (void)cam_record_set_xclk_output(false);
        timing_trace_mark(TT_MARK_XCLK_OFF);
    }
    if (mic_prepared) {
        (void)mic_record_finish(&result->mic);
        timing_trace_mark(TT_MARK_MIC_FINISHED);
    } else {
        result->mic.error = s_mic_ready ? result->mic_prepare_error
                                        : ESP_ERR_INVALID_STATE;
    }
    /* 錄製 task 已自刪：解除綁定，避免其 TCB 記憶體被重用時誤判 */
    la_probe_set_phase(LA_PHASE_NONE);

    ESP_LOGI(TAG, "Session %lld done: camera=%s (%u frames, first +%" PRId64
             " ms) mic=%s (%u samples, first +%" PRId64 " ms)",
             (long long)start_epoch,
             result->camera.success ? "ok" : "broken",
             (unsigned)result->camera.saved_frames,
             result->camera.first_frame_offset_us / 1000,
             result->mic.success ? "ok" : "broken",
             (unsigned)result->mic.samples,
             result->mic.first_sample_offset_us / 1000);

    return (result->camera.success || result->mic.success) ? ESP_OK : ESP_FAIL;
}
