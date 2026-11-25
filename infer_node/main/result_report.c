/*
 * 把推論結果與時間記錄送給 server。POST /infer/results：
 *   {"device_id":"...","time_ms":...,"dropped":N,
 *    "pending":{"audio":N,"video":N},"records":[{...}, ...]}
 * 佇列的記錄有多少送多少 (上限 BODY_BYTES)，所以平常一筆一筆立刻送，
 * 網路慢時自動變成批次。
 */
#include "result_report.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "device_identity.h"
#include "infer_config.h"
#include "sample_store.h"
#include "wifi_manager.h"

#define QUEUE_DEPTH         64U
#define BODY_BYTES          (8U * 1024U)
#define RECORD_MAX_BYTES    1200U   /* 一筆 JSON 的上限，留空間才繼續加；最長的是 cnn_ops */
#define TASK_STACK_BYTES    6144U
#define TASK_PRIORITY       2U
/*
 * CAM 角色：不綁核心時 Core 1 幾乎一直被 cam_detect (5) 佔著，這個 task 實際上都跑在 Core 0，
 * 搶占優先權 1 的 cam_infer (連續多個 ROI 時每次推論多 30–50 ms)。綁 Core 1 只在 FD 空檔執行。
 * MIC 角色 Core 1 是 audio_cnn (3)，維持不綁。
 */
#if !CONFIG_FREERTOS_UNICORE && INFER_CAM_ENABLE
#define TASK_CORE           1
#else
#define TASK_CORE           tskNO_AFFINITY
#endif
#define HTTP_TIMEOUT_MS     5000
#define RETRY_DELAY_MS      2000U

static const char *TAG = "REPORT";
static QueueHandle_t s_queue = NULL;
static StaticQueue_t s_queue_control;
static char *s_body = NULL;
static volatile uint32_t s_dropped = 0;
static volatile uint32_t s_sent = 0;
static esp_http_client_handle_t s_client = NULL;

void result_report_push(const infer_record_t *record)
{
    if (!s_queue || !record) {
        return;
    }
    if (xQueueSend(s_queue, record, 0) != pdTRUE) {
        ++s_dropped;
    }
}

uint32_t result_report_dropped(void)
{
    return s_dropped;
}

uint32_t result_report_sent(void)
{
    return s_sent;
}

/* 超出容量回傳 false，呼叫端不會再往後寫 */
static bool append(size_t *used, const char *format, ...)
{
    if (*used >= BODY_BYTES) {
        return false;
    }
    va_list args;
    va_start(args, format);
    const int written = vsnprintf(s_body + *used, BODY_BYTES - *used, format, args);
    va_end(args);
    if (written < 0 || (size_t)written >= BODY_BYTES - *used) {
        return false;
    }
    *used += (size_t)written;
    return true;
}

static bool append_probs(size_t *used, const float probs[INFER_CLASS_COUNT])
{
    return append(used, "\"probs\":[%.4f,%.4f,%.4f]", (double)probs[0], (double)probs[1],
                  (double)probs[2]);
}

static bool append_record(size_t *used, const infer_record_t *record)
{
    switch (record->type) {
    case INFER_RECORD_AUDIO: {
        const infer_audio_record_t *r = &record->data.audio;
        return append(used,
                      "{\"t\":\"audio\",\"window\":%" PRIu32 ",\"start_ms\":%" PRId64
                      ",\"end_ms\":%" PRId64 ",\"queue_us\":%" PRIu32 ",\"feature_us\":%" PRIu32
                      ",\"invoke_us\":%" PRIu32 ",\"overflows\":%" PRIu32 ",\"dropped\":%" PRIu32
                      ",\"pred\":%u,\"label\":\"%s\",\"conf\":%.4f,\"low\":%s,",
                      r->window, r->start_epoch_ms, r->end_epoch_ms, r->queue_us, r->feature_us,
                      r->invoke_us, r->overflows, r->dropped_windows, (unsigned)r->pred, r->label,
                      (double)r->conf, r->low_conf ? "true" : "false") &&
               append_probs(used, r->probs) && append(used, "}");
    }
    case INFER_RECORD_CAM_ROI: {
        const infer_cam_roi_record_t *r = &record->data.roi;
        return append(used,
                      "{\"t\":\"roi\",\"session\":%" PRId64 ",\"frame\":%" PRIu32 ",\"ts_ms\":%" PRId64
                      ",\"track\":%" PRId32 ",\"bbox\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32
                      "],\"invoke_us\":%" PRIu32 ",\"result_us\":%" PRIu32
                      ",\"pred\":%u,\"label\":\"%s\",\"conf\":%.4f,\"low\":%s,",
                      r->session, r->frame, r->ts_epoch_ms, r->track_id, r->bbox[0], r->bbox[1],
                      r->bbox[2], r->bbox[3], r->invoke_us, r->result_us, (unsigned)r->pred, r->label,
                      (double)r->conf, r->low_conf ? "true" : "false") &&
               append_probs(used, r->probs) && append(used, "}");
    }
    case INFER_RECORD_CAM_FRAME: {
        const infer_cam_frame_record_t *r = &record->data.frame;
        return append(used,
                      "{\"t\":\"frame\",\"session\":%" PRId64 ",\"frame\":%" PRIu32 ",\"ts_ms\":%" PRId64
                      ",\"capture_us\":%" PRIu32 ",\"queue_us\":%" PRIu32 ",\"downscale_us\":%" PRIu32
                      ",\"fd_us\":%" PRIu32 ",\"fd_diff_us\":%" PRIu32 ",\"fd_morph_us\":%" PRIu32
                      ",\"fd_contour_us\":%" PRIu32 ",\"fd_refine_us\":%" PRIu32 ",\"fd_nms_us\":%" PRIu32
                      ",\"track_us\":%" PRIu32 ",\"detect_us\":%" PRIu32 ",\"cnn_wait_us\":%" PRIu32
                      ",\"roi_us\":%" PRIu32 ",\"crop_us\":%" PRIu32
                      ",\"cnn_us\":%" PRIu32 ",\"total_us\":%" PRIu32
                      ",\"boxes\":%u,\"candidates\":%u}",
                      r->session, r->frame, r->ts_epoch_ms, r->capture_us, r->queue_us,
                      r->downscale_us, r->fd_us, r->fd_diff_us, r->fd_morph_us, r->fd_contour_us,
                      r->fd_refine_us, r->fd_nms_us, r->track_us, r->detect_us, r->cnn_wait_us,
                      r->roi_us, r->crop_us, r->cnn_us,
                      r->total_us, (unsigned)r->boxes, (unsigned)r->candidates);
    }
    case INFER_RECORD_CAM_SESSION: {
        const infer_cam_session_record_t *r = &record->data.session;
        return append(used,
                      "{\"t\":\"session\",\"session\":%" PRId64 ",\"frames\":%" PRIu32
                      ",\"processed\":%" PRIu32 ",\"dropped\":%" PRIu32 ",\"gaps\":%" PRIu32
                      ",\"incomplete\":%" PRIu32 ",\"rois\":%" PRIu32 ",\"rois_expired\":%" PRIu32
                      ",\"low_conf\":%" PRIu32 ",\"saved\":%" PRIu32 ",\"save_skipped\":%" PRIu32
                      ",\"first_frame_offset_us\":%" PRId32 ",\"interval_min_us\":%" PRId32
                      ",\"interval_max_us\":%" PRId32 ",\"error\":%" PRId32
                      ",\"save_wait_avg_us\":%" PRIu32 ",\"save_wait_max_us\":%" PRIu32
                      ",\"save_avg_us\":%" PRIu32 ",\"save_max_us\":%" PRIu32 "}",
                      r->session, r->frames, r->processed, r->dropped, r->gaps, r->incomplete,
                      r->rois, r->rois_expired, r->low_conf, r->saved, r->save_skipped,
                      r->first_frame_offset_us,
                      r->interval_min_us, r->interval_max_us, r->error, r->save_wait_avg_us,
                      r->save_wait_max_us, r->save_avg_us, r->save_max_us);
    }
    case INFER_RECORD_CAM_OPS: {
        const infer_cam_ops_record_t *r = &record->data.ops;
        bool ok = append(used,
                         "{\"t\":\"cnn_ops\",\"session\":%" PRId64 ",\"frame\":%" PRIu32
                         ",\"invokes\":%" PRIu32 ",\"ops\":[",
                         r->session, r->frame, r->invokes);
        for (unsigned i = 0; ok && i < r->op_count; ++i) {
            ok = append(used, "%s[\"%s\",%" PRIu32 "]", i > 0U ? "," : "", r->ops[i].name,
                        r->ops[i].avg_us);
        }
        return ok && append(used, "]}");
    }
    case INFER_RECORD_TASK_USAGE: {
        const infer_task_usage_record_t *r = &record->data.task_usage;
        bool ok = append(used,
                         "{\"t\":\"task_usage\",\"media\":\"%s\",\"ts_ms\":%" PRId64
                         ",\"inferences\":%" PRIu32 ",\"avg_wall_us\":%" PRIu32 ",\"tasks\":[",
                         r->media, r->ts_epoch_ms, r->inferences, r->avg_wall_us);
        for (unsigned i = 0; ok && i < r->task_count; ++i) {
            ok = append(used, "%s[\"%s\",%d,%" PRIu32 "]", i > 0U ? "," : "", r->tasks[i].name,
                        (int)r->tasks[i].core, r->tasks[i].avg_us);
        }
        return ok && append(used, "]}");
    }
    default:
        return false;
    }
}

/* 開頭 + 第一筆一定放得下 (BODY_BYTES 遠大於 RECORD_MAX_BYTES) */
static size_t build_body(const infer_record_t *first)
{
    struct timeval now = {0};
    gettimeofday(&now, NULL);
    const int64_t now_ms = (int64_t)now.tv_sec * 1000LL + now.tv_usec / 1000;

    size_t used = 0;
    bool ok = append(&used,
                     "{\"device_id\":\"%s\",\"time_ms\":%" PRId64 ",\"dropped\":%" PRIu32
                     ",\"pending\":{\"audio\":%u,\"video\":%u},\"records\":[",
                     device_identity_get(), now_ms, s_dropped,
                     (unsigned)sample_store_pending(SAMPLE_MEDIA_AUDIO),
                     (unsigned)sample_store_pending(SAMPLE_MEDIA_VIDEO));
    ok = ok && append_record(&used, first);

    infer_record_t next;
    while (ok && used + RECORD_MAX_BYTES < BODY_BYTES &&
           xQueueReceive(s_queue, &next, 0) == pdTRUE) {
        ok = append(&used, ",") && append_record(&used, &next);
    }
    if (!ok || !append(&used, "]}")) {
        /* 照理不會發生：上面已預留空間；寧可丟這批也不要送壞掉的 JSON */
        ESP_LOGE(TAG, "Record batch does not fit; dropped");
        return 0;
    }
    return used;
}

/* 回傳 true = 這批處理完 (成功，或 server 明確拒收)；false = 稍後重送 */
static bool post_body(size_t bytes)
{
    if (!wifi_manager_is_connected()) {
        return false;
    }
    if (!s_client) {
        const esp_http_client_config_t config = {
            .url = INFER_SERVER_RESULTS_URL,
            .method = HTTP_METHOD_POST,
            .timeout_ms = HTTP_TIMEOUT_MS,
            .keep_alive_enable = true,
        };
        s_client = esp_http_client_init(&config);
        if (!s_client) {
            return false;
        }
        esp_http_client_set_header(s_client, "Content-Type", "application/json");
    }
    esp_http_client_set_post_field(s_client, s_body, (int)bytes);
    const esp_err_t err = esp_http_client_perform(s_client);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Result POST failed: %s", esp_err_to_name(err));
        /* 下次重新連線，不要沿用可能已斷掉的連線 */
        esp_http_client_cleanup(s_client);
        s_client = NULL;
        return false;
    }
    const int status = esp_http_client_get_status_code(s_client);
    if (status == 200) {
        return true;
    }
    if (status >= 400 && status < 500) {
        ESP_LOGE(TAG, "Server rejected result batch: HTTP %d", status);
        return true;
    }
    ESP_LOGW(TAG, "Result POST: HTTP %d", status);
    return false;
}

static void result_report_task(void *arg)
{
    (void)arg;
    infer_record_t first;
    while (true) {
        (void)xQueueReceive(s_queue, &first, portMAX_DELAY);
        const size_t bytes = build_body(&first);
        if (bytes == 0U) {
            continue;
        }
        /* 重送期間新的記錄留在佇列，滿了才丟 (計入 dropped)；影像與音訊都不等這裡 */
        while (!post_body(bytes)) {
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        }
        ++s_sent;
    }
}

esp_err_t result_report_start(void)
{
    if (s_queue) {
        return ESP_OK;
    }
    /* 佇列內容與 body 都只有 task 存取，放 PSRAM 省內部 RAM */
    uint8_t *storage = heap_caps_malloc(QUEUE_DEPTH * sizeof(infer_record_t),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_body = heap_caps_malloc(BODY_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!storage || !s_body) {
        return ESP_ERR_NO_MEM;
    }
    s_queue = xQueueCreateStatic(QUEUE_DEPTH, sizeof(infer_record_t), storage, &s_queue_control);
    if (!s_queue) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(result_report_task, "result_report", TASK_STACK_BYTES, NULL,
                                TASK_PRIORITY, NULL, TASK_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
