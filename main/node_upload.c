#include "node_upload.h"

#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "cJSON.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_config.h"
#include "cam_record.h"
#include "device_identity.h"
#include "mic_record.h"
#include "node_storage.h"
#include "timing_trace.h"
#include "wifi_manager.h"

#define UPLOAD_BUFFER_SIZE      (16U * 1024U)
#define UPLOAD_BUFFER_COUNT     2U
#define READER_STACK_BYTES      4096U
#define READER_PRIORITY         2U     /* main task 是 1 */
#define ACK_BUFFER_SIZE         512U
#define ID_TEXT_SIZE            64U
/* Starting estimate for the deadline check; replaced by measured throughput. */
#define UPLOAD_INITIAL_RATE_BPS (1000U * 1000U)
#define UPLOAD_MIN_RATE_BPS     (100U * 1000U)

/*
 * 讀 SD 與 Wi-Fi 送出重疊 (debug_sdcard #23：兩者輪流時各約 4 MB/s，合計只有
 * 2.1 MB/s)。upload_reader task 讀下一塊時，main task 送出上一塊：
 *   s_free_queue (空緩衝 index) → reader read() → s_full_queue → main 送出 → s_free_queue
 * reader 最後送一個 length <= 0 的結束訊息 (0 = 讀完，< 0 = 讀取錯誤或中止)。
 */
typedef struct {
    uint8_t index;
    int32_t length;
} upload_chunk_t;

typedef struct {
    int fd;
    size_t file_size;
    volatile bool abort;        /* main 送出失敗時設定，reader 停止讀取 */
    int64_t sd_read_us;         /* reader 寫入；main 收到結束訊息後才讀 */
    int64_t max_sd_read_us;
} upload_reader_job_t;

/* 兩塊讀檔緩衝切自 node_storage 的共用 I/O 緩衝 (與相機錄影暫存區共用)。
 * 需 4-byte 對齊且位於內部 RAM：否則 sdmmc driver 會改用 512 B 暫存區逐 sector 讀取 */
_Static_assert(UPLOAD_BUFFER_COUNT * UPLOAD_BUFFER_SIZE <= NODE_STORAGE_IO_BUFFER_BYTES,
               "Upload buffers must fit in the shared I/O buffer");

static const char *TAG = "UPLOAD";
static uint8_t *s_io_buffer = NULL;      /* 上傳一個檔案期間持有 */
static uint32_t s_rate_bps = UPLOAD_INITIAL_RATE_BPS;

static upload_reader_job_t s_job;
static TaskHandle_t s_reader_task = NULL;
static StaticTask_t s_reader_tcb;
static StackType_t s_reader_stack[READER_STACK_BYTES];
static QueueHandle_t s_free_queue = NULL;
static QueueHandle_t s_full_queue = NULL;
static StaticQueue_t s_free_queue_storage;
static StaticQueue_t s_full_queue_storage;
static uint8_t s_free_queue_items[UPLOAD_BUFFER_COUNT * sizeof(uint8_t)];
static uint8_t s_full_queue_items[(UPLOAD_BUFFER_COUNT + 1U) * sizeof(upload_chunk_t)];

static uint32_t s_file_index = 0;       /* timing trace：本輪上傳第幾個檔案 */

static time_t s_offsets_epoch = 0;
static int64_t s_camera_offset_us = 0;
static int64_t s_mic_offset_us = 0;

void node_upload_note_session_offsets(time_t session_epoch,
                                      int64_t camera_offset_us,
                                      int64_t mic_offset_us)
{
    s_offsets_epoch = session_epoch;
    s_camera_offset_us = camera_offset_us;
    s_mic_offset_us = mic_offset_us;
}

static esp_err_t verify_ack(const char *json, const char *capture_id,
                            size_t expected_size)
{
    cJSON *root = cJSON_Parse(json);
    cJSON *status = root ? cJSON_GetObjectItemCaseSensitive(root, "status") : NULL;
    cJSON *ack_id = root ? cJSON_GetObjectItemCaseSensitive(root, "capture_id") : NULL;
    cJSON *bytes = root ? cJSON_GetObjectItemCaseSensitive(root, "bytes") : NULL;
    bool valid = cJSON_IsString(status) &&
                 strcmp(status->valuestring, "ok") == 0 &&
                 cJSON_IsString(ack_id) &&
                 strcmp(ack_id->valuestring, capture_id) == 0 &&
                 cJSON_IsNumber(bytes) &&
                 (size_t)bytes->valuedouble == expected_size;
    cJSON_Delete(root);
    return valid ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

/* 4xx except timeout/too-many-requests: retrying the same file cannot succeed. */
static bool is_permanent_rejection(int status)
{
    return status >= 400 && status < 500 && status != 408 && status != 429;
}

static void set_media_headers(esp_http_client_handle_t client,
                              const node_pending_file_t *pending)
{
    static char frame_count_text[16];
    static char width_text[16];
    static char height_text[16];
    static char fps_text[16];
    static char sample_rate_text[16];

    if (pending->media == NODE_MEDIA_VIDEO) {
        snprintf(frame_count_text, sizeof(frame_count_text), "%u",
                 (unsigned)(pending->file_bytes / CAM_IMAGE_FRAME_BYTES));
        snprintf(width_text, sizeof(width_text), "%u", (unsigned)CAM_IMAGE_WIDTH);
        snprintf(height_text, sizeof(height_text), "%u", (unsigned)CAM_IMAGE_HEIGHT);
        snprintf(fps_text, sizeof(fps_text), "%u", (unsigned)CAM_TARGET_FPS);
        esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
        esp_http_client_set_header(client, "X-Frame-Count", frame_count_text);
        esp_http_client_set_header(client, "X-Width", width_text);
        esp_http_client_set_header(client, "X-Height", height_text);
        esp_http_client_set_header(client, "X-FPS", fps_text);
        esp_http_client_set_header(client, "X-Format", "GRAY8");
    } else {
        snprintf(sample_rate_text, sizeof(sample_rate_text), "%u",
                 (unsigned)MIC_SAMPLE_RATE);
        esp_http_client_set_header(client, "Content-Type", "audio/wav");
        esp_http_client_set_header(client, "X-Sample-Rate", sample_rate_text);
    }
}

static void upload_reader_task(void *arg)
{
    (void)arg;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        size_t remaining = s_job.file_size;
        upload_chunk_t chunk = {0};
        int32_t end_code = 0;
        while (remaining > 0) {
            (void)xQueueReceive(s_free_queue, &chunk.index, portMAX_DELAY);
            if (s_job.abort) {
                end_code = -1;
                break;
            }
            const size_t want = remaining < UPLOAD_BUFFER_SIZE ? remaining
                                                               : UPLOAD_BUFFER_SIZE;
            const int64_t read_start_us = esp_timer_get_time();
            uint8_t *buffer = s_io_buffer + (size_t)chunk.index * UPLOAD_BUFFER_SIZE;
            const ssize_t got = read(s_job.fd, buffer, want);
            const int64_t read_us = esp_timer_get_time() - read_start_us;
            s_job.sd_read_us += read_us;
            if (read_us > s_job.max_sd_read_us) {
                s_job.max_sd_read_us = read_us;
            }
            if (got <= 0) {   /* 0 = 檔案比 file_bytes 短，-1 = I/O 錯誤 */
                end_code = -1;
                break;
            }
            chunk.length = (int32_t)got;
            remaining -= (size_t)got;
            (void)xQueueSend(s_full_queue, &chunk, portMAX_DELAY);
        }
        /* 結束訊息之後 reader 不再碰 fd 與緩衝，main 才能 close(fd) */
        chunk.length = end_code;
        (void)xQueueSend(s_full_queue, &chunk, portMAX_DELAY);
    }
}

static esp_err_t ensure_reader_task(void)
{
    if (s_reader_task) {
        return ESP_OK;
    }
    s_free_queue = xQueueCreateStatic(UPLOAD_BUFFER_COUNT, sizeof(uint8_t),
                                      s_free_queue_items, &s_free_queue_storage);
    s_full_queue = xQueueCreateStatic(UPLOAD_BUFFER_COUNT + 1U, sizeof(upload_chunk_t),
                                      s_full_queue_items, &s_full_queue_storage);
    /* 上傳期間相機已暫停，Core 1 空閒；main (HTTP) 留在 Core 0 */
#if CONFIG_FREERTOS_UNICORE
    const BaseType_t reader_core = 0;
#else
    const BaseType_t reader_core = 1;
#endif
    /* 常駐 task：static task 自刪後要等 idle task 清理才能重用同一塊記憶體 */
    s_reader_task = xTaskCreateStaticPinnedToCore(upload_reader_task, "upload_reader",
                                                  READER_STACK_BYTES, NULL,
                                                  READER_PRIORITY, s_reader_stack,
                                                  &s_reader_tcb, reader_core);
    return (s_free_queue && s_full_queue && s_reader_task) ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t send_file(const node_pending_file_t *pending, int *status_out)
{
    *status_out = 0;
    if (!wifi_manager_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    const size_t file_size = pending->file_bytes;
    if (file_size == 0 || file_size > INT_MAX ||
        (pending->media == NODE_MEDIA_VIDEO && (file_size % CAM_IMAGE_FRAME_BYTES) != 0)) {
        /* Cannot be a valid recording; the server would reject it too. */
        *status_out = 400;
        return ESP_ERR_INVALID_SIZE;
    }
    ESP_RETURN_ON_ERROR(ensure_reader_task(), TAG, "upload reader task");

    /*
     * 用 open()/read() 而不是 fopen()/fread()：stdio 的 128 B 緩衝讓 FATFS 只能
     * 逐 sector 讀 (約 1.1 MB/s，debug_sdcard #22)；無緩衝的 fread 在 ROM newlib
     * 一次只讀 1 byte (#21)。read() 16 KB 直接進對齊的內部 RAM 緩衝，FATFS 走
     * 多 sector 讀取，與錄製時的 write() 路徑相同。
     */
    int fd = open(pending->path, O_RDONLY);
    if (fd < 0) {
        return ESP_FAIL;
    }

    const char *device_id = device_identity_get();
    const char *media_name = node_storage_media_name(pending->media);
    char session_id[ID_TEXT_SIZE];
    char capture_id[ID_TEXT_SIZE + 8U];   /* session_id + "-video" */
    char recorded_at[24];
    char file_size_text[24];
    char offset_text[24];
    snprintf(session_id, sizeof(session_id), "%s-%lld",
             device_id, (long long)pending->recorded_at_epoch);
    snprintf(capture_id, sizeof(capture_id), "%s-%s", session_id, media_name);
    snprintf(recorded_at, sizeof(recorded_at), "%lld",
             (long long)pending->recorded_at_epoch);
    snprintf(file_size_text, sizeof(file_size_text), "%u", (unsigned)file_size);

    esp_http_client_config_t config = {
        .url = pending->media == NODE_MEDIA_VIDEO ? APP_SERVER_VIDEO_URL
                                                  : APP_SERVER_AUDIO_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = APP_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        close(fd);
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_set_header(client, "X-Device-ID", device_id);
    esp_http_client_set_header(client, "X-Session-ID", session_id);
    esp_http_client_set_header(client, "X-Capture-ID", capture_id);
    esp_http_client_set_header(client, "X-Recorded-At", recorded_at);
    esp_http_client_set_header(client, "X-File-Size", file_size_text);
    if (pending->recorded_at_epoch == s_offsets_epoch) {
        snprintf(offset_text, sizeof(offset_text), "%" PRId64,
                 pending->media == NODE_MEDIA_VIDEO ? s_camera_offset_us
                                                    : s_mic_offset_us);
        esp_http_client_set_header(client, "X-Start-Offset-US", offset_text);
    }
    set_media_headers(client, pending);

    const int64_t open_start_us = esp_timer_get_time();
    esp_err_t err = esp_http_client_open(client, (int)file_size);
    timing_trace_stat(TT_STAT_UP_OPEN, esp_timer_get_time() - open_start_us, s_file_index);
    if (err != ESP_OK) {
        close(fd);
        esp_http_client_cleanup(client);
        return err;
    }

    ESP_LOGI(TAG, "Uploading %s (%u bytes), capture_id=%s",
             pending->path, (unsigned)file_size, capture_id);
    const int64_t started_us = esp_timer_get_time();
    /* 計時 (debug)：sd_read 在 reader task 累計 (與送出重疊)；wait_sd = main 空等資料的時間 */
    int64_t net_send_us = 0;
    int64_t max_net_send_us = 0;
    int64_t wait_sd_us = 0;
    size_t total_written = 0;

    (void)xQueueReset(s_free_queue);
    (void)xQueueReset(s_full_queue);
    for (uint8_t i = 0; i < UPLOAD_BUFFER_COUNT; i++) {
        (void)xQueueSend(s_free_queue, &i, 0);
    }
    s_job.fd = fd;
    s_job.file_size = file_size;
    s_job.abort = false;
    s_job.sd_read_us = 0;
    s_job.max_sd_read_us = 0;
    xTaskNotifyGive(s_reader_task);

    /* 一定收到 reader 的結束訊息才離開，之後才能 close(fd)、重用緩衝 */
    while (true) {
        upload_chunk_t chunk;
        const int64_t wait_start_us = esp_timer_get_time();
        (void)xQueueReceive(s_full_queue, &chunk, portMAX_DELAY);
        wait_sd_us += esp_timer_get_time() - wait_start_us;
        if (chunk.length <= 0) {
            if (chunk.length < 0 && err == ESP_OK) {
                err = ESP_FAIL;
            }
            break;
        }
        if (err == ESP_OK) {
            const uint8_t *data = s_io_buffer + (size_t)chunk.index * UPLOAD_BUFFER_SIZE;
            const size_t read_size = (size_t)chunk.length;
            const int64_t send_start_us = esp_timer_get_time();
            size_t offset = 0;
            while (offset < read_size) {
                int written = esp_http_client_write(
                    client, (const char *)data + offset, (int)(read_size - offset));
                if (written <= 0) {
                    err = ESP_FAIL;
                    s_job.abort = true;
                    break;
                }
                offset += (size_t)written;
                total_written += (size_t)written;
            }
            const int64_t send_us = esp_timer_get_time() - send_start_us;
            net_send_us += send_us;
            if (send_us > max_net_send_us) {
                max_net_send_us = send_us;
            }
        }
        (void)xQueueSend(s_free_queue, &chunk.index, portMAX_DELAY);
    }
    close(fd);
    timing_trace_stat(TT_STAT_UP_BODY, esp_timer_get_time() - started_us, s_file_index);
    const int64_t sd_read_us = s_job.sd_read_us;
    const int64_t max_sd_read_us = s_job.max_sd_read_us;

    char ack[ACK_BUFFER_SIZE] = {0};
    int status = 0;
    int ack_bytes = 0;
    const int64_t ack_start_us = esp_timer_get_time();
    if (err == ESP_OK && total_written == file_size) {
        (void)esp_http_client_fetch_headers(client);
        status = esp_http_client_get_status_code(client);
        ack_bytes = esp_http_client_read_response(client, ack, sizeof(ack) - 1U);
        if (ack_bytes > 0) {
            ack[ack_bytes] = '\0';
        }
        if (status != 200 || ack_bytes <= 0) {
            err = ESP_FAIL;
        } else {
            err = verify_ack(ack, capture_id, file_size);
        }
    }
    const int64_t ack_wait_us = esp_timer_get_time() - ack_start_us;
    timing_trace_stat(TT_STAT_UP_ACK, ack_wait_us, s_file_index);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    *status_out = status;

    /* 速度 = 位元組 / 微秒 * 1000 = kB/s */
    ESP_LOGI(TAG, "Upload timing: total=%" PRId64 " ms | sd_read=%" PRId64
             " ms (%" PRId64 " kB/s, max %" PRId64 " ms/16KB) | wifi_send=%" PRId64
             " ms (%" PRId64 " kB/s, max %" PRId64 " ms/16KB) | wait_sd=%" PRId64
             " ms | ack_wait=%" PRId64 " ms",
             (esp_timer_get_time() - started_us) / 1000,
             sd_read_us / 1000,
             sd_read_us > 0 ? (int64_t)total_written * 1000 / sd_read_us : 0,
             max_sd_read_us / 1000,
             net_send_us / 1000,
             net_send_us > 0 ? (int64_t)total_written * 1000 / net_send_us : 0,
             max_net_send_us / 1000,
             wait_sd_us / 1000,
             ack_wait_us / 1000);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Upload not acknowledged: HTTP %d, response=%s",
                 status, ack_bytes > 0 ? ack : "(empty)");
        return err;
    }

    const int64_t elapsed_us = esp_timer_get_time() - started_us;
    if (elapsed_us > 0) {
        uint32_t measured = (uint32_t)(((uint64_t)file_size * 1000000ULL) /
                                       (uint64_t)elapsed_us);
        if (measured < UPLOAD_MIN_RATE_BPS) {
            measured = UPLOAD_MIN_RATE_BPS;
        }
        s_rate_bps = (s_rate_bps + measured) / 2U;
        ESP_LOGI(TAG, "ACK verified: %u bytes in %" PRId64 " ms (%u kB/s)",
                 (unsigned)file_size, elapsed_us / 1000,
                 (unsigned)(measured / 1000U));
    }
    const int64_t delete_start_us = esp_timer_get_time();
    const esp_err_t delete_err = node_storage_delete_uploaded(pending->path);
    timing_trace_stat(TT_STAT_UP_DELETE, esp_timer_get_time() - delete_start_us, s_file_index);
    return delete_err;
}

/* 每個檔案上傳期間持有共用 I/O 緩衝；send_file 結束時 reader 已停 (收到結束訊息) */
static esp_err_t upload_one_file(const node_pending_file_t *pending, int *status_out)
{
    *status_out = 0;
    s_io_buffer = node_storage_io_buffer_acquire("upload");
    if (s_io_buffer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t err = send_file(pending, status_out);
    node_storage_io_buffer_release(s_io_buffer);
    s_io_buffer = NULL;
    return err;
}

esp_err_t node_upload_process_pending(time_t deadline_epoch,
                                      node_upload_summary_t *summary)
{
    if (!summary) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(summary, 0, sizeof(*summary));
    summary->last_error = ESP_OK;

    for (s_file_index = 0; ; ++s_file_index) {
        node_pending_file_t pending;
        const int64_t scan_start_us = esp_timer_get_time();
        esp_err_t err = node_storage_get_next_pending(&pending);
        timing_trace_stat(TT_STAT_UP_SCAN, esp_timer_get_time() - scan_start_us, s_file_index);
        if (err == ESP_ERR_NOT_FOUND) {
            return ESP_OK;
        }
        if (err != ESP_OK) {
            summary->last_error = err;
            return err;
        }

        if (deadline_epoch > 0) {
            const time_t estimate_s = (time_t)(pending.file_bytes / s_rate_bps) + 1;
            if (time(NULL) + estimate_s > deadline_epoch) {
                ESP_LOGW(TAG, "Deferring %s: ~%lld s left before the next session",
                         pending.path, (long long)(deadline_epoch - time(NULL)));
                summary->deferred = true;
                return ESP_OK;
            }
        }

        int status = 0;
        err = upload_one_file(&pending, &status);
        if (err == ESP_OK) {
            ++summary->uploaded_count;
            summary->uploaded_bytes += pending.file_bytes;
            continue;
        }

        summary->last_error = err;
        if (is_permanent_rejection(status) &&
            node_storage_mark_rejected(&pending) == ESP_OK) {
            ++summary->rejected_count;
            continue;
        }
        ++summary->failed_count;
        return err;
    }
}
