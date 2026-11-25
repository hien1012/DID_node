/*
 * 低信賴樣本上傳：POST /infer/sample，body = meta JSON + 原始資料。
 *   X-Device-ID / X-Sample-ID / X-Media
 *   X-Meta-Length：body 開頭 meta 的位元組數
 *   X-File-Size：之後原始資料的位元組數
 * ACK {"status":"ok","sample_id":...,"bytes":<原始資料大小>} 驗證通過才刪本地檔。
 *
 * 連線管理 (批次內 Keep-Alive)：
 *   同一輪 sample_upload_pending() 共用一個 esp_http_client_handle_t，整批傳完立刻 cleanup 釋放內部 DRAM。
 *   只有「完整寫滿 Content-Length + 收到 HTTP 200 + 讀完回應 body + server 未帶 Connection: close」才保留連線；
 *   若沿用的舊連線剛好被對端關閉，自動關掉舊連線並重試一次新連線。
 */
#include "sample_upload.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "device_identity.h"
#include "infer_config.h"
#include "wifi_manager.h"

#define META_BUFFER_BYTES   2048U
#define ACK_BUFFER_BYTES    512U
#define SAMPLE_ID_MAX       (32U + SAMPLE_STEM_MAX)
/* 等空檔的上限：兩個 2 s 視窗都沒有空檔就照樣送，上傳不會因推論太慢而停擺 */
#define GAP_MAX_WAIT_MS     4000U

typedef struct {
    esp_http_client_handle_t handle;
    bool connected;         /* 底層 TCP 連線目前仍開著，下一筆將重用 */
    bool peer_will_close;   /* 回應標頭明確帶了 Connection: close */
} upload_client_t;

static const char *TAG = "SAMPLE_UPLOAD";
static char *s_meta = NULL;     /* PSRAM：只在 main task 用 */
static uint8_t *s_read_buffer = NULL;
static sample_upload_gap_fn s_wait_gap = NULL;

esp_err_t sample_upload_init(uint8_t *read_buffer, sample_upload_gap_fn wait_gap)
{
    s_read_buffer = read_buffer;
    s_wait_gap = wait_gap;
    if (!s_meta) {
        s_meta = heap_caps_malloc(META_BUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    return (s_meta && s_read_buffer) ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    upload_client_t *uc = (upload_client_t *)evt->user_data;
    if (!uc) {
        return ESP_OK;
    }
    if (evt->event_id == HTTP_EVENT_ON_HEADER && evt->header_key && evt->header_value) {
        if (strcasecmp(evt->header_key, "Connection") == 0 &&
            strcasecmp(evt->header_value, "close") == 0) {
            uc->peer_will_close = true;
        }
    } else if (evt->event_id == HTTP_EVENT_DISCONNECTED) {
        uc->connected = false;
    }
    return ESP_OK;
}

static esp_err_t read_meta(const char *path, size_t *bytes_out, bool *missing_out)
{
    *missing_out = false;
    const int fd = open(path, O_RDONLY);
    if (fd < 0) {
        *missing_out = (errno == ENOENT);
        return ESP_FAIL;
    }
    const ssize_t got = read(fd, s_meta, META_BUFFER_BYTES);
    close(fd);
    if (got <= 0 || (size_t)got >= META_BUFFER_BYTES) {
        ESP_LOGE(TAG, "Bad meta size %d: %s", (int)got, path);
        return ESP_ERR_INVALID_SIZE;
    }
    *bytes_out = (size_t)got;
    return ESP_OK;
}

static esp_err_t verify_ack(const char *json, const char *sample_id, size_t expected_bytes)
{
    cJSON *root = cJSON_Parse(json);
    cJSON *status = root ? cJSON_GetObjectItemCaseSensitive(root, "status") : NULL;
    cJSON *ack_id = root ? cJSON_GetObjectItemCaseSensitive(root, "sample_id") : NULL;
    cJSON *bytes = root ? cJSON_GetObjectItemCaseSensitive(root, "bytes") : NULL;
    const bool valid = cJSON_IsString(status) && strcmp(status->valuestring, "ok") == 0 &&
                       cJSON_IsString(ack_id) && strcmp(ack_id->valuestring, sample_id) == 0 &&
                       cJSON_IsNumber(bytes) && (size_t)bytes->valuedouble == expected_bytes;
    cJSON_Delete(root);
    return valid ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t write_all_http(esp_http_client_handle_t client, const char *data, size_t bytes)
{
    size_t offset = 0;
    while (offset < bytes) {
        const int written = esp_http_client_write(client, data + offset, (int)(bytes - offset));
        if (written <= 0) {
            return ESP_FAIL;
        }
        offset += (size_t)written;
    }
    return ESP_OK;
}

/* 讀 SD 前等管線空檔 (沒有設定 wait_gap 時直接返回)，等待時間與逾時記進 summary */
static void wait_gap(sample_upload_summary_t *summary)
{
    if (!s_wait_gap) {
        return;
    }
    const int64_t started_us = esp_timer_get_time();
    if (!s_wait_gap(GAP_MAX_WAIT_MS)) {
        ++summary->gap_timeouts;
    }
    summary->gap_wait_ms += (uint32_t)((esp_timer_get_time() - started_us) / 1000);
}

/* 4xx except timeout/too-many-requests: retrying the same sample cannot succeed. */
static bool is_permanent_rejection(int status)
{
    return status >= 400 && status < 500 && status != 408 && status != 429;
}

static esp_err_t send_sample_attempt(upload_client_t *uc, int fd, const sample_entry_t *entry,
                                     const char *sample_id, size_t meta_bytes, int *status_out,
                                     char ack[ACK_BUFFER_BYTES], int *ack_bytes_out,
                                     sample_upload_summary_t *summary)
{
    *status_out = 0;
    *ack_bytes_out = 0;
    ack[0] = '\0';

    char meta_length_text[16];
    char file_size_text[16];
    snprintf(meta_length_text, sizeof(meta_length_text), "%u", (unsigned)meta_bytes);
    snprintf(file_size_text, sizeof(file_size_text), "%u", (unsigned)entry->data_bytes);

    esp_http_client_set_header(uc->handle, "X-Sample-ID", sample_id);
    esp_http_client_set_header(uc->handle, "X-Media", sample_store_media_name(entry->media));
    esp_http_client_set_header(uc->handle, "X-Meta-Length", meta_length_text);
    esp_http_client_set_header(uc->handle, "X-File-Size", file_size_text);

    uc->peer_will_close = false;
    const int total_post_bytes = (int)(meta_bytes + entry->data_bytes);
    esp_err_t err = esp_http_client_open(uc->handle, total_post_bytes);
    if (err == ESP_OK) {
        uc->connected = true;
        err = write_all_http(uc->handle, s_meta, meta_bytes);
    }

    size_t remaining = entry->data_bytes;
    while (err == ESP_OK && remaining > 0U) {
        const size_t want =
            remaining < SAMPLE_UPLOAD_BUFFER_BYTES ? remaining : SAMPLE_UPLOAD_BUFFER_BYTES;
        /* 連線開著等空檔：最多等 GAP_MAX_WAIT_MS，低於 server 讀 body 的逾時 */
        wait_gap(summary);
        /* 讀進內部 RAM：PSRAM 緩衝會讓 sdmmc driver 退回逐 sector 讀 */
        const ssize_t got = read(fd, s_read_buffer, want);
        if (got <= 0) {
            err = ESP_FAIL;
            break;
        }
        err = write_all_http(uc->handle, (const char *)s_read_buffer, (size_t)got);
        remaining -= (size_t)got;
    }

    bool keep_open = false;
    if (err == ESP_OK && remaining == 0U) {
        const int64_t content_len = esp_http_client_fetch_headers(uc->handle);
        if (content_len < 0) {
            err = ESP_FAIL;
        } else {
            const bool has_framing =
                content_len > 0 || esp_http_client_is_chunked_response(uc->handle);
            *status_out = esp_http_client_get_status_code(uc->handle);
            *ack_bytes_out =
                esp_http_client_read_response(uc->handle, ack, ACK_BUFFER_BYTES - 1U);
            if (*ack_bytes_out > 0) {
                ack[*ack_bytes_out] = '\0';
            }
            const bool body_complete = esp_http_client_is_complete_data_received(uc->handle);
            err = (*status_out == 200 && *ack_bytes_out > 0)
                      ? verify_ack(ack, sample_id, entry->data_bytes)
                      : ESP_FAIL;
            keep_open = (err == ESP_OK) && has_framing && body_complete && !uc->peer_will_close;
        }
    }

    /* 任何未寫滿、未讀乾淨、非 200 或對端宣告關閉的情況，一律斷開底層 TCP 免得污染下一筆 framing */
    if (!keep_open) {
        (void)esp_http_client_close(uc->handle);
        uc->connected = false;
    }
    return err;
}

static esp_err_t send_sample(upload_client_t *uc, const sample_entry_t *entry, int *status_out,
                             bool *missing_on_sd_out, sample_upload_summary_t *summary)
{
    *status_out = 0;
    *missing_on_sd_out = false;

    size_t meta_bytes = 0;
    esp_err_t err = read_meta(entry->meta_path, &meta_bytes, missing_on_sd_out);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read meta failed: %s", entry->meta_path);
        return err;
    }
    if (entry->data_bytes == 0U || entry->data_bytes > (size_t)(INT_MAX - meta_bytes)) {
        *status_out = 400;      /* 不可能是完整樣本，server 也會拒收 */
        return ESP_ERR_INVALID_SIZE;
    }

    const int fd = open(entry->data_path, O_RDONLY);
    if (fd < 0) {
        *missing_on_sd_out = (errno == ENOENT);
        return ESP_FAIL;
    }

    char sample_id[SAMPLE_ID_MAX];
    snprintf(sample_id, sizeof(sample_id), "%s-%s", device_identity_get(), entry->stem);

    char ack[ACK_BUFFER_BYTES] = {0};
    int ack_bytes = 0;
    const bool was_reused = uc->connected;
    const int64_t started_us = esp_timer_get_time();

    err = send_sample_attempt(uc, fd, entry, sample_id, meta_bytes, status_out, ack, &ack_bytes,
                              summary);
    /* 重用的 Keep-Alive 連線若剛好被對端逾時關閉 (尚未拿到任何 HTTP status)，重置檔案游標重連一次 */
    if (err != ESP_OK && was_reused && *status_out == 0 && lseek(fd, 0, SEEK_SET) == 0) {
        err = send_sample_attempt(uc, fd, entry, sample_id, meta_bytes, status_out, ack,
                                  &ack_bytes, summary);
    }
    close(fd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Upload not acknowledged: %s HTTP %d, response=%s", entry->stem, *status_out,
                 ack_bytes > 0 ? ack : "(empty)");
        return err;
    }

    const int64_t elapsed_us = esp_timer_get_time() - started_us;
    ESP_LOGI(TAG, "ACK verified: %s %u bytes in %" PRId64 " ms", entry->stem,
             (unsigned)entry->data_bytes, elapsed_us / 1000);
    return ESP_OK;
}

esp_err_t sample_upload_pending(sample_media_t media, sample_upload_summary_t *summary)
{
    if (!summary || !s_meta) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(summary, 0, sizeof(*summary));
    if (!wifi_manager_is_connected()) {
        summary->last_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    upload_client_t uc = {0};
    const esp_http_client_config_t config = {
        .url = INFER_SERVER_SAMPLE_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = APP_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
        .event_handler = http_event_handler,
        .user_data = &uc,
    };
    uc.handle = esp_http_client_init(&config);
    if (!uc.handle) {
        summary->last_error = ESP_ERR_NO_MEM;
        return ESP_ERR_NO_MEM;
    }
    esp_http_client_set_header(uc.handle, "Content-Type", "application/octet-stream");
    esp_http_client_set_header(uc.handle, "X-Device-ID", device_identity_get());

    esp_err_t result = ESP_OK;
    while (true) {
        if (!wifi_manager_is_connected()) {
            result = ESP_ERR_INVALID_STATE;
            break;
        }
        /* sample_store_next 已改為純記憶體 Min-Heap 查詢，不再掃 SD 目錄；讀 meta 前才等空檔 */
        sample_entry_t entry;
        esp_err_t err = sample_store_next(media, &entry);
        if (err == ESP_ERR_NOT_FOUND) {
            break;
        }
        if (err != ESP_OK) {
            result = err;
            break;
        }

        wait_gap(summary);
        int status = 0;
        bool missing_on_sd = false;
        err = send_sample(&uc, &entry, &status, &missing_on_sd, summary);
        if (missing_on_sd) {
            ESP_LOGW(TAG, "Dropping missing sample %s", entry.stem);
            wait_gap(summary);
            (void)sample_store_remove(&entry);
            continue;
        }
        if (err == ESP_OK || is_permanent_rejection(status)) {
            wait_gap(summary);      /* 刪檔 / 搬檔也要寫 SD */
        }
        if (err == ESP_OK) {
            (void)sample_store_remove(&entry);
            ++summary->uploaded_count;
            summary->uploaded_bytes += entry.data_bytes;
            continue;
        }
        if (is_permanent_rejection(status) && sample_store_mark_rejected(&entry) == ESP_OK) {
            ++summary->rejected_count;
            continue;
        }
        ++summary->failed_count;
        result = err;
        break;
    }

    esp_http_client_cleanup(uc.handle);
    summary->last_error = result;
    return result;
}
