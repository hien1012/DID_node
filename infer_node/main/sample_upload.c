/*
 * 低信賴樣本上傳：POST /infer/sample，body = meta JSON + 原始資料。
 *   X-Device-ID / X-Sample-ID / X-Media
 *   X-Meta-Length：body 開頭 meta 的位元組數
 *   X-File-Size：之後原始資料的位元組數
 * ACK {"status":"ok","sample_id":...,"bytes":<原始資料大小>} 驗證通過才刪本地檔。
 */
#include "sample_upload.h"

#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
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

static esp_err_t read_meta(const char *path, size_t *bytes_out)
{
    const int fd = open(path, O_RDONLY);
    if (fd < 0) {
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

static esp_err_t send_sample(const sample_entry_t *entry, int *status_out,
                             sample_upload_summary_t *summary)
{
    *status_out = 0;
    size_t meta_bytes = 0;
    ESP_RETURN_ON_ERROR(read_meta(entry->meta_path, &meta_bytes), TAG, "read meta");
    if (entry->data_bytes == 0U || entry->data_bytes > (size_t)(INT_MAX - meta_bytes)) {
        *status_out = 400;      /* 不可能是完整樣本，server 也會拒收 */
        return ESP_ERR_INVALID_SIZE;
    }

    const int fd = open(entry->data_path, O_RDONLY);
    if (fd < 0) {
        return ESP_FAIL;
    }

    char sample_id[SAMPLE_ID_MAX];
    char meta_length_text[16];
    char file_size_text[16];
    snprintf(sample_id, sizeof(sample_id), "%s-%s", device_identity_get(), entry->stem);
    snprintf(meta_length_text, sizeof(meta_length_text), "%u", (unsigned)meta_bytes);
    snprintf(file_size_text, sizeof(file_size_text), "%u", (unsigned)entry->data_bytes);

    const esp_http_client_config_t config = {
        .url = INFER_SERVER_SAMPLE_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = APP_HTTP_TIMEOUT_MS,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        close(fd);
        return ESP_ERR_NO_MEM;
    }
    esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
    esp_http_client_set_header(client, "X-Device-ID", device_identity_get());
    esp_http_client_set_header(client, "X-Sample-ID", sample_id);
    esp_http_client_set_header(client, "X-Media", sample_store_media_name(entry->media));
    esp_http_client_set_header(client, "X-Meta-Length", meta_length_text);
    esp_http_client_set_header(client, "X-File-Size", file_size_text);

    const int64_t started_us = esp_timer_get_time();
    esp_err_t err = esp_http_client_open(client, (int)(meta_bytes + entry->data_bytes));
    if (err == ESP_OK) {
        err = write_all_http(client, s_meta, meta_bytes);
    }
    size_t remaining = entry->data_bytes;
    while (err == ESP_OK && remaining > 0U) {
        const size_t want = remaining < SAMPLE_UPLOAD_BUFFER_BYTES ? remaining : SAMPLE_UPLOAD_BUFFER_BYTES;
        /* 連線開著等空檔：最多等 GAP_MAX_WAIT_MS，低於 server 讀 body 的逾時 */
        wait_gap(summary);
        /* 讀進內部 RAM：PSRAM 緩衝會讓 sdmmc driver 退回逐 sector 讀 */
        const ssize_t got = read(fd, s_read_buffer, want);
        if (got <= 0) {
            err = ESP_FAIL;
            break;
        }
        err = write_all_http(client, (const char *)s_read_buffer, (size_t)got);
        remaining -= (size_t)got;
    }
    close(fd);

    char ack[ACK_BUFFER_BYTES] = {0};
    int status = 0;
    int ack_bytes = 0;
    if (err == ESP_OK) {
        (void)esp_http_client_fetch_headers(client);
        status = esp_http_client_get_status_code(client);
        ack_bytes = esp_http_client_read_response(client, ack, sizeof(ack) - 1U);
        if (ack_bytes > 0) {
            ack[ack_bytes] = '\0';
        }
        err = (status == 200 && ack_bytes > 0) ? verify_ack(ack, sample_id, entry->data_bytes)
                                               : ESP_FAIL;
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    *status_out = status;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Upload not acknowledged: %s HTTP %d, response=%s", entry->stem, status,
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

    esp_err_t result = ESP_OK;
    while (true) {
        if (!wifi_manager_is_connected()) {
            result = ESP_ERR_INVALID_STATE;
            break;
        }
        /* 掃目錄 + 讀 meta + 開連線 */
        wait_gap(summary);
        sample_entry_t entry;
        esp_err_t err = sample_store_next(media, &entry);
        if (err == ESP_ERR_NOT_FOUND) {
            break;
        }
        if (err != ESP_OK) {
            result = err;
            break;
        }

        int status = 0;
        err = send_sample(&entry, &status, summary);
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
    summary->last_error = result;
    return result;
}
