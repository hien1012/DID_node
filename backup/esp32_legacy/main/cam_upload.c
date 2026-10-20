#include "cam_upload.h"

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
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "app_config.h"
#include "cam_record.h"
#include "cam_storage.h"
#include "device_identity.h"
#include "wifi_manager.h"

#define UPLOAD_BUFFER_SIZE (16U * 1024U)
#define ACK_BUFFER_SIZE    512U
#define CAPTURE_ID_SIZE    64U

static const char *TAG = "CAM_UPLOAD";
/*
 * 讀檔用 open()/read() 直接讀進這塊緩衝：4-byte 對齊、內部 RAM，SDMMC 可直接
 * DMA，FATFS 一次讀多個 sector。fread() 經 stdio 128 B 緩衝會變成逐 sector
 * 讀取 (S3 版實測 1.1 MB/s → 4.0 MB/s，docs/debug_sdcard.md #22-#23)。
 */
static WORD_ALIGNED_ATTR uint8_t s_upload_buffer[UPLOAD_BUFFER_SIZE];

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

static esp_err_t upload_one_file(const char *path, time_t recorded_at_epoch,
                                 size_t *uploaded_bytes)
{
    if (!wifi_manager_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t file_size = 0;
    esp_err_t err = cam_storage_get_file_size(path, &file_size);
    if (err != ESP_OK || file_size == 0 || file_size > INT_MAX ||
        (file_size % CAM_IMAGE_FRAME_BYTES) != 0) {
        return err == ESP_OK ? ESP_ERR_INVALID_SIZE : err;
    }
    size_t frame_count = file_size / CAM_IMAGE_FRAME_BYTES;

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        return ESP_FAIL;
    }

    char capture_id[CAPTURE_ID_SIZE];
    char recorded_at[24];
    char file_size_text[24];
    char frame_count_text[16];
    char width_text[16];
    char height_text[16];
    char fps_text[16];
    const char *device_id = device_identity_get();
    snprintf(capture_id, sizeof(capture_id), "%s-%lld",
             device_id, (long long)recorded_at_epoch);
    snprintf(recorded_at, sizeof(recorded_at), "%lld",
             (long long)recorded_at_epoch);
    snprintf(file_size_text, sizeof(file_size_text), "%u", (unsigned)file_size);
    snprintf(frame_count_text, sizeof(frame_count_text), "%u",
             (unsigned)frame_count);
    snprintf(width_text, sizeof(width_text), "%u", (unsigned)CAM_IMAGE_WIDTH);
    snprintf(height_text, sizeof(height_text), "%u", (unsigned)CAM_IMAGE_HEIGHT);
    snprintf(fps_text, sizeof(fps_text), "%u", (unsigned)CAM_TARGET_FPS);

    esp_http_client_config_t config = {
        .url = APP_SERVER_VIDEO_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = APP_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        close(fd);
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
    esp_http_client_set_header(client, "X-Device-ID", device_id);
    esp_http_client_set_header(client, "X-Capture-ID", capture_id);
    esp_http_client_set_header(client, "X-Recorded-At", recorded_at);
    esp_http_client_set_header(client, "X-File-Size", file_size_text);
    esp_http_client_set_header(client, "X-Frame-Count", frame_count_text);
    esp_http_client_set_header(client, "X-Width", width_text);
    esp_http_client_set_header(client, "X-Height", height_text);
    esp_http_client_set_header(client, "X-FPS", fps_text);
    esp_http_client_set_header(client, "X-Format", "GRAY8");

    err = esp_http_client_open(client, (int)file_size);
    if (err != ESP_OK) {
        close(fd);
        esp_http_client_cleanup(client);
        return err;
    }

    ESP_LOGI(TAG, "Uploading %s (%u bytes, %u frames), capture_id=%s",
             path, (unsigned)file_size, (unsigned)frame_count, capture_id);
    const int64_t started_us = esp_timer_get_time();
    /* 計時 (debug)：分開累計讀 SD 與 Wi-Fi 送出的時間，找出上傳瓶頸 */
    int64_t sd_read_us = 0;
    int64_t net_send_us = 0;
    int64_t max_sd_read_us = 0;
    int64_t max_net_send_us = 0;
    size_t total_written = 0;
    while (total_written < file_size) {
        const int64_t read_start_us = esp_timer_get_time();
        const ssize_t read_result = read(fd, s_upload_buffer, sizeof(s_upload_buffer));
        const int64_t read_us = esp_timer_get_time() - read_start_us;
        sd_read_us += read_us;
        if (read_us > max_sd_read_us) {
            max_sd_read_us = read_us;
        }
        if (read_result <= 0) {   /* 0 = 檔案比預期短，-1 = I/O 錯誤 */
            err = ESP_FAIL;
            break;
        }
        const size_t read_size = (size_t)read_result;
        const int64_t send_start_us = esp_timer_get_time();
        size_t offset = 0;
        while (offset < read_size) {
            int written = esp_http_client_write(
                client, (const char *)s_upload_buffer + offset,
                (int)(read_size - offset));
            if (written <= 0) {
                err = ESP_FAIL;
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
        if (err != ESP_OK) {
            break;
        }
    }
    close(fd);

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
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    /* 速度 = 位元組 * 1000 / 微秒 = kB/s */
    const int64_t total_us = esp_timer_get_time() - started_us;
    ESP_LOGI(TAG, "Upload timing: total=%" PRId64 " ms (%" PRId64 " kB/s) | sd_read=%" PRId64
             " ms (%" PRId64 " kB/s, max %" PRId64 " ms/16KB) | wifi_send=%" PRId64
             " ms (%" PRId64 " kB/s, max %" PRId64 " ms/16KB) | ack_wait=%" PRId64 " ms",
             total_us / 1000,
             total_us > 0 ? (int64_t)total_written * 1000 / total_us : 0,
             sd_read_us / 1000,
             sd_read_us > 0 ? (int64_t)total_written * 1000 / sd_read_us : 0,
             max_sd_read_us / 1000,
             net_send_us / 1000,
             net_send_us > 0 ? (int64_t)total_written * 1000 / net_send_us : 0,
             max_net_send_us / 1000,
             ack_wait_us / 1000);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Upload not acknowledged: HTTP %d, response=%s",
                 status, ack_bytes > 0 ? ack : "(empty)");
        return err;
    }

    err = cam_storage_delete_done(path);
    if (err == ESP_OK) {
        *uploaded_bytes = file_size;
        ESP_LOGI(TAG, "ACK verified; local file removed: %s", path);
    }
    return err;
}

esp_err_t cam_upload_process_pending(cam_upload_summary_t *summary)
{
    if (!summary) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(summary, 0, sizeof(*summary));
    summary->last_error = ESP_OK;

    while (true) {
        char path[CAM_STORAGE_PATH_MAX] = {0};
        time_t recorded_at_epoch = 0;
        esp_err_t err = cam_storage_get_next_done(
            path, sizeof(path), &recorded_at_epoch);
        if (err == ESP_ERR_NOT_FOUND) {
            return ESP_OK;
        }
        if (err != ESP_OK) {
            summary->last_error = err;
            return err;
        }

        size_t uploaded_bytes = 0;
        err = upload_one_file(path, recorded_at_epoch, &uploaded_bytes);
        if (err != ESP_OK) {
            ++summary->failed_count;
            summary->last_error = err;
            return err;
        }
        ++summary->uploaded_count;
        summary->uploaded_bytes += uploaded_bytes;
    }
}
