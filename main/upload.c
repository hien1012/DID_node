#include "upload.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"

#include "app_config.h"
#include "device_identity.h"
#include "storage.h"
#include "wifi_manager.h"

#define UPLOAD_BUFFER_SIZE 4096
#define ACK_BUFFER_SIZE    512
#define CAPTURE_ID_SIZE    64

static const char *TAG = "UPLOAD";
static uint8_t s_upload_buffer[UPLOAD_BUFFER_SIZE];

static esp_err_t verify_ack(const char *json, const char *capture_id,
                            size_t expected_size)
{
    cJSON *root = cJSON_Parse(json);
    cJSON *status = root ? cJSON_GetObjectItemCaseSensitive(root, "status") : NULL;
    cJSON *ack_id = root ? cJSON_GetObjectItemCaseSensitive(root, "capture_id") : NULL;
    cJSON *bytes = root ? cJSON_GetObjectItemCaseSensitive(root, "bytes") : NULL;

    bool valid = cJSON_IsString(status) && strcmp(status->valuestring, "ok") == 0 &&
                 cJSON_IsString(ack_id) && strcmp(ack_id->valuestring, capture_id) == 0 &&
                 cJSON_IsNumber(bytes) && (size_t)bytes->valuedouble == expected_size;
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
    esp_err_t err = storage_get_file_size(path, &file_size);
    if (err != ESP_OK || file_size == 0 || file_size > INT_MAX) {
        return err == ESP_OK ? ESP_ERR_INVALID_SIZE : err;
    }

    FILE *file = fopen(path, "rb");
    if (!file) {
        return ESP_FAIL;
    }

    char capture_id[CAPTURE_ID_SIZE];
    char recorded_at[24];
    char file_size_text[24];
    const char *device_id = device_identity_get();

    snprintf(capture_id, sizeof(capture_id), "%s-%lld",
             device_id, (long long)recorded_at_epoch);
    snprintf(recorded_at, sizeof(recorded_at), "%lld",
             (long long)recorded_at_epoch);
    snprintf(file_size_text, sizeof(file_size_text), "%u", (unsigned)file_size);

    esp_http_client_config_t config = {
        .url = APP_SERVER_AUDIO_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = APP_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        fclose(file);
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_set_header(client, "Content-Type", "audio/wav");
    esp_http_client_set_header(client, "X-Device-ID", device_id);
    esp_http_client_set_header(client, "X-Capture-ID", capture_id);
    esp_http_client_set_header(client, "X-Recorded-At", recorded_at);
    esp_http_client_set_header(client, "X-File-Size", file_size_text);

    err = esp_http_client_open(client, (int)file_size);
    if (err != ESP_OK) {
        fclose(file);
        esp_http_client_cleanup(client);
        return err;
    }

    ESP_LOGI(TAG, "Uploading %s (%u bytes), capture_id=%s",
             path, (unsigned)file_size, capture_id);
    size_t total_written = 0;
    while (total_written < file_size) {
        size_t read_size = fread(s_upload_buffer, 1, sizeof(s_upload_buffer), file);
        if (read_size == 0) {
            err = ESP_FAIL;
            break;
        }

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
        if (err != ESP_OK) {
            break;
        }
    }
    fclose(file);

    char ack[ACK_BUFFER_SIZE] = {0};
    int status = 0;
    int ack_bytes = 0;
    if (err == ESP_OK && total_written == file_size) {
        (void)esp_http_client_fetch_headers(client);
        status = esp_http_client_get_status_code(client);
        ack_bytes = esp_http_client_read_response(client, ack, sizeof(ack) - 1);
        if (ack_bytes > 0) {
            ack[ack_bytes] = '\0';
        }
        if (status != 200 || ack_bytes <= 0) {
            err = ESP_FAIL;
        } else {
            err = verify_ack(ack, capture_id, file_size);
        }
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Upload not acknowledged: HTTP %d, response=%s",
                 status, ack_bytes > 0 ? ack : "(empty)");
        return err;
    }

    err = storage_delete_done(path);
    if (err == ESP_OK) {
        *uploaded_bytes = file_size;
        ESP_LOGI(TAG, "ACK verified; local file removed: %s", path);
    }
    return err;
}

esp_err_t upload_process_pending(upload_summary_t *summary)
{
    if (!summary) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(summary, 0, sizeof(*summary));
    summary->last_error = ESP_OK;

    while (true) {
        char path[STORAGE_PATH_MAX] = {0};
        time_t recorded_at_epoch = 0;
        esp_err_t err = storage_get_next_done(path, sizeof(path),
                                              &recorded_at_epoch);
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
