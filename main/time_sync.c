#include "time_sync.h"

#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"

#include "app_config.h"

#define MIN_VALID_EPOCH 1704067200LL

static const char *TAG = "TIME_SYNC";

void time_sync_configure_timezone(void)
{
    setenv("TZ", "CST-8", 1);
    tzset();
    ESP_LOGI(TAG, "Timezone configured: UTC+8");
}

bool time_sync_is_valid(void)
{
    return (int64_t)time(NULL) >= MIN_VALID_EPOCH;
}

esp_err_t time_sync_from_server(void)
{
    esp_http_client_config_t config = {
        .url = APP_SERVER_TIME_URL,
        .method = HTTP_METHOD_GET,
        .timeout_ms = APP_HTTP_TIMEOUT_MS,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        esp_http_client_cleanup(client);
        return err;
    }

    (void)esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    char response[160] = {0};
    int bytes = esp_http_client_read_response(client, response,
                                               sizeof(response) - 1);
    esp_http_client_close(client);

    if (status != 200 || bytes <= 0) {
        ESP_LOGE(TAG, "Time request failed: HTTP %d, bytes=%d", status, bytes);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }
    response[bytes] = '\0';

    cJSON *root = cJSON_Parse(response);
    cJSON *epoch_item = root ? cJSON_GetObjectItemCaseSensitive(root, "epoch") : NULL;
    if (!cJSON_IsNumber(epoch_item) || epoch_item->valuedouble < MIN_VALID_EPOCH) {
        ESP_LOGE(TAG, "Invalid time response: %s", response);
        cJSON_Delete(root);
        esp_http_client_cleanup(client);
        return ESP_ERR_INVALID_RESPONSE;
    }

    struct timeval now = {
        .tv_sec = (time_t)epoch_item->valuedouble,
        .tv_usec = 0,
    };
    cJSON_Delete(root);
    err = settimeofday(&now, NULL) == 0 ? ESP_OK : ESP_FAIL;
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Clock synchronized: epoch=%lld", (long long)now.tv_sec);
    } else {
        ESP_LOGE(TAG, "settimeofday failed");
    }

    esp_http_client_cleanup(client);
    return err;
}
