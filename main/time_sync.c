#include "time_sync.h"

#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"

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

    /* open 已送出請求；server 取時間約在「送出」與「收到回應」的中點，用來補 HTTP 延遲 */
    const int64_t sent_us = esp_timer_get_time();
    (void)esp_http_client_fetch_headers(client);
    const int64_t received_us = esp_timer_get_time();
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

    /* 優先用微秒欄位 epoch_us；舊 server 只有整數秒 epoch */
    cJSON *root = cJSON_Parse(response);
    cJSON *us_item = root ? cJSON_GetObjectItemCaseSensitive(root, "epoch_us") : NULL;
    cJSON *epoch_item = root ? cJSON_GetObjectItemCaseSensitive(root, "epoch") : NULL;
    int64_t server_us = 0;
    if (cJSON_IsNumber(us_item)) {
        server_us = (int64_t)us_item->valuedouble;
    } else if (cJSON_IsNumber(epoch_item)) {
        server_us = (int64_t)epoch_item->valuedouble * 1000000LL;
    }
    cJSON_Delete(root);
    if (server_us < MIN_VALID_EPOCH * 1000000LL) {
        ESP_LOGE(TAG, "Invalid time response: %s", response);
        esp_http_client_cleanup(client);
        return ESP_ERR_INVALID_RESPONSE;
    }

    struct timeval before = {0};
    gettimeofday(&before, NULL);
    const int64_t now_us =
        server_us + (esp_timer_get_time() - (sent_us + received_us) / 2);
    const int64_t before_us = (int64_t)before.tv_sec * 1000000LL + before.tv_usec;
    struct timeval now = {
        .tv_sec = (time_t)(now_us / 1000000LL),
        .tv_usec = (suseconds_t)(now_us % 1000000LL),
    };
    err = settimeofday(&now, NULL) == 0 ? ESP_OK : ESP_FAIL;
    if (err == ESP_OK) {
        /* offset = 校正量，每輪重新校時時可看出這段期間的時鐘漂移 */
        ESP_LOGI(TAG, "Clock synchronized: epoch=%lld rtt=%lld ms offset=%lld ms",
                 (long long)now.tv_sec,
                 (long long)((received_us - sent_us) / 1000),
                 (long long)((now_us - before_us) / 1000));
    } else {
        ESP_LOGE(TAG, "settimeofday failed");
    }

    esp_http_client_cleanup(client);
    return err;
}
