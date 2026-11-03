#include "time_sync.h"

#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "wifi_manager.h"

#define MIN_VALID_EPOCH 1704067200LL
#define SLOT_MAX_SLEEP_MS 60000U   /* 分段睡，校時改變系統時間時不會睡過頭 */

static const char *TAG = "TIME_SYNC";
static int64_t s_last_sync_us = 0;   /* 上次校時成功的 esp_timer 時間；0 = 本次開機尚未校時 */

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
        s_last_sync_us = esp_timer_get_time();
        /* offset = 校正量，重新校時時可看出這段期間的時鐘漂移 */
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

void time_sync_refresh_if_due(void)
{
    if (s_last_sync_us != 0 &&
        esp_timer_get_time() - s_last_sync_us <
            (int64_t)APP_TIME_RESYNC_SEC * 1000000LL) {
        return;
    }
    /* 沒網路或失敗就等下一輪錄製後再試，不阻塞排程 */
    if (!wifi_manager_is_connected()) {
        ESP_LOGW(TAG, "Clock resync deferred: Wi-Fi disconnected");
        return;
    }
    const esp_err_t err = time_sync_from_server();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Clock resync failed: %s", esp_err_to_name(err));
    }
}

time_t time_sync_wait_next_slot(uint32_t period_sec, uint32_t min_lead_sec)
{
    if (period_sec == 0U) {
        return time(NULL);
    }
    /* UTC+8 是整小時偏移，epoch 的 5 分倍數就是當地時間的 5 分倍數 */
    struct timeval now = {0};
    gettimeofday(&now, NULL);
    const int64_t earliest = (int64_t)now.tv_sec + 1 + (int64_t)min_lead_sec;
    const int64_t slot = ((earliest + period_sec - 1) / period_sec) * period_sec;

    const time_t slot_epoch = (time_t)slot;
    struct tm local_time = {0};
    char slot_text[32] = "?";
    if (localtime_r(&slot_epoch, &local_time)) {
        strftime(slot_text, sizeof(slot_text), "%Y-%m-%d %H:%M:%S", &local_time);
    }
    ESP_LOGI(TAG, "Next recording at %s (in %lld s)",
             slot_text, (long long)(slot - (int64_t)now.tv_sec));

    for (;;) {
        gettimeofday(&now, NULL);
        const int64_t remaining_us =
            (slot - (int64_t)now.tv_sec) * 1000000LL - (int64_t)now.tv_usec;
        if (remaining_us <= 0) {
            break;
        }
        int64_t sleep_ms = remaining_us / 1000 + 1;
        if (sleep_ms > SLOT_MAX_SLEEP_MS) {
            sleep_ms = SLOT_MAX_SLEEP_MS;
        }
        /* +1 tick：pdMS_TO_TICKS 無條件捨去，避免提早醒來讓檔名落在前一秒 */
        vTaskDelay(pdMS_TO_TICKS((uint32_t)sleep_ms) + 1U);
    }
    return slot_epoch;
}
