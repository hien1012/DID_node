#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "cJSON.h"

/* ================== 請修改以下參數 ================== */
#define WIFI_SSID           "NEAT_2.4G"  // 實驗室 Wi-Fi 名稱
#define WIFI_PASS           "221b23251"  // 實驗室 Wi-Fi 密碼
#define SERVER_TIME_URL     "http://140.123.91.132:5000/time" // 你的 PC IP
/* ==================================================== */

static const char *TAG = "APP_MAIN";

// 定義事件群組與連線成功位元
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/* --- 1. Wi-Fi 事件處理回呼函式 --- */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi 啟動，嘗試連線至 AP...");
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi 連線失敗或斷線，重新嘗試連線...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "成功取得 IP 位址: " IPSTR, IP2STR(&event->ip_info.ip));
        // 設定連線成功旗標，喚醒主程式
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* --- 2. Wi-Fi 初始化函式 --- */
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

#define SERVER_UPLOAD_URL "http://140.123.91.132:5000/upload"

/* --- 3：資料上傳 Task --- */
static void http_post_data_task(void *pvParameters)
{
    ESP_LOGI("HTTP_POST", "開始封裝感測資料並上傳...");

    // 1. 取得已經校準好的當前時間
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);

    // 2. 使用 cJSON 建立要上傳的 JSON 物件
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device", "ESP32-Lab");
    cJSON_AddStringToObject(root, "timestamp", time_str);
    cJSON_AddNumberToObject(root, "sensor_value", 25.4); // 填入你的感測器變數
    
    // 將 cJSON 物件轉為字串
    char *post_data = cJSON_PrintUnformatted(root);

    // 3. 設定 HTTP POST 客戶端
    esp_http_client_config_t config = {
        .url = SERVER_UPLOAD_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    // 設定 Header 告訴伺服器這是 JSON
    esp_http_client_set_header(client, "Content-Type", "application/json");
    // 放入 Payload
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    // 4. 執行發送
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI("HTTP_POST", "資料上傳成功，伺服器回應狀態碼: %d", esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE("HTTP_POST", "資料上傳失敗: %s", esp_err_to_name(err));
    }

    // 5. 釋放記憶體 (非常重要，否則會 Memory Leak)
    cJSON_Delete(root);
    free(post_data);
    esp_http_client_cleanup(client);
    
    vTaskDelete(NULL);
}

/* --- 4. 獨立的 HTTP 請求 Task --- */
static void http_sync_time_task(void *pvParameters)
{
    ESP_LOGI("TIME_SYNC", "開始向伺服器 (%s) 請求時間...", SERVER_TIME_URL);

    esp_http_client_config_t config = {
        .url = SERVER_TIME_URL,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 5000,
        .disable_auto_redirect = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    char local_response_buffer[128] = {0};
    esp_err_t err = esp_http_client_open(client, 0);
    
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(client);
        int read_len = esp_http_client_read(client, local_response_buffer, sizeof(local_response_buffer) - 1);
        
        if (read_len > 0) {
            local_response_buffer[read_len] = '\0';
            ESP_LOGI("TIME_SYNC", "收到伺服器回應: %s", local_response_buffer);

            // 解析 JSON
            cJSON *json = cJSON_Parse(local_response_buffer);
            if (json != NULL) {
                cJSON *epoch_item = cJSON_GetObjectItem(json, "epoch");
                if (cJSON_IsNumber(epoch_item)) {
                    long epoch_time = (long)epoch_item->valuedouble;
                    
                    // 寫入 ESP32 RTC
                    struct timeval tv = { .tv_sec = epoch_time, .tv_usec = 0 };
                    settimeofday(&tv, NULL);
                    
                    // 設定台灣時區並列印驗證
                    setenv("TZ", "CST-8", 1);
                    tzset();
                    time_t now;
                    struct tm timeinfo;
                    time(&now);
                    localtime_r(&now, &timeinfo);
                    char time_str[64];
                    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);
                    
                    ESP_LOGI("TIME_SYNC", "系統時間更新成功: %s", time_str);
                    // 時間同步成功後，啟動上傳資料的 Task
                    xTaskCreate(&http_post_data_task, "http_post_data_task", 8192, NULL, 5, NULL);
                }
                cJSON_Delete(json);
            }
        }
    } else {
        ESP_LOGE("TIME_SYNC", "HTTP GET 失敗: %s", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(client);
    
    // 任務完成後自行刪除
    vTaskDelete(NULL);
}

/* --- 4. 主程式 --- */
void app_main(void)
{
    // 初始化 NVS (Wi-Fi 驅動必備儲存空間)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "初始化 Wi-Fi...");
    wifi_init_sta();

    // 阻塞等待，直到取得 IP 才會往下執行
    ESP_LOGI(TAG, "等待連接至 Wi-Fi 取得 IP...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // 確認連線後，啟動 HTTP 請求 Task
    ESP_LOGI(TAG, "網路就緒，啟動時間同步任務...");
    xTaskCreate(&http_sync_time_task, "http_sync_time_task", 8192, NULL, 5, NULL);
}