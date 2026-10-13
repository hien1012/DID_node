#pragma once

#include "sdkconfig.h"

#define APP_WIFI_SSID              CONFIG_FIELD_WIFI_SSID
#define APP_WIFI_PASSWORD          CONFIG_FIELD_WIFI_PASSWORD
#define APP_SERVER_BASE_URL        CONFIG_FIELD_SERVER_BASE_URL

#define APP_SERVER_TIME_URL        APP_SERVER_BASE_URL "/time"
#define APP_SERVER_AUDIO_URL       APP_SERVER_BASE_URL "/upload/audio"
#define APP_SERVER_VIDEO_URL       APP_SERVER_BASE_URL "/upload/video"
#define APP_SERVER_HEARTBEAT_URL   APP_SERVER_BASE_URL "/heartbeat"

#define APP_RECORD_DURATION_SEC    300
#define APP_RECORD_PERIOD_MS       (10U * 60U * 1000U)
#define APP_HEARTBEAT_PERIOD_MS    (1U * 60U * 1000U)
#define APP_HTTP_TIMEOUT_MS        30000    // 音訊保留原值
#define CAM_HTTP_TIMEOUT_MS        120000    // 相機上傳：2 分鐘 (15MB @ 200KB/s ≈ 76s)
#define APP_UPLOAD_RETRY_MS        10000

#define CAM_RECORD_PERIOD_MS       (1U * 60U * 5000U) // 5min 拍攝一次10sec
