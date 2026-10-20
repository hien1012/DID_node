#pragma once

#include "sdkconfig.h"

#define APP_WIFI_SSID              "TP-LINK_AFA6"
#define APP_WIFI_PASSWORD          "93348942"
#define APP_SERVER_BASE_URL        "http://140.123.91.132:5000"

#define APP_SERVER_TIME_URL        APP_SERVER_BASE_URL "/time"
#define APP_SERVER_AUDIO_URL       APP_SERVER_BASE_URL "/upload/audio"
#define APP_SERVER_VIDEO_URL       APP_SERVER_BASE_URL "/upload/video"
#define APP_SERVER_HEARTBEAT_URL   APP_SERVER_BASE_URL "/heartbeat"

#define APP_RECORD_DURATION_SEC    20
#define APP_RECORD_PERIOD_MS       (5U * 60U * 1000U)
#define APP_HEARTBEAT_PERIOD_MS    (1U * 60U * 1000U)
#define APP_HTTP_TIMEOUT_MS        30000
#define APP_UPLOAD_RETRY_MS        10000

#define CAM_RECORD_PERIOD_MS       APP_RECORD_PERIOD_MS
