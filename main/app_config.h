#pragma once

#include "sdkconfig.h"

#define APP_WIFI_SSID              "Strawberry"
#define APP_WIFI_PASSWORD          "10122001"
#define APP_SERVER_BASE_URL        "http://10.10.10.7:5000"

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

/* 開機網路 A/B 測試：PC 執行 iperf2 `iperf -s -i 1`。
 * HOST 必須是節點可連線的 PC IPv4；每個狀態送 8 MiB，會延後正式錄製。
 * 設為 0 即跳過測速與周邊時脈切換，保留正常啟動流程。 */
#ifndef APP_NET_BENCH_ENABLE
#define APP_NET_BENCH_ENABLE       1
#endif
#define APP_NET_BENCH_HOST         "10.10.10.7"
#define APP_NET_BENCH_PORT         5001
#define APP_NET_BENCH_BYTES        (8U * 1024U * 1024U)

/* 相機除錯串流：PC 執行 `python tools/cam_stream_view.py`。
 * 啟用時先補傳開機舊檔，再持續串流；不進入定時錄影流程。
 * 設為 0 即恢復原有相機錄影排程。HOST 請改為接收影像的 PC IPv4。 */
#ifndef APP_CAM_STREAM_ENABLE
#define APP_CAM_STREAM_ENABLE      0
#endif
#define APP_CAM_STREAM_HOST        "10.10.10.7"
#define APP_CAM_STREAM_PORT        5002
