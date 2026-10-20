#pragma once

#include "sdkconfig.h"

#define APP_WIFI_SSID              "Strawberry"
#define APP_WIFI_PASSWORD          "10122001"
#define APP_SERVER_BASE_URL        "http://10.10.10.7:5000"

#define APP_SERVER_TIME_URL        APP_SERVER_BASE_URL "/time"
#define APP_SERVER_AUDIO_URL       APP_SERVER_BASE_URL "/upload/audio"
#define APP_SERVER_VIDEO_URL       APP_SERVER_BASE_URL "/upload/video"
#define APP_SERVER_HEARTBEAT_URL   APP_SERVER_BASE_URL "/heartbeat"

#define APP_RECORD_DURATION_SEC    300
#define APP_RECORD_PERIOD_MS       (10U * 60U * 1000U)
#define APP_HEARTBEAT_PERIOD_MS    (1U * 60U * 1000U)
#define APP_HTTP_TIMEOUT_MS        30000
#define APP_UPLOAD_RETRY_MS        10000

#define CAM_RECORD_PERIOD_MS       (1U * 60U * 10000U)

/* Debug: network throughput test at boot (net_bench.c). Run `iperf -s -i 1`
 * (iperf2) on the PC first. Each run sends APP_NET_BENCH_BYTES over a plain TCP
 * socket (no HTTP, no SD). Set to 0 when done; it costs about 10-60 s per boot. */
#define APP_NET_BENCH_ENABLE       1
#define APP_NET_BENCH_HOST         "10.10.10.7"
#define APP_NET_BENCH_PORT         5001
#define APP_NET_BENCH_BYTES        (8U * 1024U * 1024U)

/* Debug (camera firmware only): stream frames straight to the PC instead of
 * recording (cam_stream.c). Run `python tools/cam_stream_view.py` (repository
 * root) on the PC first. While 1, the node never records or uploads. */
#define APP_CAM_STREAM_ENABLE      1
#define APP_CAM_STREAM_HOST        "10.10.10.7"
#define APP_CAM_STREAM_PORT        5002
