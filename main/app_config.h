#pragma once

#include "sdkconfig.h"

/* Edit these when moving to another site; this is the only place they live. */
#define APP_WIFI_SSID              "Strawberry"
#define APP_WIFI_PASSWORD          "10122001"
#define APP_SERVER_BASE_URL        "http://10.10.10.7:5000"

#define APP_SERVER_TIME_URL        APP_SERVER_BASE_URL "/time"
#define APP_SERVER_AUDIO_URL       APP_SERVER_BASE_URL "/upload/audio"
#define APP_SERVER_VIDEO_URL       APP_SERVER_BASE_URL "/upload/video"
#define APP_SERVER_HEARTBEAT_URL   APP_SERVER_BASE_URL "/heartbeat"

#define APP_DEVICE_ID_PREFIX       "node"

/* Camera and microphone record together for this long, once per period.
 * The first session starts right after the boot checks; the next ones start
 * one period after the previous start. */
#define APP_RECORD_DURATION_SEC    60U
#define APP_RECORD_PERIOD_SEC      (10U * 60U)

/* Files are created and preallocated this long before the session starts. */
#define APP_PREPARE_LEAD_MS        8000U

#define APP_HEARTBEAT_PERIOD_MS    (60U * 1000U)
#define APP_HTTP_TIMEOUT_MS        30000
#define APP_UPLOAD_RETRY_MS        10000

/* Debug: network throughput test at boot (net_bench.c). Run `iperf -s -i 1`
 * (iperf2) on the PC first. Set to 0 when done; it costs a few seconds per boot. */
#define APP_NET_BENCH_ENABLE       1
#define APP_NET_BENCH_HOST         "10.10.10.7"
#define APP_NET_BENCH_PORT         5001
#define APP_NET_BENCH_BYTES        (8U * 1024U * 1024U)

/* Wi-Fi recovery (see node_health.c). */
#define APP_WIFI_RECONNECT_MIN_MS        1000U
#define APP_WIFI_RECONNECT_MAX_MS        30000U
#define APP_HEARTBEAT_FAILS_BEFORE_RECONNECT 3U
#define APP_WIFI_RESTART_AFTER_OFFLINE_S (10U * 60U)
