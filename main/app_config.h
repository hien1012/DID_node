#pragma once

#include "sdkconfig.h"

/* Edit these when moving to another site; this is the only place they live. */
#define APP_WIFI_SSID              "TP-LINK_AFA6"
#define APP_WIFI_PASSWORD          "93348942"
#define APP_SERVER_BASE_URL        "http://140.123.91.132:5000"

#define APP_SERVER_TIME_URL        APP_SERVER_BASE_URL "/time"
#define APP_SERVER_AUDIO_URL       APP_SERVER_BASE_URL "/upload/audio"
#define APP_SERVER_VIDEO_URL       APP_SERVER_BASE_URL "/upload/video"
#define APP_SERVER_HEARTBEAT_URL   APP_SERVER_BASE_URL "/heartbeat"

#define APP_DEVICE_ID_PREFIX       "node"

/* Camera and microphone record together for this long, once per period.
 * The first session starts right after the boot checks; the next ones start
 * one period after the previous start. */
#define APP_RECORD_DURATION_SEC    20U
#define APP_RECORD_PERIOD_SEC      (5U * 60U)

/* Debug: 0 = never initialize the microphone (no I2S clock, no WAV), so each
 * session records and uploads video only. Normal: 1. */
#define APP_MIC_ENABLE             1

/* Files are created and preallocated this long before the session starts. */
#define APP_PREPARE_LEAD_MS        8000U

#define APP_HEARTBEAT_PERIOD_MS    (60U * 1000U)
#define APP_HTTP_TIMEOUT_MS        30000
#define APP_UPLOAD_RETRY_MS        10000

/* Debug: network throughput test at boot (net_bench.c). Run `iperf -s -i 1`
 * (iperf2) on the PC first. Set to 0 when done; it costs a few seconds per boot. */
#define APP_NET_BENCH_ENABLE       1
#define APP_NET_BENCH_HOST         "140.123.91.132"
#define APP_NET_BENCH_PORT         5001
#define APP_NET_BENCH_BYTES        (8U * 1024U * 1024U)

/* Debug: per-round timing trace (timing_trace.c). Timestamps every stage of
 * prepare → record → upload in memory and prints one ====start==== ...
 * ====end==== block after the upload pass. 0 = compiled out. */
#define APP_TIMING_TRACE           0

/* Debug: stream frames straight to the PC instead of recording (cam_stream.c).
 * Run `python tools/cam_stream_view.py` on the PC first. While 1, the node
 * never records or uploads; set back to 0 when done. */
#define APP_CAM_STREAM_ENABLE      0
#define APP_CAM_STREAM_HOST        "140.123.91.132"
#define APP_CAM_STREAM_PORT        5002
/* 1: also write the SD card in the background like a recording, switching
 * it on/off every 20 frames, to A/B its effect on the image (cam_stream.c). */
#define APP_CAM_STREAM_SD_LOAD     1

/* Wi-Fi recovery (see node_health.c). */
#define APP_WIFI_RECONNECT_MIN_MS        1000U
#define APP_WIFI_RECONNECT_MAX_MS        30000U
#define APP_HEARTBEAT_FAILS_BEFORE_RECONNECT 3U
#define APP_WIFI_RESTART_AFTER_OFFLINE_S (10U * 60U)
