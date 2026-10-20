#pragma once

#include "esp_err.h"

/*
 * Debug: send APP_NET_BENCH_BYTES of zeros from RAM over a plain TCP socket to
 * an iperf2 server on the PC (`iperf -s -i 1`) and log the throughput.
 * No SD card and no HTTP, so it isolates the Wi-Fi/lwIP path of this firmware.
 * label only tags the log line.
 */
esp_err_t net_bench_run(const char *label);

/*
 * Debug: one log line with internal RAM free / largest block, DMA largest
 * block, PSRAM free and the internal low-water mark, tagged with label.
 */
void net_bench_log_heap(const char *label);
