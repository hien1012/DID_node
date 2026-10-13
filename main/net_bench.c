/*
 * 網路吞吐量測試 (debug)：用 socket 直接送 RAM 中的資料給 PC 的 iperf2 server。
 *
 * 對照組：
 *   ESP-IDF iperf 範例        約 7.5 MB/s (同一台 AP / PC)
 *   node_upload (HTTP + SD)   約 0.3 MB/s
 * 本測試若接近 iperf → 問題在 HTTP / 上傳寫法；若也很慢 → 問題在本韌體的
 * Wi-Fi / lwIP 設定或記憶體狀態。
 *
 * 內容全為 0：iperf2 server 會把前幾個位元組當成 client header 的旗標，
 * 0 代表沒有任何選項。
 */
#include "net_bench.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <sys/time.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "app_config.h"

#define NET_BENCH_CHUNK_BYTES   (16U * 1024U)   /* 與 node_upload 每次寫入大小相同 */
#define NET_BENCH_SEND_TIMEOUT_S 5

#if APP_NET_BENCH_ENABLE
static const char *TAG = "NET_BENCH";
#endif

esp_err_t net_bench_run(const char *label)
{
#if APP_NET_BENCH_ENABLE
    const size_t internal_free_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    /* 優先放內部 RAM；內部 RAM 不足時改用 PSRAM (lwIP 送出時會再複製，不影響測試意義) */
    const char *buffer_location = "internal";
    uint8_t *buffer = heap_caps_calloc(1, NET_BENCH_CHUNK_BYTES,
                                       MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buffer) {
        buffer_location = "PSRAM";
        buffer = heap_caps_calloc(1, NET_BENCH_CHUNK_BYTES,
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!buffer) {
        ESP_LOGE(TAG, "[%s] Cannot allocate %u bytes", label,
                 (unsigned)NET_BENCH_CHUNK_BYTES);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "[%s] Sending %u bytes to %s:%d (buffer in %s)", label,
             (unsigned)APP_NET_BENCH_BYTES, APP_NET_BENCH_HOST, APP_NET_BENCH_PORT,
             buffer_location);

    esp_err_t err = ESP_OK;
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "[%s] socket() failed: errno=%d", label, errno);
        heap_caps_free(buffer);
        return ESP_FAIL;
    }

    const struct timeval send_timeout = {.tv_sec = NET_BENCH_SEND_TIMEOUT_S, .tv_usec = 0};
    (void)setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));

    struct sockaddr_in server = {
        .sin_family = AF_INET,
        .sin_port = htons(APP_NET_BENCH_PORT),
    };
    if (inet_pton(AF_INET, APP_NET_BENCH_HOST, &server.sin_addr) != 1) {
        ESP_LOGE(TAG, "[%s] Invalid host %s", label, APP_NET_BENCH_HOST);
        close(sock);
        heap_caps_free(buffer);
        return ESP_ERR_INVALID_ARG;
    }
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) != 0) {
        ESP_LOGW(TAG, "[%s] Cannot connect to iperf server %s:%d (errno=%d); "
                 "is `iperf -s` running on the PC?",
                 label, APP_NET_BENCH_HOST, APP_NET_BENCH_PORT, errno);
        close(sock);
        heap_caps_free(buffer);
        return ESP_FAIL;
    }

    const int64_t started_us = esp_timer_get_time();
    int64_t max_chunk_us = 0;
    uint32_t slow_chunks = 0;          /* 單次 16 KB 送出超過 100 ms 的次數 */
    size_t sent_total = 0;
    while (sent_total < APP_NET_BENCH_BYTES) {
        const int64_t chunk_start_us = esp_timer_get_time();
        size_t offset = 0;
        while (offset < NET_BENCH_CHUNK_BYTES) {
            const int sent = send(sock, buffer + offset, NET_BENCH_CHUNK_BYTES - offset, 0);
            if (sent <= 0) {
                ESP_LOGE(TAG, "[%s] send() failed after %u bytes: errno=%d",
                         label, (unsigned)sent_total, errno);
                err = ESP_FAIL;
                break;
            }
            offset += (size_t)sent;
        }
        if (err != ESP_OK) {
            break;
        }
        sent_total += NET_BENCH_CHUNK_BYTES;
        const int64_t chunk_us = esp_timer_get_time() - chunk_start_us;
        if (chunk_us > max_chunk_us) {
            max_chunk_us = chunk_us;
        }
        if (chunk_us > 100000) {
            slow_chunks++;
        }
    }
    const int64_t elapsed_us = esp_timer_get_time() - started_us;
    shutdown(sock, SHUT_RDWR);
    close(sock);
    heap_caps_free(buffer);

    /* 速度 = 位元組 * 1000 / 微秒 = kB/s；Mbps = kB/s * 8 / 1000 */
    const int64_t rate_kbs = elapsed_us > 0 ? (int64_t)sent_total * 1000 / elapsed_us : 0;
    ESP_LOGI(TAG, "[%s] %u bytes in %" PRId64 " ms -> %" PRId64 " kB/s (%" PRId64
             ".%01" PRId64 " Mbps), max %" PRId64 " ms/16KB, slow(>100ms)=%u",
             label, (unsigned)sent_total, elapsed_us / 1000, rate_kbs,
             rate_kbs * 8 / 1000, (rate_kbs * 8 / 100) % 10,
             max_chunk_us / 1000, (unsigned)slow_chunks);
    ESP_LOGI(TAG, "[%s] internal heap: free before=%u now=%u, min since boot=%u",
             label, (unsigned)internal_free_before,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    return err;
#else
    (void)label;
    return ESP_OK;
#endif
}

void net_bench_log_heap(const char *label)
{
#if APP_NET_BENCH_ENABLE
    ESP_LOGI(TAG, "HEAP [%s]: internal free=%u largest=%u min=%u | DMA largest=%u | "
             "PSRAM free=%u",
             label,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
#else
    (void)label;
#endif
}
