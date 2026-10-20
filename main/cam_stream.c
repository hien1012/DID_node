/*
 * 相機串流測試 (debug)：每擷取一幀就直接從 frame buffer 用 TCP 送給 PC
 * (repository root 的 tools/cam_stream_view.py)，不經 SD 卡、不經 HTTP。
 *
 * 封包 (little-endian)：24 位元組 header + width*height 位元組 GRAY8
 *   "CAMF" | u32 frame index | u16 width | u16 height | u32 length | i64 timestamp_us
 *
 * 拍一幀、傳一幀：相機匯流排運作時會干擾 Wi-Fi，所以每幀複製到 PSRAM 後先
 * 暫停接收，送完再恢復，並丟掉恢復後的前幾幀讓感測器穩定。
 * 沿用 cam_record_pause/resume：ESP32 產生的 XCLK 也會停止/恢復，
 * OV2640 板載振盪器則會持續運作，不能宣稱感測器匯流排已停止。
 * 這個測試只看畫面有沒有壞，不看幀率。
 */
#include "cam_stream.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "app_config.h"
#include "cam_record.h"

#define CAM_STREAM_HEADER_BYTES      24U
#define CAM_STREAM_RETRY_MS          2000U
#define CAM_STREAM_SEND_TIMEOUT_SEC  5
#define CAM_STREAM_LOG_EVERY_FRAMES  10U
/* XCLK 恢復後丟棄的幀數：感測器重新起振 (5 fps 下約 0.6 s) */
#define CAM_STREAM_SETTLE_FRAMES     3U

#if APP_CAM_STREAM_ENABLE
static const char *TAG = "CAM_STREAM";

static bool send_all(int sock, const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    size_t offset = 0;
    while (offset < length) {
        const int sent = send(sock, bytes + offset, length - offset, 0);
        if (sent <= 0) {
            return false;
        }
        offset += (size_t)sent;
    }
    return true;
}

static int connect_viewer(void)
{
    const int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        return -1;
    }
    const struct timeval send_timeout = {.tv_sec = CAM_STREAM_SEND_TIMEOUT_SEC, .tv_usec = 0};
    (void)setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));

    struct sockaddr_in viewer = {
        .sin_family = AF_INET,
        .sin_port = htons(APP_CAM_STREAM_PORT),
    };
    if (inet_pton(AF_INET, APP_CAM_STREAM_HOST, &viewer.sin_addr) != 1) {
        ESP_LOGE(TAG, "Invalid host %s", APP_CAM_STREAM_HOST);
        close(sock);
        return -1;
    }
    if (connect(sock, (struct sockaddr *)&viewer, sizeof(viewer)) != 0) {
        ESP_LOGW(TAG, "Cannot connect to %s:%d (errno=%d); is "
                 "tools/cam_stream_view.py running on the PC?",
                 APP_CAM_STREAM_HOST, APP_CAM_STREAM_PORT, errno);
        close(sock);
        return -1;
    }
    ESP_LOGI(TAG, "Connected to viewer %s:%d", APP_CAM_STREAM_HOST, APP_CAM_STREAM_PORT);
    return sock;
}

static void build_header(uint8_t *header, uint32_t index, int64_t timestamp_us)
{
    const uint16_t width = (uint16_t)CAM_IMAGE_WIDTH;
    const uint16_t height = (uint16_t)CAM_IMAGE_HEIGHT;
    const uint32_t length = (uint32_t)CAM_IMAGE_FRAME_BYTES;
    /* ESP32 是 little-endian，直接複製即為協定的位元組順序 */
    memcpy(&header[0], "CAMF", 4);
    memcpy(&header[4], &index, sizeof(index));
    memcpy(&header[8], &width, sizeof(width));
    memcpy(&header[10], &height, sizeof(height));
    memcpy(&header[12], &length, sizeof(length));
    memcpy(&header[16], &timestamp_us, sizeof(timestamp_us));
}

/* 由 recorder 統一管理 DMA、VSYNC、XCLK 與恢復後的穩定等待。 */
static esp_err_t set_camera_running(bool running)
{
    return running ? cam_record_resume() : cam_record_pause();
}

/*
 * 取得一幀有效影像並複製到 copy (PSRAM)。先丟掉 settle_frames 幀
 * (XCLK 剛恢復時感測器尚未穩定)。
 */
static esp_err_t capture_frame(uint8_t *copy, uint32_t settle_frames, int64_t *timestamp_us,
                               uint32_t *timeouts, uint32_t *invalid_frames)
{
    for (;;) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == NULL) {
            (*timeouts)++;
            ESP_LOGW(TAG, "esp_camera_fb_get() timed out (%u so far)", (unsigned)*timeouts);
            if (*timeouts % 5U == 0U) {
                return ESP_ERR_TIMEOUT;
            }
            continue;
        }
        if (frame->format != PIXFORMAT_GRAYSCALE ||
            frame->len != CAM_IMAGE_FRAME_BYTES || frame->width != CAM_IMAGE_WIDTH ||
            frame->height != CAM_IMAGE_HEIGHT) {
            (*invalid_frames)++;
            ESP_LOGW(TAG, "Invalid frame %ux%u len=%u (%u so far)",
                     (unsigned)frame->width, (unsigned)frame->height,
                     (unsigned)frame->len, (unsigned)*invalid_frames);
            esp_camera_fb_return(frame);
            continue;
        }
        if (settle_frames > 0U) {
            settle_frames--;
            esp_camera_fb_return(frame);
            continue;
        }
        memcpy(copy, frame->buf, CAM_IMAGE_FRAME_BYTES);
        *timestamp_us = (int64_t)frame->timestamp.tv_sec * 1000000LL +
                        (int64_t)frame->timestamp.tv_usec;
        esp_camera_fb_return(frame);
        return ESP_OK;
    }
}

/*
 * 送到連線中斷為止；frame_index 跨連線累加，PC 端可看出漏了哪些幀。
 * 正常返回時相機接收已暫停；板載振盪器不受 ESP32 控制。
 * 回傳 ESP_OK = 只是連線中斷。
 */
static esp_err_t stream_frames(int sock, uint8_t *copy, uint32_t *frame_index)
{
    uint32_t timeouts = 0;
    uint32_t invalid_frames = 0;
    uint32_t window_frames = 0;
    int64_t window_start_us = esp_timer_get_time();

    for (;;) {
        esp_err_t err = set_camera_running(true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Cannot resume camera: %s", esp_err_to_name(err));
            return err;
        }
        int64_t timestamp_us = 0;
        const esp_err_t capture_err = capture_frame(copy, CAM_STREAM_SETTLE_FRAMES,
                                                    &timestamp_us, &timeouts, &invalid_frames);
        /* 先歸還影格並等待 HAL 暫停完成，再從 PSRAM 副本傳送。 */
        err = set_camera_running(false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Cannot pause camera: %s", esp_err_to_name(err));
            return err;
        }
        if (capture_err != ESP_OK) {
            continue;   /* 逾時已記錄；重新恢復相機再試 */
        }

        uint8_t header[CAM_STREAM_HEADER_BYTES];
        build_header(header, *frame_index, timestamp_us);
        const bool sent = send_all(sock, header, sizeof(header)) &&
                          send_all(sock, copy, CAM_IMAGE_FRAME_BYTES);
        if (!sent) {
            ESP_LOGW(TAG, "Viewer disconnected at frame %u (errno=%d)",
                     (unsigned)*frame_index, errno);
            return ESP_OK;
        }
        (*frame_index)++;

        if (++window_frames == CAM_STREAM_LOG_EVERY_FRAMES) {
            const int64_t now_us = esp_timer_get_time();
            const int64_t elapsed_us = now_us - window_start_us;
            const int64_t fps_x10 = elapsed_us > 0 ?
                                    (int64_t)window_frames * 10000000LL / elapsed_us : 0;
            const int64_t rate_kbs = elapsed_us > 0 ?
                                     (int64_t)window_frames * CAM_IMAGE_FRAME_BYTES * 1000LL /
                                     elapsed_us : 0;
            ESP_LOGI(TAG, "frame %u: %" PRId64 ".%" PRId64 " fps, %" PRId64
                     " kB/s, timeouts=%u invalid=%u",
                     (unsigned)*frame_index, fps_x10 / 10, fps_x10 % 10, rate_kbs,
                     (unsigned)timeouts, (unsigned)invalid_frames);
            window_frames = 0;
            window_start_us = now_us;
        }
    }
}
#endif

esp_err_t cam_stream_run(void)
{
#if APP_CAM_STREAM_ENABLE
    esp_err_t err = set_camera_running(false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot pause camera: %s", esp_err_to_name(err));
        return err;
    }
    uint8_t *copy = heap_caps_malloc(CAM_IMAGE_FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (copy == NULL) {
        ESP_LOGE(TAG, "Cannot allocate %u-byte frame copy", (unsigned)CAM_IMAGE_FRAME_BYTES);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW(TAG, "Debug stream mode: recording disabled, streaming to %s:%d",
             APP_CAM_STREAM_HOST, APP_CAM_STREAM_PORT);

    uint32_t frame_index = 0;
    for (;;) {
        const int sock = connect_viewer();
        if (sock >= 0) {
            err = stream_frames(sock, copy, &frame_index);
            shutdown(sock, SHUT_RDWR);
            close(sock);
            if (err != ESP_OK) {
                heap_caps_free(copy);
                return err;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CAM_STREAM_RETRY_MS));
    }
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
