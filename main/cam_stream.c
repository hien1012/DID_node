/*
 * 相機串流測試 (debug)：每擷取一幀就直接從 frame buffer 用 TCP 送給 PC
 * (tools/cam_stream_view.py)，不經 SD 卡、不經 HTTP。
 *
 * 用途：判斷影像損壞 (docs/debug_sdcard.md #26) 發生在哪一段：
 *   串流畫面也壞 → 擷取當下就錯 (PCLK 取樣時序 / 資料線 / 驅動)
 *   串流畫面乾淨 → 問題在錄影路徑 (寫 SD 時的干擾、writer task、檔案內容)
 *
 * 封包 (little-endian)：24 位元組 header + width*height 位元組 GRAY8
 *   "CAMF" | u32 frame index | u16 width | u16 height | u32 length | i64 timestamp_us
 *
 * 拍一幀、傳一幀：相機匯流排運作時 Wi-Fi 幾乎送不出去 (#19、#25；實測
 * send 5 s 逾時 errno=11)，所以每幀複製到 PSRAM 後先暫停相機並停 XCLK，
 * 送完再恢復，並丟掉恢復後的前幾幀讓感測器穩定。擷取當下的電氣條件與
 * 錄影相同 (XCLK 開、DMA 運作)，只是沒有寫 SD；這個測試只看畫面有沒有壞，不看幀率。
 *
 * SD 負載 A/B (APP_CAM_STREAM_SD_LOAD)：背景 task 以錄影相同的節奏
 * (每 200 ms 寫 307200 B，32 KB 內部 DMA 緩衝) 寫 /sdcard/stream_load.bin，
 * 每 CAM_STREAM_SD_LOAD_PHASE_FRAMES 幀切換一次開／關。擷取時負載開著的幀
 * magic 為 "CAMS"，否則 "CAMF"。同一次開機、同一套接線下比較兩組的損壞率：
 *   只有 CAMS 壞 → 寫 SD 的電氣干擾 (接地／電源)
 *   兩組都乾淨 → 問題在錄影流程本身 (連續擷取 + writer)
 */
#include "cam_stream.h"

#include <errno.h>
#include <fcntl.h>
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
#include "node_storage.h"

#define CAM_STREAM_HEADER_BYTES      24U
#define CAM_STREAM_RETRY_MS          2000U
#define CAM_STREAM_SEND_TIMEOUT_SEC  5
#define CAM_STREAM_LOG_EVERY_FRAMES  10U
/* XCLK 恢復後丟棄的幀數：感測器重新起振與 AEC 收斂 (5 fps 下約 0.6 s) */
#define CAM_STREAM_SETTLE_FRAMES     3U

/* SD 負載：檔案放在根目錄，不會被當成錄影檔上傳或標成 broken */
#define CAM_STREAM_SD_LOAD_PATH          "/sdcard/stream_load.bin"
#define CAM_STREAM_SD_LOAD_PERIOD_MS     200U     /* 與錄影相同：5 fps，每幀寫一次 */
#define CAM_STREAM_SD_LOAD_FILE_BYTES    (64U * 1024U * 1024U)  /* 寫到這裡就從頭覆寫 */
#define CAM_STREAM_SD_LOAD_PHASE_FRAMES  20U      /* 每 20 幀切換負載開／關 */
#define CAM_STREAM_SD_LOAD_LOG_EVERY     25U
#define CAM_STREAM_SD_LOAD_TASK_STACK    4096U
#define CAM_STREAM_SD_LOAD_TASK_PRIORITY 5U       /* 與錄影的 cam_writer 相同 */

#if APP_CAM_STREAM_ENABLE
static const char *TAG = "CAM_STREAM";

/* 由串流迴圈切換，SD 負載 task 讀取 */
static volatile bool s_sd_load_on = false;

#if APP_CAM_STREAM_SD_LOAD
/*
 * 錄影時 cam_writer 的寫法：write() 32 KB 一段，從內部 DMA 緩衝直接寫，每 200 ms
 * 一幀 (307200 B)。內容用亂數，讓 DAT0 像寫真實影像時一樣頻繁切換。
 */
static void sd_load_task(void *argument)
{
    uint8_t *buffer = (uint8_t *)argument;
    uint32_t state = 0x12345678U;
    for (size_t i = 0; i < NODE_STORAGE_IO_BUFFER_BYTES; ++i) {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        buffer[i] = (uint8_t)state;
    }

    const int fd = open(CAM_STREAM_SD_LOAD_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        ESP_LOGE(TAG, "SD load: cannot open %s (errno=%d)", CAM_STREAM_SD_LOAD_PATH, errno);
        node_storage_io_buffer_release(buffer);
        vTaskDelete(NULL);
        return;
    }

    size_t file_offset = 0;
    uint32_t frames = 0;
    uint32_t errors = 0;
    int64_t busy_us = 0;
    int64_t max_us = 0;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAM_STREAM_SD_LOAD_PERIOD_MS));
        if (!s_sd_load_on) {
            continue;
        }
        if (file_offset + CAM_IMAGE_FRAME_BYTES > CAM_STREAM_SD_LOAD_FILE_BYTES) {
            (void)lseek(fd, 0, SEEK_SET);
            file_offset = 0;
        }
        const int64_t start_us = esp_timer_get_time();
        size_t remaining = CAM_IMAGE_FRAME_BYTES;
        while (remaining > 0U) {
            const size_t chunk = remaining < NODE_STORAGE_IO_BUFFER_BYTES ?
                                 remaining : NODE_STORAGE_IO_BUFFER_BYTES;
            const ssize_t written = write(fd, buffer, chunk);
            if (written <= 0) {
                errors++;
                ESP_LOGE(TAG, "SD load: write failed (errno=%d, %u errors)",
                         errno, (unsigned)errors);
                break;
            }
            remaining -= (size_t)written;
        }
        file_offset += CAM_IMAGE_FRAME_BYTES - remaining;
        const int64_t elapsed_us = esp_timer_get_time() - start_us;
        busy_us += elapsed_us;
        if (elapsed_us > max_us) {
            max_us = elapsed_us;
        }
        if (++frames % CAM_STREAM_SD_LOAD_LOG_EVERY == 0U) {
            ESP_LOGI(TAG, "SD load: %u frames written, avg=%" PRId64 " ms max=%" PRId64
                     " ms, errors=%u", (unsigned)frames,
                     busy_us / CAM_STREAM_SD_LOAD_LOG_EVERY / 1000, max_us / 1000,
                     (unsigned)errors);
            busy_us = 0;
            max_us = 0;
        }
    }
}

static esp_err_t start_sd_load(void)
{
    uint8_t *buffer = node_storage_io_buffer_acquire("stream_sd_load");
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    const BaseType_t core = 0;      /* 與錄影的 cam_writer 相同 (Core 0；單核也有效) */
    if (xTaskCreatePinnedToCore(sd_load_task, "stream_sd_load", CAM_STREAM_SD_LOAD_TASK_STACK,
                                buffer, CAM_STREAM_SD_LOAD_TASK_PRIORITY, NULL,
                                core) != pdPASS) {
        node_storage_io_buffer_release(buffer);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
#endif

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

static void build_header(uint8_t *header, uint32_t index, int64_t timestamp_us,
                         bool sd_load)
{
    const uint16_t width = (uint16_t)CAM_IMAGE_WIDTH;
    const uint16_t height = (uint16_t)CAM_IMAGE_HEIGHT;
    const uint32_t length = (uint32_t)CAM_IMAGE_FRAME_BYTES;
    /* ESP32-S3 是 little-endian，直接複製即為協定的位元組順序 */
    memcpy(&header[0], sd_load ? "CAMS" : "CAMF", 4);
    memcpy(&header[4], &index, sizeof(index));
    memcpy(&header[8], &width, sizeof(width));
    memcpy(&header[10], &height, sizeof(height));
    memcpy(&header[12], &length, sizeof(length));
    memcpy(&header[16], &timestamp_us, sizeof(timestamp_us));
}

/* 相機匯流排開／關：關閉時 XCLK 停止，OV7670 的 PCLK 與資料線一起安靜。 */
static esp_err_t set_camera_running(bool running)
{
    esp_err_t err;
    if (running) {
        err = cam_record_set_xclk_output(true);
        if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
            return err;
        }
        return cam_record_resume();
    }
    err = cam_record_pause();
    if (err != ESP_OK) {
        return err;
    }
    err = cam_record_set_xclk_output(false);
    return err == ESP_ERR_NOT_SUPPORTED ? ESP_OK : err;
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
        if (frame->len != CAM_IMAGE_FRAME_BYTES || frame->width != CAM_IMAGE_WIDTH ||
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
 * 進入與返回時相機都是暫停、XCLK 停止。回傳 ESP_OK = 只是連線中斷。
 */
static esp_err_t stream_frames(int sock, uint8_t *copy, uint32_t *frame_index)
{
    uint32_t timeouts = 0;
    uint32_t invalid_frames = 0;
    uint32_t window_frames = 0;
    int64_t window_start_us = esp_timer_get_time();

    for (;;) {
#if APP_CAM_STREAM_SD_LOAD
        /* 前 20 幀不寫 SD (對照組)，之後每 20 幀切換；穩定幀期間負載已在運作 */
        const bool sd_load = ((*frame_index / CAM_STREAM_SD_LOAD_PHASE_FRAMES) % 2U) == 1U;
        if (sd_load != s_sd_load_on) {
            ESP_LOGI(TAG, "SD load %s from frame %u", sd_load ? "ON" : "OFF",
                     (unsigned)*frame_index);
        }
        s_sd_load_on = sd_load;
#endif
        esp_err_t err = set_camera_running(true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Cannot resume camera: %s", esp_err_to_name(err));
            return err;
        }
        int64_t timestamp_us = 0;
        const esp_err_t capture_err = capture_frame(copy, CAM_STREAM_SETTLE_FRAMES,
                                                    &timestamp_us, &timeouts, &invalid_frames);
        /* 先停相機匯流排再送，Wi-Fi 才不受干擾 */
        err = set_camera_running(false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Cannot pause camera: %s", esp_err_to_name(err));
            return err;
        }
        if (capture_err != ESP_OK) {
            continue;   /* 逾時已記錄；重新恢復相機再試 */
        }

        uint8_t header[CAM_STREAM_HEADER_BYTES];
        build_header(header, *frame_index, timestamp_us, s_sd_load_on);
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
    /* 呼叫時相機可能在運作 (開機測速後)；連線前先讓匯流排安靜 */
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
#if APP_CAM_STREAM_SD_LOAD
    err = start_sd_load();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot start SD load task: %s", esp_err_to_name(err));
    } else {
        ESP_LOGW(TAG, "SD load A/B: toggles every %u frames, %u bytes per %u ms to %s",
                 (unsigned)CAM_STREAM_SD_LOAD_PHASE_FRAMES, (unsigned)CAM_IMAGE_FRAME_BYTES,
                 (unsigned)CAM_STREAM_SD_LOAD_PERIOD_MS, CAM_STREAM_SD_LOAD_PATH);
    }
#endif

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
