/*
 * 麥克風推論：連續錄音，每 2 s 一個視窗，一邊收下一個視窗一邊推論上一個。
 *
 *   Core 0：擷取 + 特徵
 *   mic_capture   (Core 0, 優先權 7)：I2S DMA 緩衝只有約 256 ms，所以優先權最高 (高於 logmel、上傳)。
 *     讀 I2S → PCM ×30 (飽和) → 填進目前視窗；滿了送給 audio_feature、換下一塊緩衝。
 *     I2S 的 DMA 中斷也在 Core 0 (init 在 app_main)，喚醒不必跨核心。
 *   audio_feature (Core 0, 優先權 3)：logmel → 特徵寫進這個視窗專屬的 s_features[buffer]，交給 audio_cnn。
 *   Core 1：CNN + 寫 SD
 *   audio_cnn     (Core 1, 優先權 3)：CNN → 結果立刻回傳；低信賴度把視窗交給 audio_writer。
 *   audio_writer  (Core 1, 優先權 2)：寫 WAV，寫完才還視窗。SD 偶爾一次寫入要幾百 ms 以上，
 *     不能算進推論的 2 s 預算；優先權低於 audio_cnn，新視窗一到就先推論。
 *
 * 三塊 64 KB 視窗緩衝在 PSRAM (收 1 + 推論 1 + 寫 SD 1)，索引在
 * s_free_windows → s_full_windows → s_feature_jobs → (s_save_jobs) → s_free_windows 間循環。
 * 特徵緩衝跟著視窗索引走，所以 logmel 可以先算下一個視窗，不會蓋掉 CNN 還在用的特徵。
 * 第一個 2 s 只有收資料；之後每一輪都是「收這一個、推論上一個」。
 * 三塊都在忙時丟掉新資料並計數，不會擋住 I2S。
 *
 * 上傳挑空檔 (mic_infer_wait_gap)：收滿的視窗都已推論、寫完，且離下一個視窗收滿還夠久，
 * 上傳才讀 SD、送下一段，避免和寫 WAV 搶 SD、和 logmel 搶 Core 0。
 *
 * PCM 定義與訓練一致 (TTGO_I2S_Record_Test/main/app_audio_infer.c)：
 * (raw32 >> 16) × 30 後飽和，存下來的 WAV 就是推論用的那份資料。
 */
#include "mic_infer.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_attr.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "audio_model.h"
#include "board_pins.h"
#include "infer_config.h"
#include "infer_time.h"
#include "result_report.h"
#include "sample_store.h"
#include "task_usage.h"

#if MIC_INTERFACE_I2S
#include "driver/i2s_std.h"
#else
#include "driver/i2s_pdm.h"
#endif

#define MIC_SAMPLE_RATE         AUDIO_LOGMEL_SAMPLE_RATE_HZ
#define WINDOW_SAMPLES          AUDIO_LOGMEL_INPUT_SAMPLES
#define WINDOW_BYTES            (WINDOW_SAMPLES * sizeof(int16_t))
#define WINDOW_DURATION_US      ((int64_t)WINDOW_SAMPLES * 1000000LL / MIC_SAMPLE_RATE)
/* 收 1 + 推論 1 + 寫 SD 1：寫 SD 慢的那一次，還有一整個視窗 (2 s) 的餘裕 */
#define WINDOW_COUNT            3U

/* --- WAV 格式 --- */
#define WAV_BITS_PER_SAMPLE     16U
#define WAV_CHANNELS            1U
#define WAV_HEADER_BYTES        44U

/* --- I2S DMA：8 x 512 frames = 256 ms 緩衝 --- */
#define DMA_DESC_NUM            8
#define DMA_FRAME_NUM           512
#if MIC_INTERFACE_I2S
#define RAW_BYTES_PER_SAMPLE    4U      /* 32-bit slot，PCM 在高 16 位元 */
#define PCM_SHIFT_BITS          16
#else
#define RAW_BYTES_PER_SAMPLE    2U      /* PDM2PCM 輸出即為 16-bit PCM */
#endif
#define READ_SAMPLES            1024U   /* 每次讀取 64 ms */
#define READ_TIMEOUT_MS         200
#define PCM_GAIN                30      /* 模型輸入合約的一部分，不可單獨改 */
/* 麥克風喚醒需要時間 (INMP441 約數百 ms)，這段資料丟掉 */
#define MIC_SETTLE_US           500000LL

#define WRITE_BUFFER_BYTES      (8U * 1024U)    /* SD 寫入暫存 (內部 RAM)，為 512 的倍數 */
#define META_BUFFER_BYTES       640U
#define TASK_USAGE_PERIOD_WINDOWS 30U   /* 每 30 個視窗 (1 分鐘) 回報一次 task CPU 占比 */

/* 上傳空檔：離下一個視窗收滿至少還有這麼久，才讀下一段 SD 送出 (一段 16 KB 約數十 ms) */
#define UPLOAD_GAP_MARGIN_US     300000LL

#define CAPTURE_TASK_STACK_BYTES 4096U
#define FEATURE_TASK_STACK_BYTES 6144U  /* logmel 的大陣列都在 s_feature_workspace (PSRAM) */
#define CNN_TASK_STACK_BYTES     8192U  /* TFLM Invoke + 浮點 log + result_report_push */
#define WRITER_TASK_STACK_BYTES  4096U  /* snprintf 浮點 + FATFS (LFN 在 heap) */
#define CAPTURE_TASK_PRIORITY    7U
#define FEATURE_TASK_PRIORITY    3U
#define CNN_TASK_PRIORITY        3U
#define WRITER_TASK_PRIORITY     2U
#if CONFIG_FREERTOS_UNICORE
#define CAPTURE_TASK_CORE        0
#define FEATURE_TASK_CORE        0
#define CNN_TASK_CORE            0
#define WRITER_TASK_CORE         0
#else
/* Core 0 有 Wi-Fi (優先權 23)；擷取每 64 ms 只讀一次、約 0.3 ms，DMA 還有 256 ms 緩衝 */
#define CAPTURE_TASK_CORE        0
#define FEATURE_TASK_CORE        0
#define CNN_TASK_CORE            1
#define WRITER_TASK_CORE         1      /* 和 CNN 同核心、優先權較低：新視窗的 CNN 先做 */
#endif

_Static_assert((WRITE_BUFFER_BYTES % 512U) == 0U, "WAV writes must contain complete SD sectors");

typedef struct __attribute__((packed)) {
    char     riff[4];
    uint32_t file_size_minus_8;
    char     wave[4];
    char     fmt_[4];
    uint32_t fmt_chunk_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char     data[4];
    uint32_t data_size;
} wav_header_t;

_Static_assert(sizeof(wav_header_t) == WAV_HEADER_BYTES, "WAV header is 44 bytes");

typedef struct {
    uint8_t buffer;
    uint32_t window;
    int64_t start_us;
    int64_t end_us;
    uint32_t overflows;
} full_window_t;

/* logmel 算完交給 audio_cnn；特徵在 s_features[window.buffer] */
typedef struct {
    full_window_t window;
    int64_t feature_start_us;
    int64_t feature_done_us;
    bool usage_open;        /* 這次開了 task_usage 推論區間，由 audio_cnn 結束並交還 */
} feature_job_t;

/* 低信賴視窗交給 audio_writer；視窗由 writer 寫完後歸還 */
typedef struct {
    uint8_t buffer;
    int64_t handed_over_us;
    infer_audio_record_t record;
} save_job_t;

static const char *TAG = "MIC_INFER";
static i2s_chan_handle_t s_rx_handle = NULL;
static volatile uint32_t s_overflow_count = 0;
static mic_infer_stats_t s_stats;
static volatile uint32_t s_dropped_windows = 0;

static int16_t *s_windows[WINDOW_COUNT];    /* PSRAM */
static int8_t *s_features[WINDOW_COUNT];    /* PSRAM，每塊 3,968 B，和視窗同索引 */
static void *s_feature_workspace = NULL;    /* PSRAM，只有 audio_feature 用 */
static size_t s_feature_workspace_bytes = 0;
static char *s_meta = NULL;                 /* PSRAM */

static StaticQueue_t s_free_queue_control;
static StaticQueue_t s_full_queue_control;
static StaticQueue_t s_feature_queue_control;
static StaticQueue_t s_save_queue_control;
static uint8_t s_free_queue_storage[WINDOW_COUNT * sizeof(uint8_t)];
static uint8_t s_full_queue_storage[WINDOW_COUNT * sizeof(full_window_t)];
static uint8_t s_feature_queue_storage[WINDOW_COUNT * sizeof(feature_job_t)];
static uint8_t s_save_queue_storage[WINDOW_COUNT * sizeof(save_job_t)];
static QueueHandle_t s_free_windows = NULL;
static QueueHandle_t s_full_windows = NULL;
static QueueHandle_t s_feature_jobs = NULL;  /* 以下深度都 = 視窗數，一定放得下 */
static QueueHandle_t s_save_jobs = NULL;

/*
 * task_usage 的推論區間 (logmel + CNN) 跨兩個 task、兩顆核心：audio_feature 開始、audio_cnn 結束。
 * task_usage 的推論統計沒有鎖，同一時間只能有一個 task 在動它，所以用這個旗標當通行證：
 * audio_feature 拿到才 begin，audio_cnn end / report 完才交還。CNN 落後時那一個視窗不列入統計。
 */
static portMUX_TYPE s_usage_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_usage_busy = false;

/*
 * 上傳空檔的判斷 (兩顆核心都會讀寫，用 spinlock 保護 64-bit 時間)：
 *   s_windows_in_flight：已送給 audio_feature、還沒還回 s_free_windows 的視窗數 (特徵、CNN 或寫 SD 中)
 *   s_next_window_end_us：預估下一個視窗收滿的時間；0 = 第一個視窗還沒收滿
 */
static portMUX_TYPE s_gap_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_windows_in_flight = 0;
static int64_t s_next_window_end_us = 0;

static WORD_ALIGNED_ATTR uint8_t s_read_buffer[READ_SAMPLES * RAW_BYTES_PER_SAMPLE];
static WORD_ALIGNED_ATTR uint8_t s_write_buffer[WRITE_BUFFER_BYTES];   /* 只有 audio_writer 用 */

static StaticTask_t s_capture_tcb;
static StaticTask_t s_feature_tcb;
static StaticTask_t s_cnn_tcb;
static StaticTask_t s_writer_tcb;
static StackType_t s_capture_stack[CAPTURE_TASK_STACK_BYTES];
static StackType_t s_feature_stack[FEATURE_TASK_STACK_BYTES];
static StackType_t s_cnn_stack[CNN_TASK_STACK_BYTES];
static StackType_t s_writer_stack[WRITER_TASK_STACK_BYTES];

/* 推論或寫 SD 結束：視窗回到空閒佇列 */
static void release_window(uint8_t buffer)
{
    portENTER_CRITICAL(&s_gap_lock);
    --s_windows_in_flight;
    portEXIT_CRITICAL(&s_gap_lock);
    (void)xQueueSend(s_free_windows, &buffer, 0);
}

/* 拿推論統計的通行證；拿到才可以 task_usage_begin() */
static bool usage_acquire(void)
{
    bool acquired = false;
    portENTER_CRITICAL(&s_usage_lock);
    if (!s_usage_busy) {
        s_usage_busy = true;
        acquired = true;
    }
    portEXIT_CRITICAL(&s_usage_lock);
    return acquired;
}

static void usage_release(void)
{
    portENTER_CRITICAL(&s_usage_lock);
    s_usage_busy = false;
    portEXIT_CRITICAL(&s_usage_lock);
}

static bool IRAM_ATTR i2s_rx_overflow_callback(i2s_chan_handle_t handle,
                                               i2s_event_data_t *event,
                                               void *user_ctx)
{
    (void)handle; (void)event; (void)user_ctx;
    s_overflow_count++;
    return false;
}

static esp_err_t i2s_init(void)
{
    i2s_chan_config_t chan_cfg = {
        .id            = I2S_NUM_0,
        .role          = I2S_ROLE_MASTER,
        .dma_desc_num  = DMA_DESC_NUM,
        .dma_frame_num = DMA_FRAME_NUM,
        .auto_clear    = true,
    };
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_rx_handle),
                        TAG, "i2s_new_channel failed");
#if MIC_INTERFACE_I2S
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_PIN_CLK,
            .ws   = MIC_PIN_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = MIC_PIN_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    std_cfg.slot_cfg.ws_pol    = false;
    std_cfg.slot_cfg.bit_shift = true;
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx_handle, &std_cfg),
                        TAG, "i2s_channel_init_std_mode failed");
#else
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                   I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = MIC_PIN_CLK,
            .din = MIC_PIN_DIN,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_pdm_rx_mode(s_rx_handle, &pdm_rx_cfg),
                        TAG, "i2s_channel_init_pdm_rx_mode failed");
#endif
    const i2s_event_callbacks_t cbs = {
        .on_recv       = NULL,
        .on_recv_q_ovf = i2s_rx_overflow_callback,
        .on_sent       = NULL,
        .on_send_q_ovf = NULL,
    };
    ESP_RETURN_ON_ERROR(i2s_channel_register_event_callback(s_rx_handle, &cbs, NULL),
                        TAG, "register cb failed");
    return ESP_OK;
}

static inline int16_t raw_to_pcm(size_t index)
{
#if MIC_INTERFACE_I2S
    const int32_t *raw = (const int32_t *)s_read_buffer;
    const int32_t gained = (int32_t)(int16_t)(raw[index] >> PCM_SHIFT_BITS) * PCM_GAIN;
#else
    const int16_t *raw = (const int16_t *)s_read_buffer;
    const int32_t gained = (int32_t)raw[index] * PCM_GAIN;
#endif
    return gained > INT16_MAX ? INT16_MAX : gained < INT16_MIN ? INT16_MIN : (int16_t)gained;
}

/* ==========================================================
 *  mic_capture：連續讀 I2S，填滿視窗就交給 audio_feature
 * ========================================================== */
static void mic_capture(void *arg)
{
    (void)arg;
    size_t bytes_read = 0;

    /* 喚醒期間的資料丟掉 */
    const int64_t settle_deadline_us = esp_timer_get_time() + MIC_SETTLE_US;
    while (esp_timer_get_time() < settle_deadline_us) {
        (void)i2s_channel_read(s_rx_handle, s_read_buffer, sizeof(s_read_buffer),
                               &bytes_read, READ_TIMEOUT_MS);
    }

    int current = -1;           /* 正在填的緩衝；-1 = 兩塊都在忙，資料丟掉 */
    size_t fill = 0;
    uint32_t window_index = 0;
    int64_t window_start_us = 0;
    uint32_t window_start_overflows = 0;

    while (true) {
        const esp_err_t err = i2s_channel_read(s_rx_handle, s_read_buffer, sizeof(s_read_buffer),
                                               &bytes_read, READ_TIMEOUT_MS);
        const int64_t read_done_us = esp_timer_get_time();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_channel_read failed: %s", esp_err_to_name(err));
            continue;
        }
        const size_t count = bytes_read / RAW_BYTES_PER_SAMPLE;
        for (size_t i = 0; i < count; ++i) {
            if (current < 0) {
                uint8_t next = 0;
                if (xQueueReceive(s_free_windows, &next, 0) != pdTRUE) {
                    break;      /* 還是沒有空緩衝：這次讀到的其餘資料丟掉 */
                }
                current = next;
                fill = 0;
            }
            if (fill == 0U) {
                /* read 返回時最後一個 sample 剛收到，由此回推視窗第一個 sample 的時間
                 * (有積壓時會偏晚，積壓上限約 256 ms) */
                window_start_us = read_done_us -
                                  (int64_t)(count - i) * 1000000LL / MIC_SAMPLE_RATE;
                window_start_overflows = s_overflow_count;
            }
            s_windows[current][fill++] = raw_to_pcm(i);
            if (fill == WINDOW_SAMPLES) {
                const full_window_t full = {
                    .buffer = (uint8_t)current,
                    .window = window_index++,
                    .start_us = window_start_us,
                    .end_us = read_done_us -
                              (int64_t)(count - i - 1U) * 1000000LL / MIC_SAMPLE_RATE,
                    .overflows = s_overflow_count - window_start_overflows,
                };
                /* 先記帳再送出：上傳看到 in_flight=0 時，一定沒有收滿未推論的視窗 */
                portENTER_CRITICAL(&s_gap_lock);
                ++s_windows_in_flight;
                s_next_window_end_us = full.end_us + WINDOW_DURATION_US;
                portEXIT_CRITICAL(&s_gap_lock);
                /* 深度等於緩衝數，一定放得下 */
                (void)xQueueSend(s_full_windows, &full, 0);
                current = -1;
                fill = 0;
                uint8_t next = 0;
                if (xQueueReceive(s_free_windows, &next, 0) == pdTRUE) {
                    current = next;
                } else {
                    ++s_dropped_windows;
                    ESP_LOGW(TAG, "All %u windows busy (inference or SD write too slow), dropped=%u",
                             (unsigned)WINDOW_COUNT, (unsigned)s_dropped_windows);
                }
            }
        }
    }
}

/* ==========================================================
 *  audio_writer 用的 WAV 寫入
 * ========================================================== */
static void fill_wav_header(wav_header_t *hdr, uint32_t data_size)
{
    memcpy(hdr->riff, "RIFF", 4);
    memcpy(hdr->wave, "WAVE", 4);
    memcpy(hdr->fmt_, "fmt ", 4);
    memcpy(hdr->data, "data", 4);
    hdr->file_size_minus_8 = data_size + 36U;
    hdr->fmt_chunk_size    = 16U;
    hdr->audio_format      = 1U;    /* PCM */
    hdr->num_channels      = WAV_CHANNELS;
    hdr->sample_rate       = MIC_SAMPLE_RATE;
    hdr->bits_per_sample   = WAV_BITS_PER_SAMPLE;
    hdr->byte_rate         = MIC_SAMPLE_RATE * WAV_CHANNELS * (WAV_BITS_PER_SAMPLE / 8U);
    hdr->block_align       = WAV_CHANNELS * (WAV_BITS_PER_SAMPLE / 8U);
    hdr->data_size         = data_size;
}

static esp_err_t write_all(int fd, const uint8_t *data, size_t bytes)
{
    size_t offset = 0;
    while (offset < bytes) {
        const ssize_t written = write(fd, data + offset, bytes - offset);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            return ESP_FAIL;
        }
        if (written == 0) {
            return ESP_FAIL;
        }
        offset += (size_t)written;
    }
    return ESP_OK;
}

/* PSRAM 不能直接給 SDMMC DMA，經內部 RAM 暫存寫入 */
static esp_err_t write_wav(int fd, const int16_t *pcm)
{
    wav_header_t hdr = {0};
    fill_wav_header(&hdr, (uint32_t)WINDOW_BYTES);
    memcpy(s_write_buffer, &hdr, sizeof(hdr));
    size_t fill = sizeof(hdr);
    const uint8_t *source = (const uint8_t *)pcm;
    size_t remaining = WINDOW_BYTES;
    while (remaining > 0U) {
        const size_t room = WRITE_BUFFER_BYTES - fill;
        const size_t take = remaining < room ? remaining : room;
        memcpy(s_write_buffer + fill, source, take);
        source += take;
        remaining -= take;
        fill += take;
        ESP_RETURN_ON_ERROR(write_all(fd, s_write_buffer, fill), TAG, "WAV write");
        fill = 0;
    }
    return ESP_OK;
}

static esp_err_t save_sample(const int16_t *pcm, const infer_audio_record_t *record,
                             uint32_t save_wait_us)
{
    sample_file_t file;
    ESP_RETURN_ON_ERROR(sample_store_begin(&file, SAMPLE_MEDIA_AUDIO, record->start_epoch_ms),
                        TAG, "begin sample");
    const int64_t write_start_us = esp_timer_get_time();
    const esp_err_t err = write_wav(file.fd, pcm);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WAV write failed: errno=%d", errno);
        sample_store_abort(&file);
        return err;
    }
    const int64_t write_us = esp_timer_get_time() - write_start_us;

    const int meta_bytes = snprintf(
        s_meta, META_BUFFER_BYTES,
        "{\"media\":\"audio\",\"window\":%" PRIu32 ",\"start_ms\":%" PRId64 ",\"end_ms\":%" PRId64
        ",\"sample_rate\":%u,\"samples\":%u,\"gain\":%d,\"threshold\":%.2f"
        ",\"queue_us\":%" PRIu32 ",\"feature_us\":%" PRIu32 ",\"invoke_us\":%" PRIu32
        ",\"save_wait_us\":%" PRIu32 ",\"write_us\":%" PRId64 ",\"overflows\":%" PRIu32
        ",\"pred\":%u,\"label\":\"%s\",\"conf\":%.4f"
        ",\"labels\":[\"%s\",\"%s\",\"%s\"],\"probs\":[%.4f,%.4f,%.4f]}",
        record->window, record->start_epoch_ms, record->end_epoch_ms,
        (unsigned)MIC_SAMPLE_RATE, (unsigned)WINDOW_SAMPLES, PCM_GAIN,
        (double)INFER_LOW_CONF_THRESHOLD, record->queue_us, record->feature_us, record->invoke_us,
        save_wait_us, write_us, record->overflows, (unsigned)record->pred, record->label, (double)record->conf,
        g_audio_class_names[0], g_audio_class_names[1], g_audio_class_names[2],
        (double)record->probs[0], (double)record->probs[1], (double)record->probs[2]);
    if (meta_bytes < 0 || meta_bytes >= (int)META_BUFFER_BYTES) {
        sample_store_abort(&file);
        return ESP_ERR_INVALID_SIZE;
    }
    return sample_store_commit(&file, s_meta);
}

/* ==========================================================
 *  audio_feature (Core 0)：logmel，特徵交給 audio_cnn
 * ========================================================== */
static void audio_feature(void *arg)
{
    (void)arg;
    full_window_t full;
    while (true) {
        (void)xQueueReceive(s_full_windows, &full, portMAX_DELAY);
        /* 一輪 = 上一個視窗到這個視窗 (約 2 s)：推論、寫 SD、上傳、閒置都算 */
        task_usage_cycle_mark();
        /* 推論一次 = logmel + CNN。統計本身的時間不算進 feature_us / invoke_us */
        const bool usage_open = usage_acquire();
        if (usage_open) {
            task_usage_begin();
        }
        const int64_t started_us = esp_timer_get_time();
        const int feature_err = audio_logmel_extract_quantized_pcm16(
            s_windows[full.buffer], WINDOW_SAMPLES, s_features[full.buffer], s_feature_workspace,
            s_feature_workspace_bytes, NULL);
        const int64_t feature_done_us = esp_timer_get_time();
        if (feature_err != AUDIO_LOGMEL_OK) {
            ESP_LOGE(TAG, "Logmel failed: %d", feature_err);
            if (usage_open) {
                usage_release();
            }
            release_window(full.buffer);
            continue;
        }
        const feature_job_t job = {
            .window = full,
            .feature_start_us = started_us,
            .feature_done_us = feature_done_us,
            .usage_open = usage_open,
        };
        (void)xQueueSend(s_feature_jobs, &job, 0);
    }
}

/* ==========================================================
 *  audio_cnn (Core 1)：CNN，結果立刻回傳，低信賴度交給 audio_writer
 * ========================================================== */
static void audio_cnn(void *arg)
{
    (void)arg;
    feature_job_t job;
    uint32_t usage_inferences = 0;
    while (true) {
        (void)xQueueReceive(s_feature_jobs, &job, portMAX_DELAY);
        const full_window_t *full = &job.window;
        const int64_t cnn_start_us = esp_timer_get_time();
        float probs[AUDIO_MODEL_CLASS_COUNT] = {0};
        int pred = -1;
        const esp_err_t model_err = audio_model_predict(s_features[full->buffer], probs, &pred);
        const int64_t cnn_done_us = esp_timer_get_time();
        if (model_err != ESP_OK || pred < 0) {
            ESP_LOGE(TAG, "CNN failed: %s", esp_err_to_name(model_err));
            if (job.usage_open) {
                usage_release();
            }
            release_window(full->buffer);
            continue;
        }
        if (job.usage_open) {
            task_usage_end();
        }

        infer_record_t record = {.type = INFER_RECORD_AUDIO};
        infer_audio_record_t *audio = &record.data.audio;
        audio->window = full->window;
        audio->start_epoch_ms = infer_epoch_ms_from_timer(full->start_us);
        audio->end_epoch_ms = infer_epoch_ms_from_timer(full->end_us);
        audio->queue_us = (uint32_t)(job.feature_start_us - full->end_us);
        audio->feature_us = (uint32_t)(job.feature_done_us - job.feature_start_us);
        audio->invoke_us = (uint32_t)(cnn_done_us - cnn_start_us);
        audio->overflows = full->overflows;
        audio->dropped_windows = s_dropped_windows;
        audio->pred = (uint8_t)pred;
        audio->label = g_audio_class_names[pred];
        audio->conf = probs[pred];
        memcpy(audio->probs, probs, sizeof(audio->probs));
        audio->low_conf = probs[pred] < INFER_LOW_CONF_THRESHOLD;
        result_report_push(&record);    /* 不等 SD：判斷結果馬上送出 */

        /* handoff = logmel 結束到 CNN 開始 (跨核心交接 + 等 Core 1) */
        ESP_LOGI(TAG, "w%u %s %.3f [%.3f %.3f %.3f] logmel=%u ms cnn=%u ms invoke=%.1f ms "
                 "handoff=%.1f ms%s",
                 (unsigned)audio->window, audio->label, (double)audio->conf,
                 (double)probs[0], (double)probs[1], (double)probs[2],
                 (unsigned)(audio->feature_us / 1000U), (unsigned)(audio->invoke_us / 1000U),
                 (double)audio_model_last_invoke_us() / 1000.0,
                 (double)(cnn_start_us - job.feature_done_us) / 1000.0,
                 audio->low_conf ? " LOW" : "");

        ++s_stats.windows;
        /* 推論統計的 report 也要在通行證內做；正常情況每個視窗都有，等於每 30 個視窗一次 */
        if (job.usage_open) {
            if (++usage_inferences >= TASK_USAGE_PERIOD_WINDOWS) {
                task_usage_report("audio");
                usage_inferences = 0;
            }
            usage_release();
        }
        if ((s_stats.windows % TASK_USAGE_PERIOD_WINDOWS) == 0U) {
            task_usage_cycle_report("audio cycle");
        }
        s_stats.last_feature_us = audio->feature_us;
        s_stats.last_invoke_us = audio->invoke_us;
        if (!audio->low_conf) {
            release_window(full->buffer);
            continue;
        }
        /* 寫 SD 交給 audio_writer，這裡馬上回去等下一個視窗；視窗由 writer 寫完才還 */
        ++s_stats.low_conf;
        const save_job_t save = {
            .buffer = full->buffer,
            .handed_over_us = esp_timer_get_time(),
            .record = *audio,
        };
        (void)xQueueSend(s_save_jobs, &save, 0);
    }
}

static void audio_writer(void *arg)
{
    (void)arg;
    save_job_t job;
    while (true) {
        (void)xQueueReceive(s_save_jobs, &job, portMAX_DELAY);
        const uint32_t save_wait_us = (uint32_t)(esp_timer_get_time() - job.handed_over_us);
        if (save_sample(s_windows[job.buffer], &job.record, save_wait_us) == ESP_OK) {
            ++s_stats.saved;
        }
        release_window(job.buffer);
    }
}

/* ==========================================================
 *  Public API
 * ========================================================== */
esp_err_t mic_infer_init(void)
{
    s_free_windows = xQueueCreateStatic(WINDOW_COUNT, sizeof(uint8_t),
                                        s_free_queue_storage, &s_free_queue_control);
    s_full_windows = xQueueCreateStatic(WINDOW_COUNT, sizeof(full_window_t),
                                        s_full_queue_storage, &s_full_queue_control);
    s_feature_jobs = xQueueCreateStatic(WINDOW_COUNT, sizeof(feature_job_t),
                                        s_feature_queue_storage, &s_feature_queue_control);
    s_save_jobs = xQueueCreateStatic(WINDOW_COUNT, sizeof(save_job_t),
                                     s_save_queue_storage, &s_save_queue_control);
    if (!s_free_windows || !s_full_windows || !s_feature_jobs || !s_save_jobs) {
        return ESP_ERR_NO_MEM;
    }

    const uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    for (uint8_t i = 0; i < WINDOW_COUNT; ++i) {
        s_windows[i] = heap_caps_malloc(WINDOW_BYTES, caps);
        s_features[i] = heap_caps_aligned_alloc(16, AUDIO_LOGMEL_MODEL_INPUT_VALUES, caps);
        if (!s_windows[i] || !s_features[i]) {
            return ESP_ERR_NO_MEM;
        }
        (void)xQueueSend(s_free_windows, &i, 0);
    }
    s_feature_workspace_bytes = audio_logmel_int8_workspace_size();
    s_feature_workspace = heap_caps_aligned_alloc(16, s_feature_workspace_bytes, caps);
    s_meta = heap_caps_malloc(META_BUFFER_BYTES, caps);
    if (!s_feature_workspace || !s_meta) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(audio_model_init(), TAG, "Audio model initialization failed");
    ESP_RETURN_ON_ERROR(i2s_init(), TAG, "I2S initialization failed");
    ESP_LOGI(TAG, "Microphone ready: %s, %u Hz mono, gain x%d, window %u samples",
             MIC_INTERFACE_I2S ? "I2S" : "PDM", (unsigned)MIC_SAMPLE_RATE, PCM_GAIN,
             (unsigned)WINDOW_SAMPLES);
    return ESP_OK;
}

esp_err_t mic_infer_start(void)
{
    if (!s_rx_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_handle), TAG, "i2s_channel_enable failed");
    /* 常駐 task，static：不佔用動態 heap，也不會每輪造成碎片。下游先建，上游送出時一定有人收 */
    if (!xTaskCreateStaticPinnedToCore(audio_writer, "audio_writer", WRITER_TASK_STACK_BYTES, NULL,
                                       WRITER_TASK_PRIORITY, s_writer_stack, &s_writer_tcb,
                                       WRITER_TASK_CORE) ||
        !xTaskCreateStaticPinnedToCore(audio_cnn, "audio_cnn", CNN_TASK_STACK_BYTES, NULL,
                                       CNN_TASK_PRIORITY, s_cnn_stack, &s_cnn_tcb,
                                       CNN_TASK_CORE) ||
        !xTaskCreateStaticPinnedToCore(audio_feature, "audio_feature", FEATURE_TASK_STACK_BYTES,
                                       NULL, FEATURE_TASK_PRIORITY, s_feature_stack,
                                       &s_feature_tcb, FEATURE_TASK_CORE) ||
        !xTaskCreateStaticPinnedToCore(mic_capture, "mic_capture", CAPTURE_TASK_STACK_BYTES, NULL,
                                       CAPTURE_TASK_PRIORITY, s_capture_stack, &s_capture_tcb,
                                       CAPTURE_TASK_CORE)) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void mic_infer_get_stats(mic_infer_stats_t *stats)
{
    if (!stats) {
        return;
    }
    *stats = s_stats;
    stats->dropped_windows = s_dropped_windows;
    stats->overflows = s_overflow_count;
}

bool mic_infer_wait_gap(uint32_t max_wait_ms)
{
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)max_wait_ms * 1000LL;
    while (true) {
        portENTER_CRITICAL(&s_gap_lock);
        const uint32_t in_flight = s_windows_in_flight;
        const int64_t next_end_us = s_next_window_end_us;
        portEXIT_CRITICAL(&s_gap_lock);
        const int64_t now_us = esp_timer_get_time();
        /* next_end_us = 0 (還沒收滿第一個視窗) 時差值為負，視為沒有空檔 */
        if (in_flight == 0U && next_end_us - now_us >= UPLOAD_GAP_MARGIN_US) {
            return true;
        }
        if (now_us >= deadline_us) {
            return false;
        }
        vTaskDelay(1);
    }
}
