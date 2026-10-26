/*
 * 麥克風錄音：I2S0 (標準 I2S 或 PDM) -> audio block 佇列 -> SD 卡 WAV。
 *
 * 與 cam_record.c 相同的三段式介面 (prepare / start / finish)，由 rec_session.c
 * 以同一個觸發時間點同步啟動。I2S 通道在兩次錄音之間持續運轉，
 * 讓麥克風保持穩定；觸發點之前的資料直接讀出丟棄。
 *
 * 管線：
 *   mic_capture (Core 1, 優先權 7)：I2S DMA 緩衝只有約 256 ms，因此優先權高於相機擷取。
 *   mic_writer  (Core 0, 優先權 5)：套用增益，累積 8 KB 後整塊寫入預先配置好的 WAV。
 *   兩者之間有 16 個 1024-sample block (約 1 秒)，吸收 SD 卡寫入延遲。
 */
#include "mic_record.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "board_pins.h"
#include "timing_trace.h"

#if MIC_INTERFACE_I2S
#include "driver/i2s_std.h"
#else
#include "driver/i2s_pdm.h"
#endif

/* --- WAV 格式 --- */
#define WAV_BITS_PER_SAMPLE     16U
#define WAV_CHANNELS            1U
#define WAV_HEADER_BYTES        44U
#define MIC_TARGET_SAMPLES      (MIC_SAMPLE_RATE * MIC_RECORD_DURATION_SEC)  /* 960,000 samples */
#define MIC_DATA_BYTES          (MIC_TARGET_SAMPLES * sizeof(int16_t))
#define MIC_FILE_BYTES          (WAV_HEADER_BYTES + MIC_DATA_BYTES)

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

/* timing trace：一個 DMA 描述元的資料量 (mono，每個 frame 一個 sample) 與完成時間環形緩衝 */
#define DMA_DESC_BYTES          ((uint32_t)DMA_FRAME_NUM * RAW_BYTES_PER_SAMPLE)
#define DMA_TIME_RING           16U     /* > 描述元數 + 每次讀取的描述元數，積壓滿也不會被覆蓋 */

/* PCM 增益倍數 (沿用舊版設定) */
#define PCM_GAIN                30

/* --- capture -> writer block pool --- */
#define AUDIO_BLOCK_SAMPLES     1024U
#define AUDIO_BLOCK_COUNT       16U
#define WRITE_BUFFER_BYTES      (8U * 1024U)    /* 每次 SD 寫入大小，為 512 的倍數 (256 ms PCM) */

#define CAPTURE_TASK_STACK_BYTES 4096U
#define WRITER_TASK_STACK_BYTES  4096U
#define CAPTURE_TASK_PRIORITY    7U
#define WRITER_TASK_PRIORITY     5U
#if CONFIG_FREERTOS_UNICORE
#define CAPTURE_TASK_CORE        0
#define WRITER_TASK_CORE         0
#else
#define CAPTURE_TASK_CORE        1
#define WRITER_TASK_CORE         0
#endif

#define CAPTURE_DONE_BIT        BIT0
#define WRITER_DONE_BIT         BIT1

_Static_assert((WRITE_BUFFER_BYTES % 512U) == 0U,
               "WAV writes must contain complete SD sectors");
_Static_assert(MIC_FILE_BYTES <= INT32_MAX,
               "The preallocated WAV must fit in off_t");

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
_Static_assert(((READ_SAMPLES * RAW_BYTES_PER_SAMPLE) % DMA_DESC_BYTES) == 0U,
               "Each read must consume whole DMA descriptors (timing trace)");
_Static_assert(DMA_TIME_RING >= DMA_DESC_NUM + (READ_SAMPLES * RAW_BYTES_PER_SAMPLE) / DMA_DESC_BYTES,
               "DMA time ring must cover a full backlog plus one read");

typedef struct {
    size_t  sample_count;
    int64_t queued_us;      /* 填滿送進 full queue 的時間 (timing trace：排隊時間) */
    int16_t samples[AUDIO_BLOCK_SAMPLES];
} audio_block_t;

typedef struct {
    node_recording_t storage;       /* undone -> done/broken 檔案生命週期 */
    int64_t trigger_us;
    int64_t first_sample_us;
    int64_t writer_end_us;
    uint32_t samples_captured;
    uint32_t samples_written;
    uint32_t overflows_recording;   /* 收滿 sample 那一刻的 overflow 數 (之後的是時脈停止前的空窗) */
    esp_err_t capture_result;
    esp_err_t writer_result;
    esp_err_t start_error;
    bool prepared;
    bool started;
} mic_session_t;

static const char *TAG = "MIC_RECORD";
static i2s_chan_handle_t s_rx_handle = NULL;
static bool s_initialized = false;
static volatile uint32_t s_overflow_count = 0;
static mic_session_t s_session;

/*
 * timing trace：I2S DMA 描述元完成時間。序號從時脈啟動起算 (i2s_channel_enable 會清空 driver 的 msg queue)。
 * s_dma_done_seq / s_dma_done_us / s_dma_drop_count 由中斷寫入；s_dma_bytes_read 只有 mic_capture 寫。
 */
static volatile uint32_t s_dma_done_seq = 0;
static volatile uint32_t s_dma_drop_count = 0;
static volatile int64_t s_dma_done_us[DMA_TIME_RING];
static uint32_t s_dma_bytes_read = 0;

static StaticQueue_t s_free_queue_control;
static StaticQueue_t s_full_queue_control;
static uint8_t s_free_queue_storage[AUDIO_BLOCK_COUNT * sizeof(audio_block_t *)];
static uint8_t s_full_queue_storage[(AUDIO_BLOCK_COUNT + 1U) * sizeof(audio_block_t *)];
static QueueHandle_t s_free_blocks = NULL;
static QueueHandle_t s_full_blocks = NULL;
static StaticEventGroup_t s_events_control;
static EventGroupHandle_t s_events = NULL;

static audio_block_t *s_blocks = NULL;     /* PSRAM：只有 CPU 存取，不走 DMA，省下內部 RAM */
static WORD_ALIGNED_ATTR uint8_t s_read_buffer[READ_SAMPLES * RAW_BYTES_PER_SAMPLE];
static WORD_ALIGNED_ATTR uint8_t s_write_buffer[WRITE_BUFFER_BYTES];

/* ==========================================================
 *  WAV / 檔案工具
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
        ssize_t written = write(fd, data + offset, bytes - offset);
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

/* ==========================================================
 *  I2S
 * ========================================================== */
static bool IRAM_ATTR i2s_rx_overflow_callback(i2s_chan_handle_t handle,
                                                i2s_event_data_t *event,
                                                void *user_ctx)
{
    (void)handle; (void)event; (void)user_ctx;
    s_overflow_count++;
    s_dma_drop_count++;     /* driver 丟掉最舊的描述元 (timing trace 對序號用) */
    return false;
}

/* 每個 DMA 描述元填滿時呼叫 (每 32 ms)：記下完成時間 */
static bool IRAM_ATTR i2s_rx_done_callback(i2s_chan_handle_t handle,
                                           i2s_event_data_t *event,
                                           void *user_ctx)
{
    (void)handle; (void)event; (void)user_ctx;
    const uint32_t seq = s_dma_done_seq;
    s_dma_done_us[seq % DMA_TIME_RING] = esp_timer_get_time();
    s_dma_done_seq = seq + 1U;
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
        .on_recv       = i2s_rx_done_callback,
        .on_recv_q_ovf = i2s_rx_overflow_callback,
        .on_sent       = NULL,
        .on_send_q_ovf = NULL,
    };
    ESP_RETURN_ON_ERROR(i2s_channel_register_event_callback(s_rx_handle, &cbs, NULL),
                        TAG, "register cb failed");
    /* 通道建立後保持停止；錄音準備時才由 set_clock_enabled(true) 啟動 */
    return ESP_OK;
}

/*
 * 開關 I2S 通道 (類似相機的 cam_stop / cam_start)。
 * 停止時 BCLK (PDM 為 CLK) 與 WS 不再輸出、DIN 不再取樣，麥克風因沒有時脈而休眠；
 * 減少非錄音期間的時脈輻射 (上傳時 Wi-Fi 收訊會被時脈干擾)。
 * DMA 緩衝仍保留 (不釋放記憶體)，下次啟動不需重新配置。
 */
static esp_err_t set_clock_enabled(bool enable)
{
    static bool s_clock_enabled = false;
    if (enable == s_clock_enabled) {
        return ESP_OK;
    }
    if (enable) {
        /* 通道停止中、沒有中斷：重設 timing trace 的描述元序號 */
        s_dma_done_seq = 0;
        s_dma_drop_count = 0;
        s_dma_bytes_read = 0;
    }
    esp_err_t err = enable ? i2s_channel_enable(s_rx_handle)
                           : i2s_channel_disable(s_rx_handle);
    if (err == ESP_OK) {
        s_clock_enabled = enable;
        ESP_LOGI(TAG, "I2S clock %s", enable ? "started" : "stopped");
    } else {
        ESP_LOGE(TAG, "i2s_channel_%s failed: %s", enable ? "enable" : "disable",
                 esp_err_to_name(err));
    }
    return err;
}

static inline int16_t raw_to_pcm(size_t index)
{
#if MIC_INTERFACE_I2S
    const int32_t *raw = (const int32_t *)s_read_buffer;
    return (int16_t)(raw[index] >> PCM_SHIFT_BITS);
#else
    const int16_t *raw = (const int16_t *)s_read_buffer;
    return raw[index];
#endif
}

/*
 * timing trace：一次 i2s_channel_read 返回後統計 DMA 層。
 * driver 的 msg queue 依完成順序存放描述元；被讀走 (pop) 或 overflow 丟棄 (drop) 都從最舊的開始，
 * 所以「第 k 個被移出的描述元」就是「第 k 個完成的描述元」。
 * 這次讀到的描述元序號 = [移出總數(讀之前), 移出總數(讀之後) − 1]。
 * overflow 與讀取交錯時序號會差一兩個，只在 overflow 期間不準。
 */
static void trace_dma_read(uint32_t popped_before, int64_t returned_us, uint32_t read_index)
{
    const uint32_t done = s_dma_done_seq;
    const uint32_t drops = s_dma_drop_count;
    const uint32_t removed = s_dma_bytes_read / DMA_DESC_BYTES + drops;
    if (removed == 0U || removed > done) {
        return;
    }
    const uint32_t last = removed - 1U;
    if (done - last > DMA_TIME_RING) {
        return;     /* 完成時間已被環形緩衝覆蓋 */
    }
    /* 這次讀取交出的最後一個描述元：完成 → 交給 mic_capture 的延遲。
     * read 有等待時約等於中斷到 task 的喚醒延遲；有積壓時就是資料在 DMA 裡放了多久 */
    timing_trace_stat(TT_STAT_MIC_DMA_TO_READ,
                      returned_us - s_dma_done_us[last % DMA_TIME_RING], read_index);
    for (uint32_t k = popped_before + drops; k <= last; ++k) {
        if (k == 0U || done - (k - 1U) > DMA_TIME_RING) {
            continue;
        }
        timing_trace_stat(TT_STAT_MIC_DMA_INTERVAL,
                          s_dma_done_us[k % DMA_TIME_RING] -
                              s_dma_done_us[(k - 1U) % DMA_TIME_RING],
                          k);
    }
    timing_trace_gauge_max(TT_GAUGE_MIC_DMA_BACKLOG_PEAK, done - removed);
}

/* ==========================================================
 *  mic_capture 任務：I2S DMA → PCM block
 *  運行於：CPU Core 1，優先權 7 (錄製中最高)；每次錄音由 mic_record_start() 建立，結束後自刪。
 *  職責：
 *   1. 觸發點之前持續 i2s_channel_read() 並丟棄，讓 DMA 只留觸發點之後的資料。
 *   2. 每次讀 READ_SAMPLES (64 ms)。I2S 驅動的 DMA 環只有約 256 ms，沒及時讀
 *      會覆蓋最舊的資料，所以優先權最高、迴圈裡不做檔案操作。
 *   3. raw_to_pcm() 把原始 slot (標準 I2S 為 32-bit，PDM 已是 16-bit) 轉成 16-bit PCM，填進從 s_free_blocks 取得的 block；
 *      填滿 AUDIO_BLOCK_SAMPLES 就送進 s_full_blocks，再取下一個空 block。
 *   4. 收滿 MIC_TARGET_SAMPLES 後送出最後半滿的 block，記下錄製期間的 overflow 次數，
 *      再送 NULL sentinel 通知 mic_writer 結束，設 CAPTURE_DONE_BIT。
 * ========================================================== */
static void mic_capture(void *arg)
{
    (void)arg;
    size_t bytes_read = 0;
    audio_block_t *blk = NULL;
    uint32_t read_index = 0;    /* timing trace：觸發點後第幾次讀取 */
    uint32_t block_index = 0;   /* timing trace：觸發點後第幾個 block */

    /* 觸發點之前持續讀取並丟棄，讓 DMA 中只剩觸發點之後的資料 */
    int64_t now_us = esp_timer_get_time();
    while (now_us < s_session.trigger_us) {
        esp_err_t err = i2s_channel_read(s_rx_handle, s_read_buffer,
                                         sizeof(s_read_buffer), &bytes_read,
                                         READ_TIMEOUT_MS);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            s_session.capture_result = err;
            goto finish;
        }
        s_dma_bytes_read += (uint32_t)bytes_read;
        now_us = esp_timer_get_time();
    }
    s_overflow_count = 0;
    s_session.first_sample_us = now_us;
    timing_trace_mark_at(TT_MARK_MIC_FIRST_SAMPLE, now_us);

    xQueueReceive(s_free_blocks, &blk, portMAX_DELAY);
    blk->sample_count = 0;

    while (s_session.samples_captured < MIC_TARGET_SAMPLES) {
        const uint32_t popped_before = s_dma_bytes_read / DMA_DESC_BYTES;
        const int64_t read_start_us = esp_timer_get_time();
        esp_err_t err = i2s_channel_read(s_rx_handle, s_read_buffer,
                                         sizeof(s_read_buffer), &bytes_read,
                                         READ_TIMEOUT_MS);
        const int64_t read_done_us = esp_timer_get_time();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_channel_read failed: %s", esp_err_to_name(err));
            s_session.capture_result = err;
            break;
        }
        s_dma_bytes_read += (uint32_t)bytes_read;
        trace_dma_read(popped_before, read_done_us, read_index);
        timing_trace_stat(TT_STAT_MIC_READ_BLOCKED, read_done_us - read_start_us, read_index);

        int64_t free_wait_us = 0;   /* 這次轉換中等空 block 的時間，從 convert 扣掉 */
        const size_t count = bytes_read / RAW_BYTES_PER_SAMPLE;
        for (size_t i = 0; i < count &&
                           s_session.samples_captured < MIC_TARGET_SAMPLES; ++i) {
            blk->samples[blk->sample_count++] = raw_to_pcm(i);
            s_session.samples_captured++;
            if (blk->sample_count >= AUDIO_BLOCK_SAMPLES) {
                blk->queued_us = esp_timer_get_time();
                xQueueSend(s_full_blocks, &blk, portMAX_DELAY);
                timing_trace_gauge_max(TT_GAUGE_MIC_FULLQ_PEAK,
                                       (uint32_t)uxQueueMessagesWaiting(s_full_blocks));
                const int64_t wait_start_us = esp_timer_get_time();
                xQueueReceive(s_free_blocks, &blk, portMAX_DELAY);
                const int64_t waited_us = esp_timer_get_time() - wait_start_us;
                free_wait_us += waited_us;
                timing_trace_stat(TT_STAT_MIC_FREE_WAIT, waited_us, block_index);
                block_index++;
                blk->sample_count = 0;
            }
        }
        timing_trace_stat(TT_STAT_MIC_CONVERT,
                          esp_timer_get_time() - read_done_us - free_wait_us, read_index);
        read_index++;
    }

    if (blk && blk->sample_count > 0) {
        blk->queued_us = esp_timer_get_time();
        xQueueSend(s_full_blocks, &blk, portMAX_DELAY);
    }
    /* 收滿：之後到 finish 停止時脈前沒有人讀 I2S，這段的 overflow 不影響檔案 */
    s_session.overflows_recording = s_overflow_count;
    timing_trace_mark(TT_MARK_MIC_CAPTURE_DONE);

finish:
    blk = NULL;     /* sentinel：通知 mic_writer 結束 */
    xQueueSend(s_full_blocks, &blk, portMAX_DELAY);
    xEventGroupSetBits(s_events, CAPTURE_DONE_BIT);
    vTaskDelete(NULL);
}

/* ==========================================================
 *  mic_writer 任務：PCM block → WAV 檔
 *  運行於：CPU Core 0，優先權 5；每次錄音由 mic_record_start() 建立，結束後自刪。
 *  職責：
 *   1. 先在寫入緩衝放佔位 WAV header (大小 0)。
 *   2. 從 s_full_blocks 取 block，每個 sample 乘 PCM_GAIN 並飽和到 int16，
 *      累積到 WRITE_BUFFER_BYTES (8 KB，512 的倍數) 才整塊 write() 進預配置的 WAV，
 *      處理完把 block 還回 s_free_blocks 給 mic_capture 重用。
 *   3. 收到 NULL sentinel：寫出剩餘資料，回寫實際大小的 header，ftruncate 多餘的
 *      預配置空間、fsync，設 WRITER_DONE_BIT。
 *  16 個 block (約 1 秒) 的池讓 SD 偶發延遲不會擋住 mic_capture。
 * ========================================================== */
static void mic_writer(void *arg)
{
    (void)arg;
    const int fd = s_session.storage.fd;

    /* 先放入佔位 header，錄完再回寫實際大小 */
    wav_header_t hdr = {0};
    fill_wav_header(&hdr, 0);
    memcpy(s_write_buffer, &hdr, sizeof(hdr));
    size_t fill = sizeof(hdr);
    uint32_t block_index = 0;   /* timing trace：第幾個 block */
    uint32_t write_index = 0;   /* timing trace：第幾次 SD 寫入 */

    for (;;) {
        audio_block_t *blk = NULL;
        xQueueReceive(s_full_blocks, &blk, portMAX_DELAY);
        if (blk == NULL) {
            break;
        }
        const int64_t take_us = esp_timer_get_time();
        timing_trace_stat(TT_STAT_MIC_QUEUE_WAIT, take_us - blk->queued_us, block_index);
        int64_t sd_us = 0;          /* 這個 block 內 SD 寫入的時間，從 gain 扣掉 */

        for (size_t i = 0; i < blk->sample_count; ++i) {
            const int32_t gained = (int32_t)blk->samples[i] * PCM_GAIN;
            const int16_t pcm = gained > INT16_MAX ? INT16_MAX :
                                gained < INT16_MIN ? INT16_MIN :
                                (int16_t)gained;
            memcpy(s_write_buffer + fill, &pcm, sizeof(pcm));
            fill += sizeof(pcm);
            if (fill == WRITE_BUFFER_BYTES) {
                const int64_t write_start_us = esp_timer_get_time();
                if (s_session.writer_result == ESP_OK &&
                    write_all(fd, s_write_buffer, fill) != ESP_OK) {
                    ESP_LOGE(TAG, "WAV write failed: errno=%d", errno);
                    s_session.writer_result = ESP_FAIL;
                }
                const int64_t write_us = esp_timer_get_time() - write_start_us;
                sd_us += write_us;
                timing_trace_stat(TT_STAT_MIC_SD_WRITE, write_us, write_index);
                write_index++;
                fill = 0;
            }
        }
        timing_trace_stat(TT_STAT_MIC_GAIN, esp_timer_get_time() - take_us - sd_us, block_index);
        block_index++;
        s_session.samples_written += (uint32_t)blk->sample_count;
        blk->sample_count = 0;
        xQueueSend(s_free_blocks, &blk, portMAX_DELAY);
    }

    if (fill > 0 && s_session.writer_result == ESP_OK &&
        write_all(fd, s_write_buffer, fill) != ESP_OK) {
        s_session.writer_result = ESP_FAIL;
    }

    /* 回寫 header 並截斷預先配置的多餘空間 */
    const uint32_t data_bytes = s_session.samples_written * (uint32_t)sizeof(int16_t);
    fill_wav_header(&hdr, data_bytes);
    if (lseek(fd, 0, SEEK_SET) != 0 ||
        write_all(fd, (const uint8_t *)&hdr, sizeof(hdr)) != ESP_OK ||
        ftruncate(fd, (off_t)(sizeof(hdr) + data_bytes)) != 0 ||
        fsync(fd) != 0) {
        ESP_LOGE(TAG, "WAV finalize failed: errno=%d", errno);
        if (s_session.writer_result == ESP_OK) {
            s_session.writer_result = ESP_FAIL;
        }
    }

    s_session.writer_end_us = esp_timer_get_time();
    timing_trace_mark_at(TT_MARK_MIC_WRITER_DONE, s_session.writer_end_us);
    xEventGroupSetBits(s_events, WRITER_DONE_BIT);
    vTaskDelete(NULL);
}

/* ==========================================================
 *  Public API
 * ========================================================== */
esp_err_t mic_record_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_free_blocks = xQueueCreateStatic(AUDIO_BLOCK_COUNT, sizeof(audio_block_t *),
                                       s_free_queue_storage, &s_free_queue_control);
    s_full_blocks = xQueueCreateStatic(AUDIO_BLOCK_COUNT + 1U, sizeof(audio_block_t *),
                                       s_full_queue_storage, &s_full_queue_control);
    s_events = xEventGroupCreateStatic(&s_events_control);
    if (!s_free_blocks || !s_full_blocks || !s_events) {
        return ESP_ERR_NO_MEM;
    }

    s_blocks = heap_caps_malloc(AUDIO_BLOCK_COUNT * sizeof(audio_block_t),
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_blocks == NULL) {
        ESP_LOGE(TAG, "Audio block pool (%u B) PSRAM allocation failed",
                 (unsigned)(AUDIO_BLOCK_COUNT * sizeof(audio_block_t)));
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(i2s_init(), TAG, "I2S initialization failed");
    s_initialized = true;
    ESP_LOGI(TAG, "Microphone ready: %s, %u Hz mono, gain x%d (clock stopped until recording)",
             MIC_INTERFACE_I2S ? "I2S" : "PDM", (unsigned)MIC_SAMPLE_RATE, PCM_GAIN);
    return ESP_OK;
}

esp_err_t mic_record_prepare(time_t recorded_at_epoch)
{
    if (!s_initialized || s_session.prepared) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(&s_session, 0, sizeof(s_session));
    s_session.storage.fd = -1;

    xQueueReset(s_free_blocks);
    xQueueReset(s_full_blocks);
    for (size_t i = 0; i < AUDIO_BLOCK_COUNT; ++i) {
        audio_block_t *blk = &s_blocks[i];
        blk->sample_count = 0;
        xQueueSend(s_free_blocks, &blk, 0);
    }
    xEventGroupClearBits(s_events, CAPTURE_DONE_BIT | WRITER_DONE_BIT);

    /* 觸發點前約 8 秒啟動時脈：麥克風喚醒需要時間 (INMP441 約數百 ms)，
     * 啟動到觸發點之間的資料由 mic_capture 讀出丟棄 */
    esp_err_t err = set_clock_enabled(true);
    if (err != ESP_OK) {
        return err;
    }

    err = node_storage_begin_recording(&s_session.storage, NODE_MEDIA_AUDIO,
                                       recorded_at_epoch,
                                       (size_t)MIC_FILE_BYTES);
    if (err != ESP_OK) {
        (void)set_clock_enabled(false);
        return err;
    }
    s_session.capture_result = ESP_OK;
    s_session.writer_result = ESP_OK;
    s_session.prepared = true;
    return ESP_OK;
}

esp_err_t mic_record_start(int64_t trigger_us)
{
    if (!s_session.prepared || s_session.started) {
        return ESP_ERR_INVALID_STATE;
    }
    s_session.trigger_us = trigger_us;

    if (xTaskCreatePinnedToCore(mic_writer, "mic_writer", WRITER_TASK_STACK_BYTES,
                                NULL, WRITER_TASK_PRIORITY, NULL,
                                WRITER_TASK_CORE) != pdPASS) {
        s_session.start_error = ESP_ERR_NO_MEM;
        return ESP_ERR_NO_MEM;
    }
    s_session.started = true;

    if (xTaskCreatePinnedToCore(mic_capture, "mic_capture", CAPTURE_TASK_STACK_BYTES,
                                NULL, CAPTURE_TASK_PRIORITY, NULL,
                                CAPTURE_TASK_CORE) != pdPASS) {
        s_session.capture_result = ESP_ERR_NO_MEM;
        audio_block_t *sentinel = NULL;
        xQueueSend(s_full_blocks, &sentinel, portMAX_DELAY);
        xEventGroupSetBits(s_events, CAPTURE_DONE_BIT);
    }
    ESP_LOGI(TAG, "Recording to %s: %u samples", s_session.storage.undone_path,
             (unsigned)MIC_TARGET_SAMPLES);
    return ESP_OK;
}

esp_err_t mic_record_finish(mic_record_result_t *result)
{
    if (!result) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(result, 0, sizeof(*result));
    if (!s_session.prepared) {
        result->error = ESP_ERR_INVALID_STATE;
        return result->error;
    }
    result->prepared = true;
    result->recorded_at_epoch = s_session.storage.recorded_at_epoch;

    esp_err_t err = ESP_OK;
    const uint32_t overflows = s_overflow_count;
    if (!s_session.started) {
        (void)ftruncate(s_session.storage.fd, 0);
        err = s_session.start_error != ESP_OK ? s_session.start_error
                                              : ESP_ERR_INVALID_STATE;
    } else {
        xEventGroupWaitBits(s_events, CAPTURE_DONE_BIT | WRITER_DONE_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Recording finished: samples=%u/%u overflow=%u "
                 "first_sample_offset=%" PRId64 " us",
                 (unsigned)s_session.samples_written, (unsigned)MIC_TARGET_SAMPLES,
                 (unsigned)overflows,
                 s_session.first_sample_us - s_session.trigger_us);

        /* 品質檢核：I/O 錯誤或樣本不足即標記為 broken。
         * DMA 溢位 (短暫遺失聲音) 只記錄，不判定為損壞。 */
        if (overflows != 0U) {
            ESP_LOGW(TAG, "I2S overflow during recording: %u (ignored)", (unsigned)overflows);
        }
        if (s_session.capture_result != ESP_OK) {
            err = s_session.capture_result;
        } else if (s_session.writer_result != ESP_OK) {
            err = s_session.writer_result;
        } else if (s_session.samples_written != MIC_TARGET_SAMPLES) {
            err = ESP_ERR_INVALID_STATE;
        }
    }

    bool recording_ok = err == ESP_OK;
    esp_err_t storage_err = node_storage_finish_recording(&s_session.storage,
                                                          recording_ok);
    if (storage_err != ESP_OK) {
        err = storage_err;
        recording_ok = false;
    }

    result->success = recording_ok;
    result->samples = s_session.samples_written;
    result->overflow_count = overflows;
    result->first_sample_offset_us =
        s_session.samples_written > 0U ? s_session.first_sample_us - s_session.trigger_us
                                       : 0;
    result->elapsed_ms = s_session.started
                             ? (s_session.writer_end_us - s_session.trigger_us) / 1000
                             : 0;
    result->error = err;
    const char *result_path = recording_ok ? s_session.storage.done_path
                                           : s_session.storage.broken_path;
    strlcpy(result->file_path, result_path, sizeof(result->file_path));
    (void)node_storage_get_file_size(result_path, &result->file_bytes);

    /* 錄音結束 (兩個任務都已退出)：停止時脈直到下一次錄音 */
    (void)set_clock_enabled(false);
    if (s_session.started) {
        /* overflow 拆成兩段：錄音中 (真的掉資料) / 收滿到時脈停止 (沒人讀，不影響檔案) */
        const uint32_t overflows_total = s_overflow_count;
        timing_trace_gauge_set(TT_GAUGE_MIC_OVF_RECORDING, s_session.overflows_recording);
        timing_trace_gauge_set(TT_GAUGE_MIC_OVF_TAIL,
                               overflows_total - s_session.overflows_recording);
    }
    s_session.prepared = false;
    s_session.started = false;
    return err;
}
