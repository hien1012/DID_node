#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "audio_record.h"
#include "app_config.h"
#include "storage.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

//#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#define I2S_MIC 1

#if I2S_MIC
#include "driver/i2s_std.h"
#else
#include "driver/i2s_pdm.h"
#endif

// ================= 硬體腳位定義 =================
#if I2S_MIC
#define I2S_BCLK_IO     GPIO_NUM_26   // SCK / BCLK
#define I2S_WS_IO       GPIO_NUM_25   // WS / LRCLK
#define I2S_DIN_IO      GPIO_NUM_33   /* SD / DOUT */
#else
#define PDM_CLK_IO GPIO_NUM_26
#define PDM_DIN_IO GPIO_NUM_25
#endif
// Debug GPIO（選用）
#define DBG_REC_PIN     GPIO_NUM_12   // capture task 進行中
#define DBG_SD_PIN      GPIO_NUM_18   // writer task 正在 fwrite

// ================= 錄音參數 =================
#define SAMPLE_RATE             16000
#define RECORD_TIME_SEC         APP_RECORD_DURATION_SEC
#define WAV_BITS_PER_SAMPLE     16
#define WAV_CHANNELS            1

// I2S 讀取參數
#define DMA_DESC_NUM            8
#define DMA_FRAME_NUM           512
#define I2S_READ_BYTES          8096

// 32-bit raw -> 16-bit PCM 縮位量
#define PCM_SHIFT_BITS          16

// [+] PCM 增益倍數，由 #define 統一管理（原本寫死在 writer_task 內）
#define PCM_GAIN                30

// ================= 雙任務 / Block Pool 參數 =================
#define AUDIO_BLOCK_SAMPLES     1024
#define AUDIO_BLOCK_COUNT       16
#define FILE_IO_BUFFER_SIZE     32768

// [+] 開錄前丟棄的 DMA 暖身讀次數（清掉 I2S pipeline 殘留資料）
#define I2S_FLUSH_READS         4

static const char *TAG = "RECORD";

// ================= 型別定義 =================
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

typedef struct {
    size_t  sample_count;
    int16_t samples[AUDIO_BLOCK_SAMPLES];
} audio_block_t;

// ================= 全域變數 =================
static i2s_chan_handle_t rx_handle  = NULL;

static QueueHandle_t q_free_blocks = NULL;
static QueueHandle_t q_full_blocks = NULL;

static TaskHandle_t g_capture_task = NULL;
static TaskHandle_t g_writer_task  = NULL;
static TaskHandle_t g_requesting_task = NULL;
static SemaphoreHandle_t g_writer_ready_sem = NULL;
static SemaphoreHandle_t g_record_mutex = NULL;

static volatile uint32_t overflow_count = 0;
static volatile bool     capture_done   = false;
static volatile bool     writer_done    = false;
static volatile bool     writer_ready   = false;
static volatile bool     write_error    = false;
static esp_err_t         g_record_error = ESP_OK;
static size_t            g_total_samples_written = 0;
static int64_t           g_record_started_us = 0;
static bool              g_initialized = false;

// 固定記憶體池
static audio_block_t g_audio_blocks[AUDIO_BLOCK_COUNT];
static int32_t       g_i2s_read_buffer[I2S_READ_BYTES / sizeof(int32_t)];
static char          g_file_io_buffer[FILE_IO_BUFFER_SIZE];

// [+] 動態 WAV 路徑緩衝（"/sdcard/N.wav"，最大支援到 9999.wav）
static storage_recording_t g_storage_recording;

// ==========================================================
//  Debug GPIO
// ==========================================================
static void dbg_pin_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << DBG_REC_PIN) | (1ULL << DBG_SD_PIN),
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(DBG_REC_PIN, 0);
    gpio_set_level(DBG_SD_PIN, 0);
}

// ==========================================================
//  WAV 工具
// ==========================================================
static void fill_wav_header(wav_header_t *hdr, uint32_t data_size)
{
    memcpy(hdr->riff, "RIFF", 4);
    memcpy(hdr->wave, "WAVE", 4);
    memcpy(hdr->fmt_, "fmt ", 4);
    memcpy(hdr->data, "data", 4);

    hdr->file_size_minus_8 = data_size + 36;
    hdr->fmt_chunk_size    = 16;
    hdr->audio_format      = 1;  // PCM
    hdr->num_channels      = WAV_CHANNELS;
    hdr->sample_rate       = SAMPLE_RATE;
    hdr->bits_per_sample   = WAV_BITS_PER_SAMPLE;
    hdr->byte_rate         = SAMPLE_RATE * WAV_CHANNELS * (WAV_BITS_PER_SAMPLE / 8);
    hdr->block_align       = WAV_CHANNELS * (WAV_BITS_PER_SAMPLE / 8);
    hdr->data_size         = data_size;
}

static FILE *wav_open_placeholder(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open file: %s", path);
        return NULL;
    }
    setvbuf(f, g_file_io_buffer, _IOFBF, sizeof(g_file_io_buffer));

    wav_header_t hdr = {0};
    fill_wav_header(&hdr, 0);  // 先寫佔位 header，錄完再 seek 回來填實際大小
    if (fwrite(&hdr, 1, sizeof(hdr), f) != sizeof(hdr)) {
        ESP_LOGE(TAG, "Cannot write WAV header: %s", path);
        fclose(f);
        return NULL;
    }
    return f;
}

static esp_err_t wav_finalize(FILE *f, size_t total_samples_written)
{
    size_t data_bytes = total_samples_written * sizeof(int16_t);
    wav_header_t hdr = {0};
    fill_wav_header(&hdr, data_bytes);

    esp_err_t err = ESP_OK;
    if (fseek(f, 0, SEEK_SET) != 0 ||
        fwrite(&hdr, 1, sizeof(hdr), f) != sizeof(hdr) ||
        fflush(f) != 0) {
        err = ESP_FAIL;
    }
    if (fclose(f) != 0) {
        err = ESP_FAIL;
    }
    return err;
}

// ==========================================================
//  I2S
// ==========================================================
static bool IRAM_ATTR i2s_rx_overflow_callback(i2s_chan_handle_t handle,
                                                i2s_event_data_t *event,
                                                void *user_ctx)
{
    (void)handle; (void)event; (void)user_ctx;
    overflow_count++;
    return false;
}

static esp_err_t i2s_init(void)
{
    i2s_chan_config_t chan_cfg = {
        .id           = I2S_NUM_0,
        .role         = I2S_ROLE_MASTER,
        .dma_desc_num = DMA_DESC_NUM,
        .dma_frame_num= DMA_FRAME_NUM,
        .auto_clear   = true,
    };
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &rx_handle),
                        TAG, "i2s_new_channel failed");
#if I2S_MIC

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk  = I2S_GPIO_UNUSED,
            .bclk  = I2S_BCLK_IO,
            .ws    = I2S_WS_IO,
            .dout  = I2S_GPIO_UNUSED,
            .din   = I2S_DIN_IO,
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
    std_cfg.slot_cfg.msb_right = false;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx_handle, &std_cfg),
                        TAG, "i2s_channel_init_std_mode failed");

#else
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = PDM_CLK_IO,
            .din = PDM_DIN_IO,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    ESP_RETURN_ON_ERROR(i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg),
                        TAG, "i2s_channel_init_std_mode failed");
#endif
    i2s_event_callbacks_t cbs = {
        .on_recv       = NULL,
        .on_recv_q_ovf = i2s_rx_overflow_callback,
        .on_sent       = NULL,
        .on_send_q_ovf = NULL,
    };
    ESP_RETURN_ON_ERROR(i2s_channel_register_event_callback(rx_handle, &cbs, NULL),
                        TAG, "register cb failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx_handle),
                        TAG, "i2s_channel_enable failed");
    return ESP_OK;
}

static void i2s_deinit(void)
{
    if (rx_handle) {
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
        rx_handle = NULL;
    }
}

// ==========================================================
//  Audio Pipeline（Queue）
// ==========================================================
static esp_err_t audio_pipeline_init(void)
{
    q_free_blocks = xQueueCreate(AUDIO_BLOCK_COUNT,     sizeof(audio_block_t *));
    q_full_blocks = xQueueCreate(AUDIO_BLOCK_COUNT + 1, sizeof(audio_block_t *));

    if (!q_free_blocks || !q_full_blocks) {
        ESP_LOGE(TAG, "Queue create failed");
        return ESP_FAIL;
    }

    for (int i = 0; i < AUDIO_BLOCK_COUNT; i++) {
        audio_block_t *blk = &g_audio_blocks[i];
        blk->sample_count = 0;
        xQueueSend(q_free_blocks, &blk, portMAX_DELAY);
    }
    return ESP_OK;
}

static void audio_pipeline_deinit(void)
{
    if (q_free_blocks) { vQueueDelete(q_free_blocks); q_free_blocks = NULL; }
    if (q_full_blocks) { vQueueDelete(q_full_blocks); q_full_blocks = NULL; }
}

// ==========================================================
//  Capture Task
// ==========================================================
static void capture_task(void *arg)
{
    (void)arg;

    size_t target_samples        = (size_t)SAMPLE_RATE * RECORD_TIME_SEC;
    size_t total_samples_captured = 0;
    size_t bytes_read            = 0;

    // [+] 暖身：丟棄 DMA pipeline 中的殘留資料，避免開頭有雜音
    for (int flush = 0; flush < I2S_FLUSH_READS; flush++) {
        i2s_channel_read(rx_handle, g_i2s_read_buffer, I2S_READ_BYTES, &bytes_read, 100);
    }

    audio_block_t *blk = NULL;
    xQueueReceive(q_free_blocks, &blk, portMAX_DELAY);
    blk->sample_count = 0;

    capture_done = false;
    gpio_set_level(DBG_REC_PIN, 1);
    int64_t t0 = esp_timer_get_time();
    ESP_LOGI(TAG, "capture_task start, target=%u samples", (unsigned)target_samples);

    while (total_samples_captured < target_samples) {
        esp_err_t ret = i2s_channel_read(rx_handle,
                                         g_i2s_read_buffer,
                                         I2S_READ_BYTES,
                                         &bytes_read,
                                         100);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "i2s_channel_read failed: %d", ret);
            g_record_error = ret;
            break;
        }

        int n = bytes_read / sizeof(int32_t);
        for (int i = 0; i < n && total_samples_captured < target_samples; i++) {
            int16_t pcm = (int16_t)(g_i2s_read_buffer[i] >> PCM_SHIFT_BITS);
            blk->samples[blk->sample_count++] = pcm;
            total_samples_captured++;

            if (blk->sample_count >= AUDIO_BLOCK_SAMPLES) {
                xQueueSend(q_full_blocks, &blk, portMAX_DELAY);
                xQueueReceive(q_free_blocks, &blk, portMAX_DELAY);
                blk->sample_count = 0;
            }
        }
    }

    if (blk && blk->sample_count > 0) {
        xQueueSend(q_full_blocks, &blk, portMAX_DELAY);
    }
    capture_done = true;
    blk = NULL;  // sentinel：通知 writer_task 結束
    xQueueSend(q_full_blocks, &blk, portMAX_DELAY);

    gpio_set_level(DBG_REC_PIN, 0);

    double elapsed_s = (esp_timer_get_time() - t0) / 1000000.0;
    ESP_LOGI(TAG, "capture_task done: %u samples, %.3f s",
             (unsigned)total_samples_captured, elapsed_s);
    g_capture_task = NULL;
    vTaskDelete(NULL);
}

// ==========================================================
//  Writer Task
// ==========================================================
static void writer_task(void *arg)
{
    const char *path = (const char *)arg;

    FILE *f = wav_open_placeholder(path);
    if (!f) {
        writer_ready = false;
        g_record_error = ESP_FAIL;
        xSemaphoreGive(g_writer_ready_sem);
        if (g_requesting_task) {
            xTaskNotifyGive(g_requesting_task);
        }
        g_writer_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    writer_done            = false;
    g_total_samples_written = 0;
    writer_ready = true;
    xSemaphoreGive(g_writer_ready_sem);
    ESP_LOGI(TAG, "writer_task start: %s", path);

    while (1) {
        audio_block_t *blk = NULL;
        xQueueReceive(q_full_blocks, &blk, portMAX_DELAY);
        if (blk == NULL) break;  // sentinel，結束迴圈

        // [+] 增益（使用 #define PCM_GAIN，統一管理）
        for (int i = 0; i < (int)blk->sample_count; i++) {
            int32_t s = (int32_t)(blk->samples[i] * PCM_GAIN);
            blk->samples[i] = (s > INT16_MAX) ? INT16_MAX :
                               (s < INT16_MIN) ? INT16_MIN :
                               (int16_t)s;
        }

        gpio_set_level(DBG_SD_PIN, 1);
        size_t written = fwrite(blk->samples, sizeof(int16_t), blk->sample_count, f);
        gpio_set_level(DBG_SD_PIN, 0);

        if (written != blk->sample_count) {
            ESP_LOGE(TAG, "fwrite short: expect=%u actual=%u",
                     (unsigned)blk->sample_count, (unsigned)written);
            write_error = true;
            g_record_error = ESP_FAIL;
        }

        g_total_samples_written += written;
        blk->sample_count = 0;
        xQueueSend(q_free_blocks, &blk, portMAX_DELAY);
    }

    esp_err_t finalize_err = wav_finalize(f, g_total_samples_written);
    if (finalize_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to finalize WAV file");
        write_error = true;
        g_record_error = finalize_err;
    }
    writer_done = true;

    ESP_LOGI(TAG, "writer_task done: %u samples, %.3f s",
             (unsigned)g_total_samples_written,
             (double)g_total_samples_written / SAMPLE_RATE);

    if (g_requesting_task) {
        xTaskNotifyGive(g_requesting_task);
    }
    g_writer_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t audio_record_init(void)
{
    if (g_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Free internal heap: %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
#ifdef CONFIG_SPIRAM
    ESP_LOGI(TAG, "Free PSRAM: %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
#endif

    dbg_pin_init();

    g_writer_ready_sem = xSemaphoreCreateBinary();
    g_record_mutex = xSemaphoreCreateMutex();
    if (!g_writer_ready_sem || !g_record_mutex) {
        ESP_LOGE(TAG, "Recorder semaphore creation failed");
        if (g_writer_ready_sem) {
            vSemaphoreDelete(g_writer_ready_sem);
            g_writer_ready_sem = NULL;
        }
        if (g_record_mutex) {
            vSemaphoreDelete(g_record_mutex);
            g_record_mutex = NULL;
        }
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = storage_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Storage initialization failed");
        vSemaphoreDelete(g_writer_ready_sem);
        vSemaphoreDelete(g_record_mutex);
        g_writer_ready_sem = NULL;
        g_record_mutex = NULL;
        return err;
    }
    ESP_LOGI(TAG, "Storage initialized");

    err = i2s_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S initialization failed");
        storage_deinit();
        vSemaphoreDelete(g_writer_ready_sem);
        vSemaphoreDelete(g_record_mutex);
        g_writer_ready_sem = NULL;
        g_record_mutex = NULL;
        return err;
    }
    ESP_LOGI(TAG, "I2S initialized");

    g_initialized = true;
    return ESP_OK;
}

esp_err_t audio_record_once(audio_record_result_t *result)
{
    if (!result) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(result, 0, sizeof(*result));

    if (!g_initialized) {
        result->error = ESP_ERR_INVALID_STATE;
        return result->error;
    }

    if (xSemaphoreTake(g_record_mutex, 0) != pdTRUE) {
        result->error = ESP_ERR_INVALID_STATE;
        return result->error;
    }

    g_requesting_task = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, 0);
    xSemaphoreTake(g_writer_ready_sem, 0);

    overflow_count = 0;
    capture_done = false;
    writer_done = false;
    writer_ready = false;
    write_error = false;
    g_record_error = ESP_OK;
    g_total_samples_written = 0;
    g_record_started_us = esp_timer_get_time();

    bool storage_active = false;
    time_t recorded_at_epoch = time(NULL);
    esp_err_t err = storage_begin_recording(&g_storage_recording,
                                            recorded_at_epoch);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot reserve recording path: %s", esp_err_to_name(err));
        g_record_error = err;
        goto finish;
    }
    storage_active = true;
    ESP_LOGI(TAG, ">>> Recording -> %s <<<", g_storage_recording.undone_path);

    err = audio_pipeline_init();
    if (err != ESP_OK) {
        g_record_error = err;
        goto finish;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        writer_task, "wtr_task", 6144,
        (void *)g_storage_recording.undone_path, 6, &g_writer_task, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create writer_task");
        g_record_error = ESP_ERR_NO_MEM;
        goto cleanup_pipeline;
    }

    xSemaphoreTake(g_writer_ready_sem, portMAX_DELAY);
    if (!writer_ready) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        goto cleanup_pipeline;
    }

    ok = xTaskCreatePinnedToCore(
        capture_task, "cap_task", 4096, NULL, 10, &g_capture_task, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create capture_task");
        g_record_error = ESP_ERR_NO_MEM;
        audio_block_t *sentinel = NULL;
        xQueueSend(q_full_blocks, &sentinel, portMAX_DELAY);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        goto cleanup_pipeline;
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

cleanup_pipeline:
    audio_pipeline_deinit();

finish:
    bool recording_ok = (g_record_error == ESP_OK && capture_done &&
                         writer_done && !write_error);
    if (storage_active) {
        esp_err_t storage_err = storage_finish_recording(
            &g_storage_recording, recording_ok);
        if (storage_err != ESP_OK) {
            g_record_error = storage_err;
            recording_ok = false;
        }
    }

    result->capture_done = capture_done;
    result->writer_done = writer_done;
    result->write_error = write_error;
    result->error = g_record_error;
    result->total_samples = g_total_samples_written;
    result->overflow_count = overflow_count;
    result->elapsed_ms = (uint32_t)((esp_timer_get_time() - g_record_started_us) / 1000);
    result->recorded_at_epoch = recorded_at_epoch;
    const char *result_path = recording_ok ? g_storage_recording.done_path
                                           : g_storage_recording.undone_path;
    snprintf(result->file_path, sizeof(result->file_path), "%s", result_path);
    if (result_path[0] != '\0') {
        storage_get_file_size(result_path, &result->file_bytes);
    }
    result->success = recording_ok;

    g_requesting_task = NULL;
    xSemaphoreGive(g_record_mutex);
    return result->success ? ESP_OK : result->error;
}

void audio_record_deinit(void)
{
    if (!g_initialized) {
        return;
    }

    if (xSemaphoreTake(g_record_mutex, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Recorder is busy; deinit skipped");
        return;
    }

    i2s_deinit();
    storage_deinit();
    g_initialized = false;

    xSemaphoreGive(g_record_mutex);
    vSemaphoreDelete(g_record_mutex);
    vSemaphoreDelete(g_writer_ready_sem);
    g_record_mutex = NULL;
    g_writer_ready_sem = NULL;
}
