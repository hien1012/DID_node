#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"          // [+] 新增：binary semaphore

#include "driver/i2s_pdm.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "esp_heap_caps.h"

#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

// ================= 硬體腳位定義 =================
#define PDM_CLK_IO      GPIO_NUM_26
#define PDM_DIN_IO      GPIO_NUM_25

// [+] 錄音觸發按鈕 —— 按下 = LOW（GPIO 啟用內部上拉）
#define BTN_PIN         GPIO_NUM_0    // 依實際接線修改

// Debug GPIO（選用）
#define DBG_REC_PIN     GPIO_NUM_12   // capture task 進行中
#define DBG_SD_PIN      GPIO_NUM_18   // writer task 正在 fwrite

// ================= 錄音參數 =================
#define SAMPLE_RATE             16000
#define RECORD_TIME_SEC         20
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

// ================= [+] 按鈕 Debounce 參數 =================
#define BTN_DEBOUNCE_MS         50    // 消抖視窗（確認確實按下）
#define BTN_POST_RELEASE_MS     300   // 放開後額外延遲，讓機械聲消散才開錄

// ================= 雙任務 / Block Pool 參數 =================
#define AUDIO_BLOCK_SAMPLES     1024
#define AUDIO_BLOCK_COUNT       16
#define FILE_IO_BUFFER_SIZE     32768

// [+] 開錄前丟棄的 DMA 暖身讀次數（清掉 I2S pipeline 殘留資料）
#define I2S_FLUSH_READS         4

static const char *TAG = "I2S_REC";

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
static sdmmc_card_t     *sd_card   = NULL;

static QueueHandle_t q_free_blocks = NULL;
static QueueHandle_t q_full_blocks = NULL;

static TaskHandle_t g_capture_task = NULL;
static TaskHandle_t g_writer_task  = NULL;
static TaskHandle_t g_main_task    = NULL;

// [+] 按鈕信號量
//   g_btn_pressed_sem  : ISR -> button_task（偵測到下降沿）
//   g_record_start_sem : button_task -> app_main（確認放開 + 延遲完成）
static SemaphoreHandle_t g_btn_pressed_sem  = NULL;
static SemaphoreHandle_t g_record_start_sem = NULL;

static volatile uint32_t overflow_count = 0;
static volatile bool     capture_done   = false;
static volatile bool     writer_done    = false;
static size_t            g_total_samples_written = 0;

// 固定記憶體池
static audio_block_t g_audio_blocks[AUDIO_BLOCK_COUNT];
static int32_t       g_pdm_read_buffer[I2S_READ_BYTES / sizeof(int32_t)];
static char          g_file_io_buffer[FILE_IO_BUFFER_SIZE];

// [+] 動態 WAV 路徑緩衝（"/sdcard/N.wav"，最大支援到 9999.wav）
static char g_wav_path[32];

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
//  [+] 按鈕初始化 & ISR
// ==========================================================

/**
 * @brief GPIO 中斷服務：偵測到下降沿（按下），給信號量通知 button_task
 */
static void IRAM_ATTR btn_isr_handler(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(g_btn_pressed_sem, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
}

static void btn_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << BTN_PIN),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,   // 按下 = LOW
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_NEGEDGE,    // 下降沿觸發
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_PIN, btn_isr_handler, NULL);
}

/**
 * @brief 按鈕去抖任務
 *
 * 流程：
 *   1. 阻塞等待 ISR 給出的 g_btn_pressed_sem（下降沿）
 *   2. 等 BTN_DEBOUNCE_MS，確認電位確實為 LOW（非雜訊）
 *   3. 輪詢等到按鈕放開（電位回 HIGH）
 *   4. 再等 BTN_POST_RELEASE_MS，讓機械彈跳聲消散
 *   5. 清除等待期間可能累積的額外 ISR 信號
 *   6. 給 g_record_start_sem，通知 app_main 可以開始錄音
 */
static void button_task(void *arg)
{
    (void)arg;

    while (1) {
        // --- 步驟 1：等下降沿 ---
        xSemaphoreTake(g_btn_pressed_sem, portMAX_DELAY);

        // --- 步驟 2：消抖確認 ---
        vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));
        if (gpio_get_level(BTN_PIN) != 0) {
            ESP_LOGD(TAG, "btn: noise, ignored");
            continue;  // 雜訊，重新等待
        }

        // --- 步驟 3：等放開 ---
        while (gpio_get_level(BTN_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // --- 步驟 4：等按鍵聲消散 ---
        vTaskDelay(pdMS_TO_TICKS(BTN_POST_RELEASE_MS));

        // --- 步驟 5：清掉等待期間積累的多餘 ISR 信號 ---
        xSemaphoreTake(g_btn_pressed_sem, 0);

        // --- 步驟 6：觸發錄音 ---
        ESP_LOGI(TAG, "Button confirmed released -> start recording");
        xSemaphoreGive(g_record_start_sem);
    }
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
    fwrite(&hdr, 1, sizeof(hdr), f);
    return f;
}

static void wav_finalize(FILE *f, size_t total_samples_written)
{
    size_t data_bytes = total_samples_written * sizeof(int16_t);
    wav_header_t hdr = {0};
    fill_wav_header(&hdr, data_bytes);
    fseek(f, 0, SEEK_SET);
    fwrite(&hdr, 1, sizeof(hdr), f);
    fclose(f);
}

// ==========================================================
//  [+] 取得下一個可用的 WAV 檔案編號
//      掃描 /sdcard/1.wav, 2.wav … 找到第一個不存在的編號
// ==========================================================
static int get_next_file_index(void)
{
    int   idx = 1;
    char  path[32];

    while (1) {
        snprintf(path, sizeof(path), "/sdcard/%d.wav", idx);
        FILE *f = fopen(path, "rb");
        if (!f) {
            break;   // 此編號尚未使用
        }
        fclose(f);
        idx++;
    }
    return idx;
}

// ==========================================================
//  SD Card
// ==========================================================
static esp_err_t sd_card_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files              = 5,
        .allocation_unit_size   = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags        = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = 20000;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width  = 1;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    return esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &sd_card);
}

static void sd_card_deinit(void)
{
    if (sd_card) {
        esp_vfs_fat_sdcard_unmount("/sdcard", sd_card);
        sd_card = NULL;
    }
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
        i2s_channel_read(rx_handle, g_pdm_read_buffer, I2S_READ_BYTES, &bytes_read, 100);
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
                                         g_pdm_read_buffer,
                                         I2S_READ_BYTES,
                                         &bytes_read,
                                         100);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "i2s_channel_read failed: %d", ret);
            break;
        }

        int n = bytes_read / sizeof(int32_t);
        for (int i = 0; i < n && total_samples_captured < target_samples; i++) {
            int16_t pcm = (int16_t)(g_pdm_read_buffer[i] >> PCM_SHIFT_BITS);
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
    blk = NULL;  // sentinel：通知 writer_task 結束
    xQueueSend(q_full_blocks, &blk, portMAX_DELAY);

    gpio_set_level(DBG_REC_PIN, 0);
    capture_done = true;

    double elapsed_s = (esp_timer_get_time() - t0) / 1000000.0;
    ESP_LOGI(TAG, "capture_task done: %u samples, %.3f s",
             (unsigned)total_samples_captured, elapsed_s);
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
        xTaskNotifyGive(g_main_task);
        vTaskDelete(NULL);
        return;
    }

    writer_done            = false;
    g_total_samples_written = 0;
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
        }

        g_total_samples_written += written;
        blk->sample_count = 0;
        xQueueSend(q_free_blocks, &blk, portMAX_DELAY);
    }

    wav_finalize(f, g_total_samples_written);
    writer_done = true;

    ESP_LOGI(TAG, "writer_task done: %u samples, %.3f s",
             (unsigned)g_total_samples_written,
             (double)g_total_samples_written / SAMPLE_RATE);

    xTaskNotifyGive(g_main_task);  // 通知 app_main 本次錄音完成
    vTaskDelete(NULL);
}

// ==========================================================
//  app_main —— 初始化後進入無限等待迴圈
// ==========================================================
void app_main(void)
{
    g_main_task = xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG, "Free internal heap: %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
#ifdef CONFIG_SPIRAM
    ESP_LOGI(TAG, "Free PSRAM: %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
#endif

    dbg_pin_init();

    // [+] 建立按鈕信號量
    g_btn_pressed_sem  = xSemaphoreCreateBinary();
    g_record_start_sem = xSemaphoreCreateBinary();
    if (!g_btn_pressed_sem || !g_record_start_sem) {
        ESP_LOGE(TAG, "Semaphore create failed");
        return;
    }

    // [+] 初始化按鈕 GPIO & 啟動去抖任務
    btn_init();
    xTaskCreatePinnedToCore(button_task, "btn_task", 2048, NULL, 5, NULL, 0);

    // 初始化 SD 卡 & I2S（只做一次，錄音之間不需重新初始化）
    if (sd_card_init() != ESP_OK) {
        ESP_LOGE(TAG, "SD Card initialization failed");
        return;
    }
    ESP_LOGI(TAG, "SD Card initialized");

    if (i2s_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2S initialization failed");
        sd_card_deinit();
        return;
    }
    ESP_LOGI(TAG, "I2S initialized");

    ESP_LOGI(TAG, "=== Ready. Press button to start recording ===");

    // =========================================================
    //  [+] 主迴圈：等按鈕 -> 錄音 -> 等下次按鈕 -> 無限重複
    // =========================================================
    while (1) {

        // --- 等待 button_task 確認放開後的信號 ---
        xSemaphoreTake(g_record_start_sem, portMAX_DELAY);

        // --- 決定本次輸出路徑 ---
        int file_idx = get_next_file_index();
        snprintf(g_wav_path, sizeof(g_wav_path), "/sdcard/%d.wav", file_idx);
        ESP_LOGI(TAG, ">>> Recording #%d -> %s <<<", file_idx, g_wav_path);

        overflow_count = 0;

        // --- 初始化 Queue（每次錄音前重新建立，確保乾淨狀態）---
        if (audio_pipeline_init() != ESP_OK) {
            ESP_LOGE(TAG, "audio_pipeline_init failed, skipping this recording");
            continue;
        }

        // --- 建立 capture task（Core 1，高優先權，專責 I2S 讀取）---
        BaseType_t ok = xTaskCreatePinnedToCore(
            capture_task, "cap_task", 4096, NULL, 10, &g_capture_task, 1);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "Failed to create capture_task");
            audio_pipeline_deinit();
            continue;
        }

        // --- 建立 writer task（Core 0，傳入動態路徑）---
        ok = xTaskCreatePinnedToCore(
            writer_task, "wtr_task", 6144, (void *)g_wav_path, 6, &g_writer_task, 0);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "Failed to create writer_task");
            // capture_task 已啟動，等它結束後再清理
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            audio_pipeline_deinit();
            continue;
        }

        // --- 阻塞等待 writer_task 完成並呼叫 xTaskNotifyGive ---
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGI(TAG, "Finished: %s | capture=%d writer=%d overflow=%u samples=%u",
                 g_wav_path,
                 (int)capture_done, (int)writer_done,
                 (unsigned)overflow_count,
                 (unsigned)g_total_samples_written);

        // --- 清理 Queue，準備下一次錄音 ---
        audio_pipeline_deinit();

        ESP_LOGI(TAG, "=== Press button to record again ===");
    }

    // 正常情況不會到達這裡；若跳出迴圈才做完整清理
    i2s_deinit();
    sd_card_deinit();
}