/*
 * SD 重播除錯 (說明見 debug_replay.h)：SD 上的固定輸入 → 正式 interpreter 推論 → 既有的逐層 dump。
 * 輸出格式與即時 dump 完全相同，只是 meta.json 的 source = "sd_replay"、input_file = 檔名。
 */
#include "debug_replay.h"

#include <dirent.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "infer_config.h"

#if INFER_CAM_ENABLE
#include "cam_debug.h"
#include "cam_model.h"
#define REPLAY_PREFIX        "cam_"
#define REPLAY_EXT           ".gray"
#else
#include "audio_debug.h"
#include "audio_model.h"
#include "logmel_features_int8.h"
#define REPLAY_PREFIX        "mic_"
#define REPLAY_EXT           ".wav"
#define REPLAY_PCM_BYTES     (AUDIO_LOGMEL_INPUT_SAMPLES * sizeof(int16_t))
#endif

/* main/CMakeLists.txt 由 ESP_NN_KERNEL_SET cache 變數帶進來 */
#ifndef INFER_ESP_NN_KERNEL_SET
#define INFER_ESP_NN_KERNEL_SET "unknown"
#endif

#define REPLAY_NAME_BYTES        72U      /* > CONFIG_FATFS_MAX_LFN (64) */
#define REPLAY_PATH_BYTES        (sizeof(DEBUG_REPLAY_DIR) + REPLAY_NAME_BYTES + 1U)
#define REPLAY_TASK_STACK_BYTES  12288U   /* TFLM Invoke + dump；與 cam_infer / audio_cnn 相當再加餘裕 */
#define REPLAY_TASK_PRIORITY     5U

static const char *TAG = "DEBUG_REPLAY";

typedef struct {
    TaskHandle_t caller;
    esp_err_t result;
} replay_context_t;

/* 只在 debug_replay task 內使用 */
static char s_files[DEBUG_REPLAY_MAX_FILES][REPLAY_NAME_BYTES];
static char s_stem[REPLAY_NAME_BYTES];
static char s_path[REPLAY_PATH_BYTES];

static bool has_suffix(const char *name, const char *suffix)
{
    const size_t name_len = strlen(name);
    const size_t suffix_len = strlen(suffix);
    return name_len > suffix_len && strcasecmp(name + name_len - suffix_len, suffix) == 0;
}

static int compare_names(const void *a, const void *b)
{
    return strcasecmp((const char *)a, (const char *)b);
}

/* 找出這個角色的輸入檔並依檔名排序；回傳個數 */
static size_t list_inputs(void)
{
    DIR *dir = opendir(DEBUG_REPLAY_DIR);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Cannot open %s (create the folder on the SD card)", DEBUG_REPLAY_DIR);
        return 0;
    }
    size_t count = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        const char *name = entry->d_name;
        if (entry->d_type == DT_DIR || strncasecmp(name, REPLAY_PREFIX, strlen(REPLAY_PREFIX)) != 0 ||
            !has_suffix(name, REPLAY_EXT)) {
            continue;
        }
        const size_t length = strlen(name);
        if (length >= REPLAY_NAME_BYTES) {
            ESP_LOGW(TAG, "Skip %s: file name too long", name);
            continue;
        }
        if (count == DEBUG_REPLAY_MAX_FILES) {
            ESP_LOGW(TAG, "More than %u inputs; the rest are skipped", (unsigned)DEBUG_REPLAY_MAX_FILES);
            break;
        }
        memcpy(s_files[count], name, length + 1U);
        ++count;
    }
    closedir(dir);
    qsort(s_files, count, REPLAY_NAME_BYTES, compare_names);
    return count;
}

/* dump 名稱 = 檔名去掉副檔名；PC 端用空白切欄位，其他字元一律換成 '_' */
static void make_stem(const char *file_name)
{
    const size_t length = strlen(file_name) - strlen(REPLAY_EXT);
    for (size_t i = 0; i < length; ++i) {
        const char c = file_name[i];
        const bool keep = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                          c == '-' || c == '_';
        s_stem[i] = keep ? c : '_';
    }
    s_stem[length] = '\0';
}

#if INFER_CAM_ENABLE

typedef struct {
    uint8_t *roi;
} replay_buffers_t;

static bool allocate_buffers(replay_buffers_t *b)
{
    b->roi = heap_caps_malloc(CAM_MODEL_ROI_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return b->roi != NULL;
}

static void free_buffers(replay_buffers_t *b)
{
    heap_caps_free(b->roi);
    b->roi = NULL;
}

/* .gray：剛好 64 x 64 bytes，不接受其他大小 (避免把別的解析度誤當成 ROI) */
static esp_err_t read_roi(const char *path, uint8_t *roi)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Cannot open %s", path);
        return ESP_ERR_NOT_FOUND;
    }
    esp_err_t err = ESP_OK;
    long size = -1;
    if (fseek(file, 0, SEEK_END) == 0) {
        size = ftell(file);
    }
    if (size != (long)CAM_MODEL_ROI_BYTES) {
        ESP_LOGE(TAG, "%s: %ld bytes, expected %u (64x64 uint8)", path, size, (unsigned)CAM_MODEL_ROI_BYTES);
        err = ESP_ERR_INVALID_SIZE;
    } else if (fseek(file, 0, SEEK_SET) != 0 || fread(roi, 1, CAM_MODEL_ROI_BYTES, file) != CAM_MODEL_ROI_BYTES) {
        ESP_LOGE(TAG, "%s: read failed", path);
        err = ESP_FAIL;
    }
    fclose(file);
    return err;
}

static esp_err_t replay_one(uint32_t index, replay_buffers_t *b)
{
    esp_err_t err = read_roi(s_path, b->roi);
    if (err != ESP_OK) {
        return err;
    }
    float probs[CAM_MODEL_CLASS_COUNT] = {0};
    int pred = -1;
    err = cam_model_predict(b->roi, probs, &pred);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: inference failed: %s", s_path, esp_err_to_name(err));
        return err;
    }
    static const int32_t no_bbox[4] = {0, 0, 0, 0};
    const cam_debug_roi_t roi = {
        .roi = b->roi,
        .probs = probs,
        .pred = pred,
        .index = index,
        .frame = 0,
        .track_id = -1,
        .bbox = no_bbox,
        .ts_epoch_ms = 0,
        .invoke_us = cam_model_last_invoke_us(),
        .replay_name = s_stem,
    };
    return cam_debug_dump_roi(&roi);
}

#else /* 麥克風 */

typedef struct {
    int16_t *pcm;
    int8_t *features;
    void *workspace;
    size_t workspace_bytes;
} replay_buffers_t;

static void *psram_alloc(size_t bytes)
{
    return heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static bool allocate_buffers(replay_buffers_t *b)
{
    b->workspace_bytes = audio_logmel_int8_workspace_size();
    b->workspace = psram_alloc(b->workspace_bytes);
    b->pcm = psram_alloc(REPLAY_PCM_BYTES);
    b->features = psram_alloc(AUDIO_LOGMEL_MODEL_INPUT_VALUES);
    return b->workspace != NULL && b->pcm != NULL && b->features != NULL;
}

static void free_buffers(replay_buffers_t *b)
{
    heap_caps_free(b->workspace);
    heap_caps_free(b->pcm);
    heap_caps_free(b->features);
    b->workspace = NULL;
    b->pcm = NULL;
    b->features = NULL;
}

static uint32_t get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t get_le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

/* WAV：只接受 PCM16 / mono / 16 kHz (與即時路徑相同，不在節點上重取樣)；讀前 2 s */
static esp_err_t read_wav(const char *path, int16_t *pcm)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Cannot open %s", path);
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t header[12];
    if (fread(header, 1, sizeof(header), file) != sizeof(header) || memcmp(header, "RIFF", 4) != 0 ||
        memcmp(header + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "%s: not a RIFF/WAVE file", path);
        fclose(file);
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ESP_ERR_NOT_FOUND;
    bool format_ok = false;
    uint8_t chunk[8];
    while (err == ESP_ERR_NOT_FOUND && fread(chunk, 1, sizeof(chunk), file) == sizeof(chunk)) {
        const uint32_t size = get_le32(chunk + 4);
        const long padded = (long)size + (long)(size & 1U);
        if (memcmp(chunk, "fmt ", 4) == 0) {
            uint8_t fmt[16];
            if (size < sizeof(fmt) || fread(fmt, 1, sizeof(fmt), file) != sizeof(fmt)) {
                ESP_LOGE(TAG, "%s: bad fmt chunk", path);
                err = ESP_ERR_INVALID_ARG;
                break;
            }
            const unsigned format = get_le16(fmt);
            const unsigned channels = get_le16(fmt + 2);
            const unsigned rate = (unsigned)get_le32(fmt + 4);
            const unsigned bits = get_le16(fmt + 14);
            if (format != 1U || channels != 1U || rate != AUDIO_LOGMEL_SAMPLE_RATE_HZ || bits != 16U) {
                ESP_LOGE(TAG, "%s: need PCM16 mono %u Hz, got format=%u channels=%u rate=%u bits=%u",
                         path, (unsigned)AUDIO_LOGMEL_SAMPLE_RATE_HZ, format, channels, rate, bits);
                err = ESP_ERR_NOT_SUPPORTED;
                break;
            }
            format_ok = true;
            if (fseek(file, padded - (long)sizeof(fmt), SEEK_CUR) != 0) {
                err = ESP_FAIL;
            }
        } else if (memcmp(chunk, "data", 4) == 0) {
            if (!format_ok) {
                ESP_LOGE(TAG, "%s: data chunk before fmt chunk", path);
                err = ESP_ERR_INVALID_ARG;
            } else if (size < REPLAY_PCM_BYTES) {
                ESP_LOGE(TAG, "%s: only %u samples, need %u (2 s)", path, (unsigned)(size / sizeof(int16_t)),
                         (unsigned)AUDIO_LOGMEL_INPUT_SAMPLES);
                err = ESP_ERR_INVALID_SIZE;
            } else {
                err = fread(pcm, 1, REPLAY_PCM_BYTES, file) == REPLAY_PCM_BYTES ? ESP_OK : ESP_FAIL;
            }
        } else if (fseek(file, padded, SEEK_CUR) != 0) {
            err = ESP_FAIL;
        }
    }
    fclose(file);
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "%s: no data chunk", path);
    } else if (err == ESP_FAIL) {
        ESP_LOGE(TAG, "%s: read failed", path);
    }
    return err;
}

static esp_err_t replay_one(uint32_t index, replay_buffers_t *b)
{
    esp_err_t err = read_wav(s_path, b->pcm);
    if (err != ESP_OK) {
        return err;
    }
    const int64_t feature_started_us = esp_timer_get_time();
    const int fe_err = audio_logmel_extract_quantized_pcm16(b->pcm, AUDIO_LOGMEL_INPUT_SAMPLES, b->features,
                                                            b->workspace, b->workspace_bytes, NULL);
    const uint32_t feature_us = (uint32_t)(esp_timer_get_time() - feature_started_us);
    if (fe_err != AUDIO_LOGMEL_OK) {
        ESP_LOGE(TAG, "%s: logmel failed: %d", s_path, fe_err);
        return ESP_FAIL;
    }
    float probs[AUDIO_MODEL_CLASS_COUNT] = {0};
    int pred = -1;
    err = audio_model_predict(b->features, probs, &pred);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: inference failed: %s", s_path, esp_err_to_name(err));
        return err;
    }
    const audio_debug_window_t window = {
        .pcm = b->pcm,
        .features = b->features,
        .probs = probs,
        .pred = pred,
        .window = index,
        .start_epoch_ms = 0,
        .feature_us = feature_us,
        .invoke_us = audio_model_last_invoke_us(),
        .replay_name = s_stem,
    };
    return audio_debug_dump_window(&window);
}

#endif /* INFER_CAM_ENABLE */

static esp_err_t replay_all(void)
{
    const size_t count = list_inputs();
    if (count == 0) {
        ESP_LOGE(TAG, "No %s*%s files in %s", REPLAY_PREFIX, REPLAY_EXT, DEBUG_REPLAY_DIR);
        return ESP_ERR_NOT_FOUND;
    }
    replay_buffers_t buffers = {0};
    if (!allocate_buffers(&buffers)) {
        ESP_LOGE(TAG, "PSRAM allocation failed");
        free_buffers(&buffers);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW(TAG, "Replaying %u input(s) from %s, ESP-NN=%s (PC: receive_infer_node_dump.py --count %u)",
             (unsigned)count, DEBUG_REPLAY_DIR, INFER_ESP_NN_KERNEL_SET, (unsigned)count);
    size_t dumped = 0;
    for (size_t i = 0; i < count; ++i) {
        make_stem(s_files[i]);
        snprintf(s_path, sizeof(s_path), "%s/%s", DEBUG_REPLAY_DIR, s_files[i]);
        ESP_LOGW(TAG, "[%u/%u] %s", (unsigned)(i + 1U), (unsigned)count, s_files[i]);
        const esp_err_t err = replay_one((uint32_t)i, &buffers);
        if (err == ESP_OK) {
            ++dumped;
        } else {
            ESP_LOGE(TAG, "[%u/%u] %s failed: %s", (unsigned)(i + 1U), (unsigned)count, s_files[i],
                     esp_err_to_name(err));
        }
        vTaskDelay(1);
    }
    free_buffers(&buffers);
    ESP_LOGW(TAG, "Replay done: %u/%u dumped", (unsigned)dumped, (unsigned)count);
    return dumped == count ? ESP_OK : ESP_FAIL;
}

static void replay_task(void *arg)
{
    replay_context_t *context = (replay_context_t *)arg;
    context->result = replay_all();
    xTaskNotifyGive(context->caller);
    vTaskDelete(NULL);
}

esp_err_t debug_replay_run(void)
{
    replay_context_t context = {
        .caller = xTaskGetCurrentTaskHandle(),
        .result = ESP_FAIL,
    };
    /* app_main 在 Core 0 等待；重播時即時管線都還沒啟動，放哪個核心都不會互相干擾 */
#if CONFIG_FREERTOS_UNICORE
    const BaseType_t core = 0;
#else
    const BaseType_t core = 1;
#endif
    if (xTaskCreatePinnedToCore(replay_task, "debug_replay", REPLAY_TASK_STACK_BYTES, &context,
                                REPLAY_TASK_PRIORITY, NULL, core) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return context.result;
}
