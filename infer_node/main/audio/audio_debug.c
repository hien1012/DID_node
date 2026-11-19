/*
 * 音訊除錯 dump：正式推論路徑的一個視窗，經 console (UART log) 輸出每個前處理階段與 CNN 每一層。
 * 說明見 audio_debug.h；PC 端用 TTGO_I2S_Record_Test/tools/receive_infer_node_dump.py 接收還原。
 * 輸出格式 (@@DUMP / @@FILE / @@D) 與 nn_index.csv 見 debug_dump.c (與相機的 cam_debug.c 共用)。
 *
 * 檔案 (全部 little-endian 原始二進位)：
 *   input.wav             正式路徑的 PCM (16 kHz mono PCM16，32000 樣本)
 *   fe_dc_removed.f32     32000            去 DC 後的音訊          (僅 INFER_AUDIO_DEBUG_DUMP_FULL_FRONTEND)
 *   fe_power.f32          124 x 257        [frame][fft_bin] power  (僅 INFER_AUDIO_DEBUG_DUMP_FULL_FRONTEND)
 *   fe_mel_power.f32      32 x 124         [mel][frame]
 *   fe_log_mel.f32        32 x 124         減全段最大值、-80 dB 下限之後
 *   fe_delta.f32          32 x 124
 *   fe_cmvn.f32           32 x 124         (CMVN 模式)
 *   fe_normalized.f32     32 x 124 x C     量化前的模型輸入
 *   fe_quantized.i8       3968             同一份 PCM 打開 debug buffers 重算的 int8 輸入
 *   fe_quantized_prod.i8  3968             正式路徑實際送進 CNN 的 int8 輸入 (s_features)
 *   nn_index.csv                           每個張量的索引、型別、shape、量化參數、檔名、名稱
 *   nn_tNNN.bin                            每個非常數張量 (各層輸出) 的原始資料
 *   meta.json                              正式 / 除錯 interpreter 的機率、時間、模型長度
 */
#include "audio_debug.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "debug_dump.h"
#include "infer_config.h"
#include "logmel_features_int8.h"
#include "model_data_int8.h"
#include "preprocess_config_int8.h"

#ifndef INFER_AUDIO_DEBUG_DUMP_FULL_FRONTEND
#define INFER_AUDIO_DEBUG_DUMP_FULL_FRONTEND 0
#endif

/* main/CMakeLists.txt 由 ESP_NN_KERNEL_SET cache 變數帶進來 */
#ifndef INFER_ESP_NN_KERNEL_SET
#define INFER_ESP_NN_KERNEL_SET "unknown"
#endif

#define WAV_HEADER_BYTES      44U
#define WAV_BYTES             (WAV_HEADER_BYTES + AUDIO_LOGMEL_INPUT_SAMPLES * sizeof(int16_t))

static const char *TAG = "AUDIO_DEBUG";

typedef struct {
    int8_t *input_debug;
    uint8_t *wav;
    void *workspace;
    size_t workspace_bytes;
    audio_logmel_int8_debug_buffers_t stages;
} debug_buffers_t;

static debug_buffers_t s_buffers;
static bool s_buffers_ready = false;
/* 只在呼叫端 task 內使用；放 static 不佔 audio_cnn 的 stack */
static char s_meta[1536];

static void *psram_alloc(size_t bytes)
{
    return heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static bool allocate_buffers(void)
{
    if (s_buffers_ready) {
        return true;
    }
    debug_buffers_t *b = &s_buffers;
    b->workspace_bytes = audio_logmel_int8_workspace_size();
    b->workspace = psram_alloc(b->workspace_bytes);
    b->input_debug = psram_alloc(AUDIO_LOGMEL_MODEL_INPUT_VALUES);
    b->wav = psram_alloc(WAV_BYTES);
    b->stages.dc_removed_audio = psram_alloc(AUDIO_LOGMEL_INPUT_SAMPLES * sizeof(float));
    b->stages.power_spectrogram =
        psram_alloc(AUDIO_LOGMEL_FRAME_COUNT * AUDIO_LOGMEL_FFT_BINS * sizeof(float));
    b->stages.mel_power = psram_alloc(AUDIO_LOGMEL_PLANE_VALUES * sizeof(float));
    b->stages.log_mel = psram_alloc(AUDIO_LOGMEL_PLANE_VALUES * sizeof(float));
    b->stages.delta = psram_alloc(AUDIO_LOGMEL_PLANE_VALUES * sizeof(float));
    b->stages.cmvn = psram_alloc(AUDIO_LOGMEL_PLANE_VALUES * sizeof(float));
    b->stages.normalized_input = psram_alloc(AUDIO_LOGMEL_MODEL_INPUT_VALUES * sizeof(float));
    b->stages.quantized_input = psram_alloc(AUDIO_LOGMEL_MODEL_INPUT_VALUES);
    s_buffers_ready = b->workspace && b->input_debug && b->wav && b->stages.dc_removed_audio &&
                      b->stages.power_spectrogram && b->stages.mel_power && b->stages.log_mel &&
                      b->stages.delta && b->stages.cmvn && b->stages.normalized_input &&
                      b->stages.quantized_input;
    return s_buffers_ready;
}

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void build_wav(uint8_t *wav, const int16_t *pcm)
{
    const uint32_t data_bytes = AUDIO_LOGMEL_INPUT_SAMPLES * sizeof(int16_t);
    memcpy(wav, "RIFF", 4);
    put_le32(wav + 4, 36U + data_bytes);
    memcpy(wav + 8, "WAVEfmt ", 8);
    put_le32(wav + 16, 16U);
    put_le16(wav + 20, 1U);                                      /* PCM */
    put_le16(wav + 22, 1U);                                      /* mono */
    put_le32(wav + 24, AUDIO_LOGMEL_SAMPLE_RATE_HZ);
    put_le32(wav + 28, AUDIO_LOGMEL_SAMPLE_RATE_HZ * sizeof(int16_t));
    put_le16(wav + 32, sizeof(int16_t));
    put_le16(wav + 34, 16U);
    memcpy(wav + 36, "data", 4);
    put_le32(wav + 40, data_bytes);
    memcpy(wav + WAV_HEADER_BYTES, pcm, data_bytes);
}

static void emit_frontend(debug_dump_stats_t *stats, const int8_t *prod_features)
{
    const audio_logmel_int8_debug_buffers_t *s = &s_buffers.stages;
#if INFER_AUDIO_DEBUG_DUMP_FULL_FRONTEND
    debug_dump_file(stats, "fe_dc_removed.f32", s->dc_removed_audio, AUDIO_LOGMEL_INPUT_SAMPLES * sizeof(float));
    debug_dump_file(stats, "fe_power.f32", s->power_spectrogram,
                    AUDIO_LOGMEL_FRAME_COUNT * AUDIO_LOGMEL_FFT_BINS * sizeof(float));
#endif
    debug_dump_file(stats, "fe_mel_power.f32", s->mel_power, AUDIO_LOGMEL_PLANE_VALUES * sizeof(float));
    debug_dump_file(stats, "fe_log_mel.f32", s->log_mel, AUDIO_LOGMEL_PLANE_VALUES * sizeof(float));
    debug_dump_file(stats, "fe_delta.f32", s->delta, AUDIO_LOGMEL_PLANE_VALUES * sizeof(float));
#if AUDIO_MODEL_FEATURE_MODE == AUDIO_MODEL_FEATURE_CMVN
    debug_dump_file(stats, "fe_cmvn.f32", s->cmvn, AUDIO_LOGMEL_PLANE_VALUES * sizeof(float));
#endif
    debug_dump_file(stats, "fe_normalized.f32", s->normalized_input,
                    AUDIO_LOGMEL_MODEL_INPUT_VALUES * sizeof(float));
    debug_dump_file(stats, "fe_quantized.i8", s_buffers.input_debug, AUDIO_LOGMEL_MODEL_INPUT_VALUES);
    debug_dump_file(stats, "fe_quantized_prod.i8", prod_features, AUDIO_LOGMEL_MODEL_INPUT_VALUES);
}

esp_err_t audio_debug_dump_window(const audio_debug_window_t *w)
{
    if (w == NULL || w->pcm == NULL || w->features == NULL || w->probs == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const int64_t started_us = esp_timer_get_time();
    if (!allocate_buffers()) {
        ESP_LOGE(TAG, "PSRAM allocation failed");
        return ESP_ERR_NO_MEM;
    }
    const bool replay = w->replay_name != NULL;
    char stem[96];
    if (replay) {
        snprintf(stem, sizeof(stem), "%.63s_%s", w->replay_name, INFER_ESP_NN_KERNEL_SET);
    } else {
        snprintf(stem, sizeof(stem), "w%05u_%lld", (unsigned)w->window, (long long)w->start_epoch_ms);
    }

    /* 同一份 PCM 打開 debug buffers 重算；用自己的 workspace，不碰 audio_feature 的 */
    const int fe_err = audio_logmel_extract_quantized_pcm16(
        w->pcm, AUDIO_LOGMEL_INPUT_SAMPLES, s_buffers.input_debug, s_buffers.workspace,
        s_buffers.workspace_bytes, &s_buffers.stages);
    if (fe_err != AUDIO_LOGMEL_OK) {
        ESP_LOGE(TAG, "logmel (debug) failed: %d", fe_err);
        return ESP_FAIL;
    }
    int feature_mismatch = 0;
    for (size_t i = 0; i < AUDIO_LOGMEL_MODEL_INPUT_VALUES; ++i) {
        feature_mismatch += s_buffers.input_debug[i] != w->features[i];
    }
    /* 先複製 PCM：輸出要幾十秒，期間只讀自己的副本 */
    build_wav(s_buffers.wav, w->pcm);

    ESP_LOGW(TAG, "Dumping window %u over the console as %s (~40 s at 115200 baud)...",
             (unsigned)w->window, stem);
    debug_dump_stats_t stats;
    debug_dump_begin(&stats, stem);
    debug_dump_file(&stats, "input.wav", s_buffers.wav, WAV_BYTES);
    emit_frontend(&stats, w->features);

    /* 保留所有張量的 interpreter，輸入 = 正式路徑實際送進 CNN 的特徵；每一層在 callback 裡直接輸出 */
    debug_dump_index_reset();
    float dump_probs[AUDIO_MODEL_CLASS_COUNT] = {0};
    int dump_pred = -1;
    size_t arena_used = 0;
    const esp_err_t dump_err = audio_model_debug_run(w->features, dump_probs, &dump_pred, &arena_used,
                                                     debug_dump_tensor, &stats);
    if (dump_err != ESP_OK) {
        ++stats.errors;
    }
    debug_dump_index_emit(&stats);

    const int length = snprintf(
        s_meta, sizeof(s_meta),
        "{\n"
        " \"source\": \"%s\",\n"
        " \"kind\": \"audio_window\",\n"
        " \"input_file\": \"%.63s\",\n"
        " \"esp_nn_kernel_set\": \"%s\",\n"
        " \"window\": %u,\n"
        " \"start_ms\": %lld,\n"
        " \"samples\": %u,\n"
        " \"model_bytes\": %u,\n"
        " \"input_scale\": %.9g,\n"
        " \"input_zero_point\": %d,\n"
        " \"live_feature_us\": %u,\n"
        " \"live_invoke_us\": %u,\n"
        " \"live_vs_recomputed_feature_mismatch\": %d,\n"
        " \"prod_pred\": \"%s\",\n"
        " \"prod_probs\": [%.6f, %.6f, %.6f],\n"
        " \"dump_model_ok\": %s,\n"
        " \"dump_pred\": \"%s\",\n"
        " \"dump_probs\": [%.6f, %.6f, %.6f],\n"
        " \"dump_arena_used\": %u,\n"
        " \"full_frontend\": %s,\n"
        " \"labels\": [\"%s\", \"%s\", \"%s\"]\n"
        "}\n",
        replay ? "sd_replay" : "live_capture", replay ? w->replay_name : "", INFER_ESP_NN_KERNEL_SET,
        (unsigned)w->window, (long long)w->start_epoch_ms, (unsigned)AUDIO_LOGMEL_INPUT_SAMPLES,
        (unsigned)g_audio_model_int8_tflite_len, (double)AUDIO_MODEL_INPUT_SCALE, AUDIO_MODEL_INPUT_ZERO_POINT,
        (unsigned)w->feature_us, (unsigned)w->invoke_us, feature_mismatch,
        w->pred >= 0 ? g_audio_class_names[w->pred] : "",
        (double)w->probs[0], (double)w->probs[1], (double)w->probs[2],
        dump_err == ESP_OK ? "true" : "false", dump_pred >= 0 ? g_audio_class_names[dump_pred] : "",
        (double)dump_probs[0], (double)dump_probs[1], (double)dump_probs[2], (unsigned)arena_used,
        INFER_AUDIO_DEBUG_DUMP_FULL_FRONTEND ? "true" : "false",
        g_audio_class_names[0], g_audio_class_names[1], g_audio_class_names[2]);
    if (length > 0 && length < (int)sizeof(s_meta)) {
        debug_dump_file(&stats, "meta.json", s_meta, (size_t)length);
    } else {
        ++stats.errors;
    }
    debug_dump_end(&stats, stem);

    ESP_LOGW(TAG, "Window %u: live=%s [%.3f %.3f %.3f] debug_interp=%s [%.3f %.3f %.3f] "
             "live_vs_recomputed_features_mismatch=%d files=%d errors=%d (%.1f s)",
             (unsigned)w->window, w->pred >= 0 ? g_audio_class_names[w->pred] : "?",
             (double)w->probs[0], (double)w->probs[1], (double)w->probs[2],
             dump_pred >= 0 ? g_audio_class_names[dump_pred] : "?",
             (double)dump_probs[0], (double)dump_probs[1], (double)dump_probs[2],
             feature_mismatch, stats.files, stats.errors,
             (double)(esp_timer_get_time() - started_us) / 1e6);
    return stats.errors == 0 ? ESP_OK : ESP_FAIL;
}
