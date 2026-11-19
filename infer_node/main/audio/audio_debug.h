#pragma once

/*
 * 音訊除錯 dump (infer_config.h 的 INFER_AUDIO_DEBUG_DUMP=1)：
 * 正常推論流程照常運作；audio_cnn 對前 INFER_AUDIO_DEBUG_DUMP_WINDOWS 個視窗做完正式推論後，
 * 在視窗釋放前呼叫 audio_debug_dump_window()，把
 *   - 這個視窗的 PCM (input.wav，就是正式路徑送進 logmel 的資料)
 *   - 正式路徑實際送進 CNN 的 int8 特徵與推論結果
 *   - 同一份 PCM 打開 debug buffers 重算的每個前處理階段
 *   - 保留所有張量的 interpreter 對正式特徵推論的每一層輸出
 * 以 base64 文字行從 console (UART0) 輸出，不寫 SD。PC 端：
 *   TTGO_I2S_Record_Test/tools/receive_infer_node_dump.py  接收、驗證 CRC、還原成檔案
 *   DID_NODE/tools/compare_debug_dumps.py                  逐步比對 (含 Python 訓練前處理)
 * SD 重播 (debug_replay.c) 也走這裡，replay_name 不是 NULL。
 */
#include <stdint.h>

#include "esp_err.h"

#include "audio_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 正式推論路徑的一個視窗；所有指標在呼叫期間必須有效 (視窗尚未釋放)。 */
typedef struct {
    const int16_t *pcm;                 /* AUDIO_LOGMEL_INPUT_SAMPLES 個樣本 */
    const int8_t *features;             /* 正式 logmel 的輸出 = 正式 CNN 的輸入 */
    const float *probs;                 /* 正式 CNN 的機率 (AUDIO_MODEL_CLASS_COUNT) */
    int pred;
    uint32_t window;
    int64_t start_epoch_ms;
    uint32_t feature_us;
    uint32_t invoke_us;
    /*
     * SD 重播 (debug_replay.c) 的輸入檔名，不含副檔名，例如 "mic_drone01"；NULL = 即時擷取。
     * 重播時 dump 名稱是 <replay_name>_<ESP_NN_KERNEL_SET>，PC 端靠它配對兩種 kernel 的 dump。
     */
    const char *replay_name;
} audio_debug_window_t;

/**
 * 在呼叫端 task 裡同步輸出 (115200 baud 約 40 s)；期間不會處理新視窗，錄音端會丟掉視窗。
 * 自己配置所需的 PSRAM (第一次呼叫時)，不使用正式路徑的 workspace。
 */
esp_err_t audio_debug_dump_window(const audio_debug_window_t *window);

#ifdef __cplusplus
}
#endif
