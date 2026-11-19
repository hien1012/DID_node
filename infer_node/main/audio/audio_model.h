#pragma once

/*
 * 音訊 CNN (全 int8 log-Mel)。模型、logmel_features_int8.c、preprocess_config_int8.h 從
 * TTGO_I2S_Record_Test/main 原樣複製，三者是一組部署合約，要一起換。
 * interpreter 與 arena 是單例，只由 audio_cnn task 呼叫，不可重入。
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "debug_dump.h"
#include "logmel_features_int8.h"
#include "preprocess_config_int8.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t audio_model_init(void);

/** 輸入是 logmel_features_int8.c 量化好的張量；輸出已反量化成機率。 */
esp_err_t audio_model_predict(const int8_t quantized_features[AUDIO_LOGMEL_MODEL_INPUT_VALUES],
                              float probabilities[AUDIO_MODEL_CLASS_COUNT],
                              int *predicted_class);

/** 上一次 audio_model_predict() 裡 Invoke() 本身的時間 (不含複製輸入、反量化)；失敗時是 0。 */
uint32_t audio_model_last_invoke_us(void);

/* ---- 除錯 (INFER_AUDIO_DEBUG_DUMP，audio_debug.c)：每一層的輸出 ---- */

/**
 * 用另一個 preserve_all_tensors 的 interpreter (2 MB PSRAM arena，第一次呼叫時建立) 推論，
 * 再對每個張量呼叫 tensor_cb (debug_dump.h)。正式的 interpreter 不受影響。
 * 只給除錯模式用，不可與 audio_cnn 同時呼叫。
 */
esp_err_t audio_model_debug_run(const int8_t quantized_features[AUDIO_LOGMEL_MODEL_INPUT_VALUES],
                                float probabilities[AUDIO_MODEL_CLASS_COUNT],
                                int *predicted_class,
                                size_t *arena_used_bytes,
                                debug_tensor_cb_t tensor_cb,
                                void *ctx);

#ifdef __cplusplus
}
#endif
