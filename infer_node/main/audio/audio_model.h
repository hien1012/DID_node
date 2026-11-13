#pragma once

/*
 * 音訊 CNN (全 int8 log-Mel)。模型、logmel_features_int8.c、preprocess_config_int8.h 從
 * TTGO_I2S_Record_Test/main 原樣複製，三者是一組部署合約，要一起換。
 * interpreter 與 arena 是單例，只由 audio_cnn task 呼叫，不可重入。
 */
#include <stdint.h>

#include "esp_err.h"

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

#ifdef __cplusplus
}
#endif
