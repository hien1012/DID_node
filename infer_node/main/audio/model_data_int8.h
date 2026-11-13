#pragma once

/**
 * @file model_data_int8.h
 * @brief Linkage contract for the generated full-INT8 TFLite flatbuffer.
 *
 * The const model array is stored in Flash. Regenerate this header,
 * model_data_int8.cc, and preprocess_config_int8.h as one model release.
 */

#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
/** Embedded full-INT8 TFLite flatbuffer and its exact byte length. */
extern const unsigned char g_audio_model_int8_tflite[];
extern const size_t g_audio_model_int8_tflite_len;
#ifdef __cplusplus
}
#endif
