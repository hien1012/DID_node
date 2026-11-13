#pragma once

/**
 * @file logmel_features_int8.h
 * @brief Float32 audio frontend with an int8 TFLite input boundary.
 *
 * Signal processing remains float32 to preserve Python/C numerical parity.
 * Only the final normalized [32][124][1] tensor is quantized with the scale and
 * zero point generated from the TFLite model. This module allocates no heap.
 */

#include <stddef.h>
#include <stdint.h>

#include "preprocess_config_int8.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Fixed dimensions. Keep aligned with the generated preprocessing contract. */
#define AUDIO_LOGMEL_SAMPLE_RATE_HZ      16000
#define AUDIO_LOGMEL_RECORD_SECONDS      2
#define AUDIO_LOGMEL_INPUT_SAMPLES       32000
#define AUDIO_LOGMEL_N_FFT               512
#define AUDIO_LOGMEL_HOP_LENGTH          256
#define AUDIO_LOGMEL_FFT_BINS            257
#define AUDIO_LOGMEL_N_MELS              32
#define AUDIO_LOGMEL_FRAME_COUNT         124
#define AUDIO_LOGMEL_CHANNEL_COUNT       AUDIO_MODEL_INPUT_CHANNELS
#define AUDIO_LOGMEL_PLANE_VALUES        (AUDIO_LOGMEL_N_MELS * AUDIO_LOGMEL_FRAME_COUNT)
#define AUDIO_LOGMEL_MODEL_INPUT_VALUES  (AUDIO_LOGMEL_PLANE_VALUES * AUDIO_LOGMEL_CHANNEL_COUNT)

enum {
    AUDIO_LOGMEL_OK = 0,
    AUDIO_LOGMEL_ERR_INVALID_ARG = -1,
    AUDIO_LOGMEL_ERR_BAD_SAMPLE_COUNT = -2,
    AUDIO_LOGMEL_ERR_WORKSPACE_TOO_SMALL = -3,
};

/**
 * Optional buffers for Python/C stage comparison. All pointers may be NULL.
 * power_spectrogram uses [frame][fft_bin]; Mel arrays use [mel][frame].
 * normalized_input uses model NHWC order [mel][frame][channel].
 */
typedef struct {
    float *dc_removed_audio;       /* 32000 floats */
    float *power_spectrogram;      /* 124 * 257 floats */
    float *mel_power;              /* 32 * 124 floats */
    float *log_mel;                /* 32 * 124 floats */
    float *delta;                  /* 32 * 124 floats */
    float *cmvn;                   /* Optional 32 * 124 CMVN values */
    float *normalized_input;       /* 32 * 124 * model channel count */
    int8_t *quantized_input;       /* Final int8 NHWC model input */
} audio_logmel_int8_debug_buffers_t;

/** @return Exact int8 frontend workspace requirement in bytes. */
size_t audio_logmel_int8_workspace_size(void);

/**
 * @brief Convert PCM16 audio into the exact int8 tensor expected by TFLite.
 *
 * @param[in] pcm Fixed two-second mono capture.
 * @param[in] sample_count Must equal AUDIO_LOGMEL_INPUT_SAMPLES.
 * @param[out] model_input Quantized NHWC tensor in [mel][frame][channel] order.
 * @param[in,out] workspace Caller-owned float DSP scratch region.
 * @param[in] workspace_bytes Size of workspace in bytes.
 * @param[out] debug_buffers Optional parity outputs; pass NULL in production.
 * @return AUDIO_LOGMEL_OK on success, otherwise AUDIO_LOGMEL_ERR_*.
 *
 * Input must be exactly two seconds of mono 16 kHz PCM16. The returned model
 * frontend calculations remain float32, then the normalized NHWC values are
 * quantized with the TFLite input scale and zero point.
 */
int audio_logmel_extract_quantized_pcm16(
    const int16_t *pcm,
    size_t sample_count,
    int8_t model_input[AUDIO_LOGMEL_MODEL_INPUT_VALUES],
    void *workspace,
    size_t workspace_bytes,
    const audio_logmel_int8_debug_buffers_t *debug_buffers);

#ifdef __cplusplus
}
#endif
