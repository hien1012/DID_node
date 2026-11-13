/**
 * @file logmel_features_int8.c
 * @brief Float32 log-Mel CMVN frontend followed by TFLite int8 quantization.
 *
 * FFT, Mel filtering, logarithm, and CMVN intentionally remain float32. This
 * isolates model quantization from frontend approximation error and keeps the
 * Python/C contract testable. Quantization occurs only when each final NHWC
 * value is written to model_input.
 */
#include "logmel_features_int8.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "preprocess_config_int8.h"

#if AUDIO_MODEL_INPUT_HEIGHT != AUDIO_LOGMEL_N_MELS || \
    AUDIO_MODEL_INPUT_WIDTH != AUDIO_LOGMEL_FRAME_COUNT || \
    AUDIO_MODEL_INPUT_CHANNELS != AUDIO_LOGMEL_CHANNEL_COUNT || \
    AUDIO_MODEL_INPUT_VALUES != AUDIO_LOGMEL_MODEL_INPUT_VALUES
#error "Generated model input contract does not match logmel_features.h"
#endif

#if AUDIO_MODEL_FEATURE_MODE == AUDIO_MODEL_FEATURE_LOGMEL
#if AUDIO_MODEL_INPUT_CHANNELS != 1
#error "Raw log-mel models require exactly one input channel"
#endif
#elif AUDIO_MODEL_FEATURE_MODE == AUDIO_MODEL_FEATURE_LOGMEL_DELTA
#if AUDIO_MODEL_INPUT_CHANNELS != 2
#error "Log-mel+delta models require exactly two input channels"
#endif
#elif AUDIO_MODEL_FEATURE_MODE == AUDIO_MODEL_FEATURE_CMVN
#if AUDIO_MODEL_INPUT_CHANNELS != 1
#error "CMVN log-mel models require exactly one input channel"
#endif
#else
#error "Unsupported generated log-mel feature mode"
#endif

#ifndef AUDIO_MODEL_MEL_F_MIN_HZ
#error "preprocess_config_int8.h lacks the mel band; regenerate it with tools/convert_logmel_tflite_to_c_int8.py"
#endif

#define AUDIO_LOG_FLOOR 1.0e-12f
#define AUDIO_LOG_TOP_DB 80.0f
#define AUDIO_PI 3.14159265358979323846

/* Caller-owned DSP scratch. No heap allocation occurs inside the extractor. */
typedef struct {
    float fft_re[AUDIO_LOGMEL_N_FFT];
    float fft_im[AUDIO_LOGMEL_N_FFT];
    float power[AUDIO_LOGMEL_FFT_BINS];
    float hamming[AUDIO_LOGMEL_N_FFT];
    float twiddle_cos[AUDIO_LOGMEL_N_FFT / 2];
    float twiddle_sin[AUDIO_LOGMEL_N_FFT / 2];
    float log_mel[AUDIO_LOGMEL_PLANE_VALUES];
} audio_logmel_workspace_t;

static void initialize_tables(audio_logmel_workspace_t *work)
{
    /* Regenerate tables per call so no mutable global initialization is needed. */
    const double denominator = (double)(AUDIO_LOGMEL_N_FFT - 1);
    for (int i = 0; i < AUDIO_LOGMEL_N_FFT; ++i) {
        double phase = 2.0 * AUDIO_PI * (double)i / denominator;
        work->hamming[i] = (float)(0.54 - 0.46 * cos(phase));
    }

    for (int i = 0; i < AUDIO_LOGMEL_N_FFT / 2; ++i) {
        double phase = 2.0 * AUDIO_PI * (double)i / (double)AUDIO_LOGMEL_N_FFT;
        work->twiddle_cos[i] = (float)cos(phase);
        work->twiddle_sin[i] = (float)sin(phase);
    }
    /* Mel filter edges come from g_audio_mel_bin_edges in preprocess_config_int8.h. */
}

static void fft_radix2(audio_logmel_workspace_t *work)
{
    /* In-place radix-2 FFT; changing N_FFT requires a power-of-two value. */
    int j = 0;
    for (int i = 1; i < AUDIO_LOGMEL_N_FFT; ++i) {
        int bit = AUDIO_LOGMEL_N_FFT >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if (i < j) {
            float tmp = work->fft_re[i];
            work->fft_re[i] = work->fft_re[j];
            work->fft_re[j] = tmp;
            tmp = work->fft_im[i];
            work->fft_im[i] = work->fft_im[j];
            work->fft_im[j] = tmp;
        }
    }

    for (int length = 2; length <= AUDIO_LOGMEL_N_FFT; length <<= 1) {
        int half = length >> 1;
        int twiddle_step = AUDIO_LOGMEL_N_FFT / length;
        for (int start = 0; start < AUDIO_LOGMEL_N_FFT; start += length) {
            for (int offset = 0; offset < half; ++offset) {
                int twiddle_index = offset * twiddle_step;
                float wr = work->twiddle_cos[twiddle_index];
                float wi = -work->twiddle_sin[twiddle_index];
                int even = start + offset;
                int odd = even + half;
                float odd_re = work->fft_re[odd];
                float odd_im = work->fft_im[odd];
                float tr = wr * odd_re - wi * odd_im;
                float ti = wr * odd_im + wi * odd_re;
                float even_re = work->fft_re[even];
                float even_im = work->fft_im[even];
                work->fft_re[even] = even_re + tr;
                work->fft_im[even] = even_im + ti;
                work->fft_re[odd] = even_re - tr;
                work->fft_im[odd] = even_im - ti;
            }
        }
    }
}

static float mel_filter_power(const audio_logmel_workspace_t *work, int mel_index)
{
    /* Sum one triangular Mel filter directly from the current power spectrum. */
    int left = g_audio_mel_bin_edges[mel_index];
    int center = g_audio_mel_bin_edges[mel_index + 1];
    int right = g_audio_mel_bin_edges[mel_index + 2];
    float sum = 0.0f;

    if (center > left) {
        float width = (float)(center - left);
        for (int bin = left; bin < center; ++bin) {
            float weight = (float)(bin - left) / width;
            sum += weight * work->power[bin];
        }
    }
    if (right > center) {
        float width = (float)(right - center);
        for (int bin = center; bin < right; ++bin) {
            float weight = 1.0f - (float)(bin - center) / width;
            sum += weight * work->power[bin];
        }
    }
    return sum > 0.0f ? sum : 0.0f;
}

static int8_t quantize_model_value(float normalized_value)
{
    /* TFLite affine quantization: q = round(real / scale) + zero_point. */
    const float transformed =
        normalized_value / AUDIO_MODEL_INPUT_SCALE +
        (float)AUDIO_MODEL_INPUT_ZERO_POINT;
    long quantized = lrintf(transformed);
    if (quantized > INT8_MAX) {
        quantized = INT8_MAX;
    } else if (quantized < INT8_MIN) {
        quantized = INT8_MIN;
    }
    return (int8_t)quantized;
}

size_t audio_logmel_int8_workspace_size(void)
{
    return sizeof(audio_logmel_workspace_t);
}

int audio_logmel_extract_quantized_pcm16(
    const int16_t *pcm,
    size_t sample_count,
    int8_t model_input[AUDIO_LOGMEL_MODEL_INPUT_VALUES],
    void *workspace,
    size_t workspace_bytes,
    const audio_logmel_int8_debug_buffers_t *debug_buffers)
{
    if (pcm == NULL || model_input == NULL || workspace == NULL) {
        return AUDIO_LOGMEL_ERR_INVALID_ARG;
    }
    if (sample_count != AUDIO_LOGMEL_INPUT_SAMPLES) {
        return AUDIO_LOGMEL_ERR_BAD_SAMPLE_COUNT;
    }
    if (workspace_bytes < sizeof(audio_logmel_workspace_t)) {
        return AUDIO_LOGMEL_ERR_WORKSPACE_TOO_SMALL;
    }

    audio_logmel_workspace_t *work = (audio_logmel_workspace_t *)workspace;
    initialize_tables(work);

    /* Phase 1: remove one DC value shared by the complete two-second clip. */
    double sum = 0.0;
    for (size_t i = 0; i < sample_count; ++i) {
        sum += (double)pcm[i] / 32767.0;
    }
    const float dc_mean = (float)(sum / (double)sample_count);
    float global_log_max = -INFINITY;

    /* Phase 2: generate float log-Mel values and the sample-relative dB peak. */
    for (int frame = 0; frame < AUDIO_LOGMEL_FRAME_COUNT; ++frame) {
        int start = frame * AUDIO_LOGMEL_HOP_LENGTH;
        for (int i = 0; i < AUDIO_LOGMEL_N_FFT; ++i) {
            float centered = (float)pcm[start + i] / 32767.0f - dc_mean;
            work->fft_re[i] = centered * work->hamming[i];
            work->fft_im[i] = 0.0f;
        }
        fft_radix2(work);

        for (int bin = 0; bin < AUDIO_LOGMEL_FFT_BINS; ++bin) {
            float re = work->fft_re[bin];
            float im = work->fft_im[bin];
            float value = re * re + im * im;
            work->power[bin] = value;
            if (debug_buffers != NULL && debug_buffers->power_spectrogram != NULL) {
                debug_buffers->power_spectrogram[
                    frame * AUDIO_LOGMEL_FFT_BINS + bin] = value;
            }
        }

        for (int mel = 0; mel < AUDIO_LOGMEL_N_MELS; ++mel) {
            float mel_power = mel_filter_power(work, mel);
            int index = mel * AUDIO_LOGMEL_FRAME_COUNT + frame;
            if (debug_buffers != NULL && debug_buffers->mel_power != NULL) {
                debug_buffers->mel_power[index] = mel_power;
            }
            float log_value = 10.0f * log10f(
                mel_power > AUDIO_LOG_FLOOR ? mel_power : AUDIO_LOG_FLOOR);
            work->log_mel[index] = log_value;
            if (log_value > global_log_max) {
                global_log_max = log_value;
            }
        }
    }

    if (debug_buffers != NULL && debug_buffers->dc_removed_audio != NULL) {
        for (size_t i = 0; i < sample_count; ++i) {
            debug_buffers->dc_removed_audio[i] =
                (float)pcm[i] / 32767.0f - dc_mean;
        }
    }

    /*
     * Phase 3: CMVN, training normalization, and immediate int8 conversion.
     * Immediate conversion avoids a 3968-element float model-input buffer.
     */
    for (int mel = 0; mel < AUDIO_LOGMEL_N_MELS; ++mel) {
#if AUDIO_MODEL_FEATURE_MODE == AUDIO_MODEL_FEATURE_CMVN
        /* Python uses population statistics over time for each Mel band. */
        float cmvn_sum = 0.0f;
        for (int frame = 0; frame < AUDIO_LOGMEL_FRAME_COUNT; ++frame) {
            int plane_index = mel * AUDIO_LOGMEL_FRAME_COUNT + frame;
            float value = work->log_mel[plane_index] - global_log_max;
            if (value < -AUDIO_LOG_TOP_DB) {
                value = -AUDIO_LOG_TOP_DB;
            }
            cmvn_sum += value;
        }
        const float cmvn_mean = cmvn_sum / (float)AUDIO_LOGMEL_FRAME_COUNT;

        float cmvn_squared_sum = 0.0f;
        for (int frame = 0; frame < AUDIO_LOGMEL_FRAME_COUNT; ++frame) {
            int plane_index = mel * AUDIO_LOGMEL_FRAME_COUNT + frame;
            float value = work->log_mel[plane_index] - global_log_max;
            if (value < -AUDIO_LOG_TOP_DB) {
                value = -AUDIO_LOG_TOP_DB;
            }
            float difference = value - cmvn_mean;
            cmvn_squared_sum += difference * difference;
        }
        float cmvn_std = sqrtf(cmvn_squared_sum / (float)AUDIO_LOGMEL_FRAME_COUNT);
        if (cmvn_std < AUDIO_MODEL_CMVN_STD_FLOOR_DB) {
            cmvn_std = AUDIO_MODEL_CMVN_STD_FLOOR_DB;
        }
        const float cmvn_inv_std = 1.0f / cmvn_std;
#endif

        float previous = 0.0f;
        for (int frame = 0; frame < AUDIO_LOGMEL_FRAME_COUNT; ++frame) {
            int plane_index = mel * AUDIO_LOGMEL_FRAME_COUNT + frame;
            float log_value = work->log_mel[plane_index] - global_log_max;
            if (log_value < -AUDIO_LOG_TOP_DB) {
                log_value = -AUDIO_LOG_TOP_DB;
            }
            float delta = frame == 0 ? 0.0f : log_value - previous;
            previous = log_value;

#if AUDIO_MODEL_FEATURE_MODE == AUDIO_MODEL_FEATURE_LOGMEL
            const float normalized_logmel =
                (log_value - g_audio_input_mean[0]) * g_audio_input_inv_std[0];
            model_input[plane_index] = quantize_model_value(normalized_logmel);
            if (debug_buffers != NULL && debug_buffers->normalized_input != NULL) {
                debug_buffers->normalized_input[plane_index] = normalized_logmel;
            }
#elif AUDIO_MODEL_FEATURE_MODE == AUDIO_MODEL_FEATURE_LOGMEL_DELTA
            int input_index = plane_index * AUDIO_LOGMEL_CHANNEL_COUNT;
            const float normalized_logmel =
                (log_value - g_audio_input_mean[0]) * g_audio_input_inv_std[0];
            const float normalized_delta =
                (delta - g_audio_input_mean[1]) * g_audio_input_inv_std[1];
            model_input[input_index] = quantize_model_value(normalized_logmel);
            model_input[input_index + 1] = quantize_model_value(normalized_delta);
            if (debug_buffers != NULL && debug_buffers->normalized_input != NULL) {
                debug_buffers->normalized_input[input_index] = normalized_logmel;
                debug_buffers->normalized_input[input_index + 1] = normalized_delta;
            }
#elif AUDIO_MODEL_FEATURE_MODE == AUDIO_MODEL_FEATURE_CMVN
            float cmvn_value = (log_value - cmvn_mean) * cmvn_inv_std;
            const float normalized_cmvn =
                (cmvn_value - g_audio_input_mean[0]) * g_audio_input_inv_std[0];
            model_input[plane_index] = quantize_model_value(normalized_cmvn);
            if (debug_buffers != NULL && debug_buffers->normalized_input != NULL) {
                debug_buffers->normalized_input[plane_index] = normalized_cmvn;
            }
#endif

            if (debug_buffers != NULL) {
                if (debug_buffers->log_mel != NULL) {
                    debug_buffers->log_mel[plane_index] = log_value;
                }
                if (debug_buffers->delta != NULL) {
                    debug_buffers->delta[plane_index] = delta;
                }
#if AUDIO_MODEL_FEATURE_MODE == AUDIO_MODEL_FEATURE_CMVN
                if (debug_buffers->cmvn != NULL) {
                    debug_buffers->cmvn[plane_index] = cmvn_value;
                }
#endif
            }
        }
    }

    if (debug_buffers != NULL && debug_buffers->quantized_input != NULL) {
        memcpy(debug_buffers->quantized_input,
               model_input,
               AUDIO_LOGMEL_MODEL_INPUT_VALUES * sizeof(int8_t));
    }
    return AUDIO_LOGMEL_OK;
}
