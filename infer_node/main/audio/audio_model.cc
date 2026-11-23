/* 改寫自 TTGO_I2S_Record_Test/main/model_runner_int8.cc：拿掉 memory_monitor，其餘相同。 */
#include "audio_model.h"

#include <cmath>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "debug_tensors.h"
#include "esp_timer.h"
#include "model_data_int8.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

namespace {

constexpr char TAG[] = "AUDIO_MODEL";
/* Int8 activations peak at about 80 KB; keep the source project's margin.
 * The boot log prints the bytes actually used. */
constexpr int kTensorArenaSize = 256 * 1024;
constexpr float kScaleTolerance = 1.0e-7f;

uint32_t s_last_invoke_us = 0;

/* INFER_MODEL_IN_PSRAM=1：模型從 flash 複製到 PSRAM 再交給 TFLM，同 vision/cam_model.cc (理由在那裡) */
#ifndef INFER_MODEL_IN_PSRAM
#define INFER_MODEL_IN_PSRAM 0
#endif
const unsigned char *s_model_data = nullptr;

const unsigned char *model_data()
{
    if (s_model_data != nullptr) {
        return s_model_data;
    }
    s_model_data = g_audio_model_int8_tflite;
#if INFER_MODEL_IN_PSRAM
    auto *copy = static_cast<unsigned char *>(heap_caps_aligned_alloc(
        16, g_audio_model_int8_tflite_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (copy != nullptr) {
        std::memcpy(copy, g_audio_model_int8_tflite, g_audio_model_int8_tflite_len);
        s_model_data = copy;
    } else {
        ESP_LOGW(TAG, "Copying the model to PSRAM failed; using it from flash");
    }
#endif
    return s_model_data;
}

uint8_t *s_tensor_arena = nullptr;
tflite::MicroInterpreter *s_interpreter = nullptr;
TfLiteTensor *s_input = nullptr;
TfLiteTensor *s_output = nullptr;

/* 除錯 interpreter (audio_model_debug_run)：保留所有張量 = 線性配置，不重用記憶體，所以 arena 要大很多 */
constexpr size_t kDebugArenaSize = 2 * 1024 * 1024;
uint8_t *s_debug_arena = nullptr;
tflite::MicroInterpreter *s_debug_interpreter = nullptr;
bool s_debug_init_attempted = false;

using AudioOpResolver = tflite::MicroMutableOpResolver<6>;

/* Keep synchronized with the model's operators (logmel_conversion_summary_int8.json). */
TfLiteStatus register_ops(AudioOpResolver &resolver)
{
    if (resolver.AddAveragePool2D() != kTfLiteOk || resolver.AddConv2D() != kTfLiteOk ||
        resolver.AddDepthwiseConv2D() != kTfLiteOk || resolver.AddMaxPool2D() != kTfLiteOk ||
        resolver.AddMean() != kTfLiteOk || resolver.AddSoftmax() != kTfLiteOk) {
        return kTfLiteError;
    }
    return kTfLiteOk;
}

/* int8 輸出 → 機率，與 audio_model_predict() 相同：先出現的最大值勝出 */
int dequantize_output(const TfLiteTensor *output, float probabilities[AUDIO_MODEL_CLASS_COUNT])
{
    int best = 0;
    for (int i = 0; i < AUDIO_MODEL_CLASS_COUNT; i++) {
        probabilities[i] = (static_cast<int32_t>(output->data.int8[i]) -
                            AUDIO_MODEL_OUTPUT_ZERO_POINT) * AUDIO_MODEL_OUTPUT_SCALE;
        if (probabilities[i] > probabilities[best]) {
            best = i;
        }
    }
    return best;
}

}  // namespace

extern "C" esp_err_t audio_model_init(void)
{
    if (s_interpreter != nullptr) {
        return ESP_OK;
    }

    const tflite::Model *model = tflite::GetModel(model_data());
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema %ld != supported %d",
                 static_cast<long>(model->version()), TFLITE_SCHEMA_VERSION);
        return ESP_FAIL;
    }

    s_tensor_arena = static_cast<uint8_t *>(heap_caps_aligned_alloc(
        16, kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (s_tensor_arena == nullptr) {
        ESP_LOGE(TAG, "Tensor arena allocation failed: %d bytes", kTensorArenaSize);
        return ESP_ERR_NO_MEM;
    }

    static AudioOpResolver resolver;
    if (register_ops(resolver) != kTfLiteOk) {
        ESP_LOGE(TAG, "Op resolver setup failed");
        return ESP_FAIL;
    }

    static tflite::MicroInterpreter interpreter(model, resolver, s_tensor_arena, kTensorArenaSize);
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors failed");
        return ESP_FAIL;
    }

    /* Reject a model that does not match the compiled frontend contract. */
    s_input = interpreter.input(0);
    s_output = interpreter.output(0);
    if (s_input->type != kTfLiteInt8 || s_output->type != kTfLiteInt8) {
        ESP_LOGE(TAG, "Expected int8 input/output, got input=%d output=%d",
                 s_input->type, s_output->type);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_input->dims->size != 4 || s_input->dims->data[0] != 1 ||
        s_input->dims->data[1] != AUDIO_LOGMEL_N_MELS ||
        s_input->dims->data[2] != AUDIO_LOGMEL_FRAME_COUNT ||
        s_input->dims->data[3] != AUDIO_LOGMEL_CHANNEL_COUNT) {
        ESP_LOGE(TAG, "Unexpected input shape");
        return ESP_ERR_INVALID_SIZE;
    }
    if (s_output->dims->size != 2 || s_output->dims->data[0] != 1 ||
        s_output->dims->data[1] != AUDIO_MODEL_CLASS_COUNT) {
        ESP_LOGE(TAG, "Unexpected output shape");
        return ESP_ERR_INVALID_SIZE;
    }
    /* Catch stale generated headers before they silently corrupt predictions. */
    if (std::fabs(s_input->params.scale - AUDIO_MODEL_INPUT_SCALE) > kScaleTolerance ||
        s_input->params.zero_point != AUDIO_MODEL_INPUT_ZERO_POINT ||
        std::fabs(s_output->params.scale - AUDIO_MODEL_OUTPUT_SCALE) > kScaleTolerance ||
        s_output->params.zero_point != AUDIO_MODEL_OUTPUT_ZERO_POINT) {
        ESP_LOGE(TAG, "Generated int8 quantization constants do not match model tensors");
        return ESP_ERR_INVALID_STATE;
    }

    s_interpreter = &interpreter;
    ESP_LOGI(TAG, "Audio int8 model ready: model=%u bytes (%s) arena=%d used=%u bytes",
             static_cast<unsigned>(g_audio_model_int8_tflite_len),
             s_model_data == g_audio_model_int8_tflite ? "flash" : "PSRAM", kTensorArenaSize,
             static_cast<unsigned>(s_interpreter->arena_used_bytes()));
    return ESP_OK;
}

extern "C" esp_err_t audio_model_predict(
    const int8_t quantized_features[AUDIO_LOGMEL_MODEL_INPUT_VALUES],
    float probabilities[AUDIO_MODEL_CLASS_COUNT],
    int *predicted_class)
{
    if (!quantized_features || !probabilities || !predicted_class) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_interpreter == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    std::memcpy(s_input->data.int8, quantized_features, AUDIO_LOGMEL_MODEL_INPUT_VALUES);
    s_last_invoke_us = 0;
    const int64_t invoke_start_us = esp_timer_get_time();
    if (s_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke failed");
        return ESP_FAIL;
    }
    s_last_invoke_us = static_cast<uint32_t>(esp_timer_get_time() - invoke_start_us);

    *predicted_class = dequantize_output(s_output, probabilities);
    return ESP_OK;
}

extern "C" uint32_t audio_model_last_invoke_us(void)
{
    return s_last_invoke_us;
}

/* 第一次呼叫時建立保留所有張量的 interpreter；與正式的 interpreter 分開，正式路徑不受影響 */
static esp_err_t debug_interpreter_init(const tflite::Model *model)
{
    if (s_debug_interpreter != nullptr) {
        return ESP_OK;
    }
    if (s_debug_init_attempted) {
        return ESP_ERR_INVALID_STATE;
    }
    s_debug_init_attempted = true;

    s_debug_arena = static_cast<uint8_t *>(heap_caps_aligned_alloc(
        16, kDebugArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (s_debug_arena == nullptr) {
        ESP_LOGE(TAG, "Debug arena allocation failed: %u bytes", static_cast<unsigned>(kDebugArenaSize));
        return ESP_ERR_NO_MEM;
    }
    static AudioOpResolver resolver;
    if (register_ops(resolver) != kTfLiteOk) {
        ESP_LOGE(TAG, "Debug op resolver setup failed");
        return ESP_FAIL;
    }
    static tflite::MicroInterpreter interpreter(model, resolver, s_debug_arena, kDebugArenaSize,
                                                nullptr, nullptr, /*preserve_all_tensors=*/true);
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "Debug AllocateTensors failed (arena %u bytes)", static_cast<unsigned>(kDebugArenaSize));
        return ESP_FAIL;
    }
    s_debug_interpreter = &interpreter;
    ESP_LOGI(TAG, "Debug interpreter ready (preserve_all_tensors): arena used=%u bytes",
             static_cast<unsigned>(interpreter.arena_used_bytes()));
    return ESP_OK;
}

extern "C" esp_err_t audio_model_debug_run(
    const int8_t quantized_features[AUDIO_LOGMEL_MODEL_INPUT_VALUES],
    float probabilities[AUDIO_MODEL_CLASS_COUNT],
    int *predicted_class,
    size_t *arena_used_bytes,
    debug_tensor_cb_t tensor_cb,
    void *ctx)
{
    if (!quantized_features || !probabilities || !predicted_class) {
        return ESP_ERR_INVALID_ARG;
    }
    const tflite::Model *model = tflite::GetModel(model_data());
    const esp_err_t init_err = debug_interpreter_init(model);
    if (init_err != ESP_OK) {
        return init_err;
    }

    TfLiteTensor *input = s_debug_interpreter->input(0);
    std::memcpy(input->data.int8, quantized_features, AUDIO_LOGMEL_MODEL_INPUT_VALUES);
    if (s_debug_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Debug Invoke failed");
        return ESP_FAIL;
    }
    *predicted_class = dequantize_output(s_debug_interpreter->output(0), probabilities);
    if (arena_used_bytes != nullptr) {
        *arena_used_bytes = s_debug_interpreter->arena_used_bytes();
    }
    debug_tensors_walk(model, *s_debug_interpreter, tensor_cb, ctx);
    return ESP_OK;
}
