/* 改寫自 TTGO_I2S_Record_Test/main/model_runner_int8.cc：拿掉 memory_monitor，其餘相同。 */
#include "audio_model.h"

#include <cmath>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
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
uint8_t *s_tensor_arena = nullptr;
tflite::MicroInterpreter *s_interpreter = nullptr;
TfLiteTensor *s_input = nullptr;
TfLiteTensor *s_output = nullptr;

}  // namespace

extern "C" esp_err_t audio_model_init(void)
{
    if (s_interpreter != nullptr) {
        return ESP_OK;
    }

    const tflite::Model *model = tflite::GetModel(g_audio_model_int8_tflite);
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

    /* Keep synchronized with the model's operators (logmel_conversion_summary_int8.json). */
    static tflite::MicroMutableOpResolver<6> resolver;
    if (resolver.AddAveragePool2D() != kTfLiteOk || resolver.AddConv2D() != kTfLiteOk ||
        resolver.AddDepthwiseConv2D() != kTfLiteOk || resolver.AddMaxPool2D() != kTfLiteOk ||
        resolver.AddMean() != kTfLiteOk || resolver.AddSoftmax() != kTfLiteOk) {
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
    ESP_LOGI(TAG, "Audio int8 model ready: model=%u bytes arena=%d used=%u bytes",
             static_cast<unsigned>(g_audio_model_int8_tflite_len), kTensorArenaSize,
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

    int best = 0;
    for (int i = 0; i < AUDIO_MODEL_CLASS_COUNT; i++) {
        probabilities[i] = (static_cast<int32_t>(s_output->data.int8[i]) -
                            AUDIO_MODEL_OUTPUT_ZERO_POINT) * AUDIO_MODEL_OUTPUT_SCALE;
        if (probabilities[i] > probabilities[best]) {
            best = i;
        }
    }
    *predicted_class = best;
    return ESP_OK;
}

extern "C" uint32_t audio_model_last_invoke_us(void)
{
    return s_last_invoke_us;
}
