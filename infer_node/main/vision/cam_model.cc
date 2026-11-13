/* 改寫自 FD_did_cam_v4/main/main_functions.cc 的模型載入與 CNN 輸出解碼。 */
#include "cam_model.h"

#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "model.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/compatibility.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_profiler_interface.h"
#include "tensorflow/lite/schema/schema_generated.h"

namespace {

constexpr char TAG[] = "CAM_MODEL";
/* 原專案保留 1.39 MB；activation 峰值 131 KB，再加 tensor 結構與 kernel scratch。
 * 開機 log 的 used 是實際用量，可依此再調小。 */
constexpr int kTensorArenaSize = 320 * 1024;

/* 訓練標籤：0=無人機、1=鳥、2=背景 */
constexpr int kDroneClass = 0;
/* 原專案的判定門檻 (偏 recall)：drone 機率超過就判 drone，否則取最大值 */
constexpr float kDroneProbGate = 0.30f;
const char *const kClassNames[CAM_MODEL_CLASS_COUNT] = {"drone", "bird", "background"};

/*
 * TFLM 每執行一層會呼叫一次 Begin / End，順序固定，所以用呼叫順序當層的編號。
 * 每次推論只多幾十次 esp_timer 呼叫，相對於整次推論可以忽略。
 * 不用內建的 MicroProfiler：它會保留 4096 筆事件，要佔掉幾十 KB。
 */
class OpTimer : public tflite::MicroProfilerInterface {
 public:
  void StartInvoke()
  {
    next_op_ = 0;
    ++invokes_;
  }

  uint32_t BeginEvent(const char *tag) override
  {
    const uint32_t op = next_op_++;
    if (op < CAM_MODEL_MAX_OPS) {
      names_[op] = tag;
      started_us_[op] = esp_timer_get_time();
    }
    return op;
  }

  void EndEvent(uint32_t op) override
  {
    if (op < CAM_MODEL_MAX_OPS) {
      total_us_[op] += static_cast<uint32_t>(esp_timer_get_time() - started_us_[op]);
    }
  }

  size_t Take(cam_model_op_time_t *ops, uint32_t *invokes)
  {
    const size_t count = next_op_ < CAM_MODEL_MAX_OPS ? next_op_ : CAM_MODEL_MAX_OPS;
    for (size_t i = 0; i < count; i++) {
      ops[i].name = names_[i];
      ops[i].total_us = total_us_[i];
      total_us_[i] = 0;
    }
    *invokes = invokes_;
    invokes_ = 0;
    return count;
  }

 private:
  const char *names_[CAM_MODEL_MAX_OPS] = {};
  int64_t started_us_[CAM_MODEL_MAX_OPS] = {};
  uint32_t total_us_[CAM_MODEL_MAX_OPS] = {};
  uint32_t next_op_ = 0;
  uint32_t invokes_ = 0;

  TF_LITE_REMOVE_VIRTUAL_DELETE
};

OpTimer s_op_timer;
uint32_t s_last_invoke_us = 0;
uint8_t *s_tensor_arena = nullptr;
tflite::MicroInterpreter *s_interpreter = nullptr;
TfLiteTensor *s_input = nullptr;
TfLiteTensor *s_output = nullptr;

}  // namespace

extern "C" const char *cam_model_class_name(int cls)
{
    return (cls >= 0 && cls < CAM_MODEL_CLASS_COUNT) ? kClassNames[cls] : "unknown";
}

extern "C" esp_err_t cam_model_init(void)
{
    if (s_interpreter != nullptr) {
        return ESP_OK;
    }

    const tflite::Model *model = tflite::GetModel(g_model);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema %d != supported %d", static_cast<int>(model->version()),
                 TFLITE_SCHEMA_VERSION);
        return ESP_FAIL;
    }

    s_tensor_arena = static_cast<uint8_t *>(heap_caps_aligned_alloc(
        16, kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (s_tensor_arena == nullptr) {
        ESP_LOGE(TAG, "Tensor arena allocation failed: %d bytes", kTensorArenaSize);
        return ESP_ERR_NO_MEM;
    }

    static tflite::MicroMutableOpResolver<9> resolver;
    if (resolver.AddConv2D() != kTfLiteOk || resolver.AddDepthwiseConv2D() != kTfLiteOk ||
        resolver.AddFullyConnected() != kTfLiteOk || resolver.AddSoftmax() != kTfLiteOk ||
        resolver.AddMaxPool2D() != kTfLiteOk || resolver.AddReshape() != kTfLiteOk ||
        resolver.AddQuantize() != kTfLiteOk || resolver.AddMul() != kTfLiteOk ||
        resolver.AddAdd() != kTfLiteOk) {
        ESP_LOGE(TAG, "Op resolver setup failed");
        return ESP_FAIL;
    }

    static tflite::MicroInterpreter interpreter(model, resolver, s_tensor_arena, kTensorArenaSize,
                                                nullptr, &s_op_timer);
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors failed");
        return ESP_FAIL;
    }

    s_input = interpreter.input(0);
    s_output = interpreter.output(0);
    if (s_input->type != kTfLiteUInt8 || s_output->type != kTfLiteUInt8) {
        ESP_LOGE(TAG, "Expected uint8 input/output, got input=%d output=%d",
                 s_input->type, s_output->type);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_input->bytes != CAM_MODEL_ROI_BYTES || s_output->bytes != CAM_MODEL_CLASS_COUNT) {
        ESP_LOGE(TAG, "Unexpected tensor size: input=%u output=%u",
                 static_cast<unsigned>(s_input->bytes), static_cast<unsigned>(s_output->bytes));
        return ESP_ERR_INVALID_SIZE;
    }

    s_interpreter = &interpreter;
    ESP_LOGI(TAG, "Camera model ready: model=%d bytes arena=%d used=%u bytes",
             g_model_len, kTensorArenaSize,
             static_cast<unsigned>(s_interpreter->arena_used_bytes()));
    return ESP_OK;
}

extern "C" esp_err_t cam_model_predict(const uint8_t *roi,
                                       float probabilities[CAM_MODEL_CLASS_COUNT],
                                       int *predicted_class)
{
    if (!roi || !probabilities || !predicted_class) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_interpreter == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    std::memcpy(s_input->data.uint8, roi, CAM_MODEL_ROI_BYTES);
    s_op_timer.StartInvoke();
    s_last_invoke_us = 0;
    const int64_t invoke_start_us = esp_timer_get_time();
    if (s_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke failed");
        return ESP_FAIL;
    }
    s_last_invoke_us = static_cast<uint32_t>(esp_timer_get_time() - invoke_start_us);

    const int zero_point = s_output->params.zero_point;
    const float scale = s_output->params.scale;
    int best = 0;
    for (int i = 0; i < CAM_MODEL_CLASS_COUNT; i++) {
        probabilities[i] =
            (static_cast<float>(s_output->data.uint8[i]) - static_cast<float>(zero_point)) * scale;
        if (probabilities[i] > probabilities[best]) {
            best = i;
        }
    }
    *predicted_class = probabilities[kDroneClass] > kDroneProbGate ? kDroneClass : best;
    return ESP_OK;
}

extern "C" uint32_t cam_model_last_invoke_us(void)
{
    return s_last_invoke_us;
}

extern "C" size_t cam_model_take_op_times(cam_model_op_time_t ops[CAM_MODEL_MAX_OPS],
                                          uint32_t *invokes)
{
    return s_op_timer.Take(ops, invokes);
}
