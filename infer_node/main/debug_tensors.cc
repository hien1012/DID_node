/* 除錯 dump 的張量列舉 (原本在 audio_model.cc 的 audio_model_debug_run)，說明見 debug_tensors.h。 */
#include "debug_tensors.h"

#include "tensorflow/lite/c/common.h"

namespace {

size_t tensor_type_size(tflite::TensorType type)
{
    switch (type) {
    case tflite::TensorType_INT8:
    case tflite::TensorType_UINT8:
    case tflite::TensorType_BOOL:
        return 1;
    case tflite::TensorType_INT16:
    case tflite::TensorType_FLOAT16:
        return 2;
    case tflite::TensorType_INT32:
    case tflite::TensorType_FLOAT32:
        return 4;
    case tflite::TensorType_INT64:
        return 8;
    default:
        return 0;
    }
}

}  // namespace

void debug_tensors_walk(const tflite::Model *model, tflite::MicroInterpreter &interpreter,
                        debug_tensor_cb_t cb, void *ctx)
{
    if (model == nullptr || cb == nullptr) {
        return;
    }
    /* 張量索引 = flatbuffer subgraph 0 的索引，與 PC 端 tf.lite.Interpreter 的索引相同 */
    const auto *tensors = model->subgraphs()->Get(0)->tensors();
    const auto *buffers = model->buffers();
    for (size_t i = 0; i < tensors->size(); ++i) {
        const tflite::Tensor *tensor = tensors->Get(i);
        const tflite::Buffer *buffer =
            buffers != nullptr && tensor->buffer() < buffers->size() ? buffers->Get(tensor->buffer()) : nullptr;

        debug_tensor_info_t info = {};
        info.index = static_cast<int>(i);
        info.name = tensor->name() != nullptr ? tensor->name()->c_str() : "";
        info.type_name = tflite::EnumNameTensorType(tensor->type());
        info.is_const = buffer != nullptr && buffer->data() != nullptr && buffer->data()->size() > 0;
        size_t elements = 1;
        if (tensor->shape() != nullptr) {
            info.dims_count = static_cast<int>(tensor->shape()->size());
            for (int d = 0; d < info.dims_count && d < DEBUG_TENSOR_MAX_DIMS; ++d) {
                info.dims[d] = tensor->shape()->Get(d);
                elements *= static_cast<size_t>(info.dims[d]);
            }
        }
        const tflite::QuantizationParameters *q = tensor->quantization();
        if (q != nullptr && q->scale() != nullptr && q->scale()->size() > 0) {
            info.scale = q->scale()->Get(0);
            info.zero_point = q->zero_point() != nullptr && q->zero_point()->size() > 0
                                  ? static_cast<int>(q->zero_point()->Get(0)) : 0;
            info.quantized = true;
        }
        info.bytes = elements * tensor_type_size(tensor->type());
        TfLiteEvalTensor *eval = interpreter.GetTensor(static_cast<int>(i));
        info.data = eval != nullptr ? eval->data.raw : nullptr;
        cb(&info, ctx);
    }
}
