#pragma once

/*
 * 除錯 dump：列舉 TFLM interpreter 的每個張量 (audio_model.cc、cam_model.cc 共用，只有 C++)。
 * interpreter 要以 preserve_all_tensors 建立，各層輸出才不會被後面的層覆蓋。
 */
#include "debug_dump.h"

#ifdef __cplusplus
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

/** 對 model subgraph 0 的每個張量 (依索引順序) 呼叫一次 cb；data 指向 interpreter 內的資料。 */
void debug_tensors_walk(const tflite::Model *model, tflite::MicroInterpreter &interpreter,
                        debug_tensor_cb_t cb, void *ctx);
#endif
