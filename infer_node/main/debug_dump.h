#pragma once

/*
 * 除錯 dump 的共用輸出 (audio/audio_debug.c、vision/cam_debug.c)：把檔案以 base64 文字行
 * 從 console (UART0) 輸出，不寫 SD。格式見 debug_dump.c。PC 端：
 *   TTGO_I2S_Record_Test/tools/receive_infer_node_dump.py  接收、驗證 CRC、還原成 <stem>/ 資料夾
 * 只給除錯模式用，不可重入 (同一時間只有一個 task 在 dump)。
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_TENSOR_MAX_DIMS 6

/** 模型的一個張量；索引 = flatbuffer subgraph 0 的索引，與 PC 端 tf.lite.Interpreter 相同。 */
typedef struct {
    int index;
    const char *name;
    const char *type_name;      /* flatbuffer TensorType，例如 "INT8"、"UINT8"、"INT32" */
    int dims_count;
    int dims[DEBUG_TENSOR_MAX_DIMS];
    bool quantized;
    float scale;
    int zero_point;
    bool is_const;              /* 權重 / bias：資料來自 flatbuffer，不輸出 */
    const void *data;           /* 只在 callback 期間有效 */
    size_t bytes;
} debug_tensor_info_t;

typedef void (*debug_tensor_cb_t)(const debug_tensor_info_t *tensor, void *ctx);

typedef struct {
    int files;
    int errors;
    uint32_t lines;
} debug_dump_stats_t;

/** @@DUMP BEGIN <stem>，並把 stats 歸零 */
void debug_dump_begin(debug_dump_stats_t *stats, const char *stem);

/** 一個檔案 = @@FILE 標頭 + 多行 @@D；data 為 NULL 時記一個錯誤 */
void debug_dump_file(debug_dump_stats_t *stats, const char *name, const void *data, size_t bytes);

/** @@DUMP END <stem> <files> <errors> */
void debug_dump_end(const debug_dump_stats_t *stats, const char *stem);

/*
 * 各層輸出 + nn_index.csv：
 *   debug_dump_index_reset();
 *   xxx_model_debug_run(..., debug_dump_tensor, &stats);   每個非常數張量輸出成 nn_tNNN.bin
 *   debug_dump_index_emit(&stats);                         最後輸出 nn_index.csv
 */
void debug_dump_index_reset(void);
void debug_dump_tensor(const debug_tensor_info_t *tensor, void *ctx);   /* ctx = debug_dump_stats_t * */
void debug_dump_index_emit(debug_dump_stats_t *stats);

#ifdef __cplusplus
}
#endif
