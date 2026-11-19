#pragma once

/*
 * 相機除錯 dump (infer_config.h 的 INFER_CAM_DEBUG_DUMP=1)：
 * 正常推論流程照常運作；cam_infer 對開機後前 INFER_CAM_DEBUG_DUMP_ROIS 個 ROI 做完正式推論後，
 * 呼叫 cam_debug_dump_roi()，把
 *   - 正式路徑實際送進 CNN 的 64x64 uint8 ROI (roi.gray) 與正式推論結果
 *   - 正式 interpreter 對同一個 ROI 再推論一次的結果 (檢查同一輸入是否每次相同)
 *   - 保留所有張量的 interpreter 對同一個 ROI 推論的每一層輸出
 * 以 base64 文字行從 console (UART0) 輸出，不寫 SD (格式與音訊 dump 相同，見 debug_dump.c)。PC 端：
 *   TTGO_I2S_Record_Test/tools/receive_infer_node_dump.py -o cam_dump  接收、驗證 CRC、還原成檔案
 *   DID_NODE/tools/compare_debug_dumps.py --dump-dir cam_dump         逐層比對
 * SD 重播 (debug_replay.c) 也走這裡，replay_name 不是 NULL。
 */
#include <stdint.h>

#include "esp_err.h"

#include "cam_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 正式推論路徑的一個 ROI；所有指標在呼叫期間必須有效。 */
typedef struct {
    const uint8_t *roi;                 /* CAM_MODEL_ROI_BYTES，正式 CNN 的輸入 */
    const float *probs;                 /* 正式 CNN 的機率 (CAM_MODEL_CLASS_COUNT) */
    int pred;                           /* 正式判定 (含 drone 門檻) */
    uint32_t index;                     /* 開機後第幾個被 dump 的 ROI */
    uint32_t frame;
    int32_t track_id;
    const int32_t *bbox;                /* x1, y1, x2, y2 (640x480 座標，已外擴並補成正方形) */
    int64_t ts_epoch_ms;
    uint32_t invoke_us;                 /* 正式 Invoke() 本身的時間 */
    /*
     * SD 重播 (debug_replay.c) 的輸入檔名，不含副檔名，例如 "cam_drone01"；NULL = 即時擷取。
     * 重播時 dump 名稱是 <replay_name>_<ESP_NN_KERNEL_SET>，PC 端靠它配對兩種 kernel 的 dump。
     */
    const char *replay_name;
} cam_debug_roi_t;

/**
 * 在呼叫端 task (cam_infer) 裡同步輸出 (115200 baud 約 20 s)；期間 CNN 停住，
 * 候選留在追蹤器 (FD 與取幀照常)，之後恢復正常。自己配置所需的 PSRAM (第一次呼叫時)。
 */
esp_err_t cam_debug_dump_roi(const cam_debug_roi_t *roi);

#ifdef __cplusplus
}
#endif
