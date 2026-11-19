#pragma once

/*
 * SD 重播除錯 (infer_config.h 的 INFER_DEBUG_REPLAY=1)：
 * 開機後不做即時擷取，改讀 SD 上固定的輸入檔，逐一用正式 interpreter 推論，再交給既有的
 * dump (cam_debug.c / audio_debug.c) 從 console 輸出每一層。同一組輸入分別用
 * ESP_NN_KERNEL_SET=all 與 none 各燒一次、各收一次，PC 端就能在完全相同的輸入上比對
 * node ESP-NN / node TFLM reference / PC reference / PC Python。
 *
 * 輸入檔 (放在 SD 的 debug/in/，檔名不分大小寫，依檔名排序處理，最多 DEBUG_REPLAY_MAX_FILES 個)：
 *   相機節點：cam_<名稱>.gray   64 x 64 uint8 灰階 (row-major)，剛好 4096 bytes
 *   麥克風節點：mic_<名稱>.wav  16 kHz / mono / PCM16，至少 2 s；只用前 2 s (32000 樣本)
 * 另一種角色的檔案會被略過。PC 端 tools/make_debug_inputs.py 可由圖片、WAV 或既有 dump 產生。
 * dump 名稱 = <檔名去掉副檔名>_<ESP_NN_KERNEL_SET>，例如 cam_drone01_all、mic_heli02_none。
 *
 * 重播完不啟動即時推論 (板子停在原地，重新上電或換回一般韌體才恢復)。
 */
#include "esp_err.h"

#define DEBUG_REPLAY_DIR          "/sdcard/debug/in"
#define DEBUG_REPLAY_MAX_FILES    32U

/**
 * 在另一個 task (Core 1) 依序處理所有輸入檔，結束後才返回。SD 必須已掛載，模型必須已初始化
 * (cam_infer_init / mic_infer_init 之後)，而且即時管線尚未啟動。
 * @return ESP_OK = 每個檔案都 dump 成功；ESP_ERR_NOT_FOUND = 沒有可用的輸入檔
 */
esp_err_t debug_replay_run(void);
