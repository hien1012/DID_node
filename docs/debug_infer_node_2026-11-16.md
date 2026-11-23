# infer_node (ESP32 / TTGO T8) cam 除錯紀錄 2026-11-16

## 1. 換新 cam 模型
- 新模型 `DID_CAM_SEAN/CNN/quantize_out/convert_cppmodel/model_int8.cpp` 覆蓋 `infer_node/main/vision/model.cc`
  (281,392 B；uint8 輸入 [1,64,64,1] scale 0.003875 zp 0，輸出 [1,3] scale 1/256，類別順序不變)。
- 架構改為 SepConv → BN → ReLU，BN 被摺進卷積：**MUL / ADD 消失**（舊模型佔約 54% 推論時間）。
- `Flatten` 在 batch=None 下多出 SHAPE / STRIDED_SLICE / PACK，`cam_model.cc` 的 resolver 改為
  `<10>` 並加 `AddShape` / `AddStridedSlice` / `AddPack`，移除 `AddMul` / `AddAdd`。S3 版同樣處理。
- 推論：不和 FD 重疊時約 **162 ms**（舊模型約 500 ms）。

## 2. 推論時間不固定 (163–300 ms)
- 量測：每次推論記錄 `run`（cam_infer 自己的 CPU 時間）、`preempt`（invoke − run）、
  `fd_overlap`（同期間 Core 1 的 cam_detect 處理幀的時間），每個回報週期印相關係數。
- 結果：`run` 與 `fd_overlap` 的 r ≈ 0.9–0.98。**兩顆核心共用 PSRAM / flash 匯流排**，FD 與 CNN 同時跑時
  CNN 約慢 60–90 ms；被搶占只有約 10 ms，連續多個 ROI 時升到 30–50 ms（result_report 不綁核心、優先權 2）。
- 吃記憶體頻寬的 op 最受影響：第一層 CONV_2D (64×64×16) 約 45–65 ms、FULLY_CONNECTED（權重在 flash）約 35–49 ms。

## 3. 相機掉幀 (Camera: incomplete 10–20%)
- 現象：沒有 CNN 時約 10%（每 300 幀 29–31 幀），有 CNN 時 17–20%；全部是長度不足的短幀，
  用 log 時間戳核對 frames + incomplete = 時間 × 5 fps，沒有其他沒被計到的丟幀（無 FB-OVF / EV-OVF）。
- 試過但無效：`CONFIG_CAMERA_TASK_IRAM`（本地驅動選項，cam_task 的 VSYNC 處理放 IRAM），掉幀不變。
- **原因：`task_usage.c` 的 `uxTaskGetSystemState()`**。IDF 5.1 雙核心下它在 critical section 內
  （關閉呼叫端核心的中斷、拿住 kernel lock）對每個 task 逐 byte 掃描剩餘 stack，約 1–2 ms。
  相機 DMA 只有 2 個半區（各 3 行 ≈ 1.2 ms），中斷被擋住時兩次 EOF 合併成一次 → 少 3 行 → incomplete。
  - cam_detect（Core 1）每 10 幀 `task_usage_cycle_mark()` 一次 → 每 300 幀約 30 次 → 約 10%。
  - 每次推論前後 `task_usage_begin/end()`（Core 0）→ Core 1 的 ISR 等 kernel lock → 每次快照約多丟 0.35 幀。
- 修正：`infer_config.h` 新增 `INFER_TASK_USAGE_ENABLE`（預設 0），關閉時 task_usage 全是空函式。
- 驗證：關閉後 300 幀 incomplete **0**（22 ROI），下一個 300 幀 **1**（61 ROI）；幀間隔 199.90–199.94 ms。

## 4. 後續
- 不再掉幀後，每幀 FD 都會跑（約 124 ms / 200 ms），CNN 幾乎每次都和 FD 重疊，推論穩定在約 232 ms
  （之前的 163 ms 多半是下一幀剛好掉了、FD 沒事做）。
- 要再快：減少 PSRAM 用量（FD 解析度 320×240 的三個緩衝；模型第一層通道數 / FC 前多一層 pool）。
- 若需要 CPU 統計：改成逐 task `vTaskGetInfo(..., pdFALSE, ...)`，不掃 stack。
- `CONFIG_CAMERA_TASK_IRAM` 可保留或移除（未證實有效）。
- 已同步到 `DID_NODE/infer_node`（S3）：模型、resolver、`INFER_TASK_USAGE_ENABLE`、推論量測；S3 未上板驗證。
