# infer_node（ESP32 / TTGO T8）：麥克風 / 相機各自推論的韌體

由 `DID_NODE/infer_node`（ESP32-S3）移植到 TTGO T8（ESP32-WROVER，4 MB flash、PSRAM 映射 4 MB）。
獨立的 ESP-IDF 專案，和錄影韌體（`../main`，`FIELD_FIRMWARE=audio|camera`）平行存在、互不影響。
Wi-Fi、server 位址沿用 `../main/app_config.h`；推論專屬設定在 `main/infer_config.h`，腳位在 `main/board_pins.h`。

**一塊板子只跑一個功能**：燒錄前改 `main/infer_config.h` 的 `INFER_NODE_ROLE`
（`INFER_ROLE_CAM` 或 `INFER_ROLE_MIC`）。麥克風種類改 `main/board_pins.h` 的 `MIC_INTERFACE_I2S`。
開機 log 的 `Device ID: e32cam-...` / `e32mic-...` 可確認燒的是哪一種。

## Build / flash

```powershell
. C:\Espressif\frameworks\esp-idf-v5.1.2-2\export.ps1
cd "C:\Users\Potato\ESP_project file\ESP32_cam_mic\infer_node"
idf.py -B build build                    # 第一次會下載 esp_jpeg (esp32-camera 的相依)
idf.py -B build -p COMxx flash monitor
```

改 `sdkconfig.defaults` 後要刪掉這個資料夾的 `sdkconfig` 才會套用。
ESP-NN 預設 `auto` = `all`，cam / mic 都開（CMake 輸出 `ESP-NN kernel set: ...`）：
- mic：TTGO_I2S_Record_Test 原本就用 ESP-NN；上板實測 CNN 1433 ms → 278 ms，輸出正常。
- cam：FD_did_cam_v4 曾在 ESP32 上開 ESP-NN 時 CNN 輸出變成均分（見該專案的 `ESP_NN_DISABLE_NOTES.txt`）。
  上板要確認 cam CNN 機率會隨畫面變化；卡住就退回 `none`，或用 `dw` / `conv` / … 逐一找出有問題的 kernel。
- 手動指定 `idf.py -B build -DESP_NN_KERNEL_SET=all|none|dw|conv|fc|arith|pool|softmax build` 會留在 CMake cache，
  之後要用 `-DESP_NN_KERNEL_SET=auto` 才會回到自動。

Server：跑 `DID_NODE/server_mix.py`（有 `/infer/*` 與儀表板），這個 repo 的 `server_mix.py` 沒有推論端點。
`../main/app_config.h` 的 `APP_SERVER_BASE_URL` 要指向跑 server 的那台 PC。

## 運作方式

架構與 S3 版相同（說明見 `DID_NODE/infer_node/README.md`）：

```
麥克風節點（連續）
  mic_capture   Core 0 prio 7  讀 I2S → PCM ×30 飽和 → 填 2 s 視窗，滿了換下一塊（共 3 塊）
  audio_feature Core 0 prio 3  logmel → 特徵交給 audio_cnn
  audio_cnn     Core 1 prio 3  CNN → 結果馬上回傳；低信賴的視窗交給 audio_writer
  audio_writer  Core 1 prio 2  寫 WAV 到 SD，寫完才還視窗

相機節點（連續）
  cam_task       Core 1 驅動    I2S0 DMA → 取 Y → PSRAM fb
  camera_capture Core 1 prio 6  取 fb 指標、檢查時戳
  cam_detect     Core 1 prio 5  下採樣 ×2 → FD → 追蹤；有候選就交給 cam_infer
  cam_infer      Core 0 prio 1  crop 64×64 → CNN → 結果馬上回傳
  cam_writer     Core 0 prio 5  低信賴的幀：全幀 + ROI + meta 寫 SD

兩種節點都有
  result_report  不綁核心 prio 2  批次 POST /infer/results
  heartbeat      不綁核心 prio 4  每分鐘 /heartbeat，兼 Wi-Fi watchdog
  main           Core 0 prio 1   滿 20 個樣本就上傳，定期校時
```

**Task 統計**（`main/task_usage.c`，與 S3 版相同，需要 sdkconfig 的 run time stats）：
- 推論期間：每次推論前後抓 `uxTaskGetSystemState()`，各 task 的 CPU 時間相減取平均，
  回傳 server（`task_usage` 記錄）並印 `Task CPU per inference` 表。相機每分鐘、麥克風每 30 個視窗一次。
- 一輪（2 s，相機 10 幀 / 麥克風一個視窗）：含 FD、寫 SD、上傳與閒置，只印 `Task CPU per video|audio cycle`，
  最後一行 `busy: core 0 / core 1` 是兩顆核心的忙碌度。

## 與 S3 版的差別

| 項目 | S3 (DID_NODE) | ESP32 (這裡) | 原因 |
|---|---|---|---|
| 相機介面 | LCD_CAM | I2S0 平行輸入 + LEDC XCLK | 晶片不同；感測器設定、暖機與 `cam_record.c` 相同 |
| OV2640 XCLK 提示值 | 20 MHz | 12 MHz | 同錄影韌體（超過 10 MHz 影響 ESP32 I2S 採樣模式） |
| 開機 FPS 測試 | 10 s | 5 s | 同錄影韌體 |
| pause 時 XCLK | 開著 | 停止，`cam_infer_start()` 時恢復並等 500 ms | 同錄影韌體；開機到開始推論之間少干擾 Wi-Fi |
| `cam_detect` 跟不上時 | — | 讓出 1 tick | PSRAM 較慢，FD 可能超過 200 ms，避免 IDLE1 看門狗警告 |
| PCM（見下） | PDM 讀 16-bit | 預設 32-bit >> 16 | 與 TTGO_I2S_Record_Test 訓練資料一致 |
| PDM 開機丟棄 | 0.5 s | 7 s | 同 `audio_record.c` 的 `MIC_BOOT_DELAY_MS`（DC 偏移收斂） |
| SD | GPIO 40/38/39 | SDMMC slot 1（14/15/2），40 MHz | 板子不同 |
| Wi-Fi TX buffer | 16 static | 32 static + 64 cache（偏好 PSRAM） | ESP32 內部 RAM 少 |
| LED | WS2812 | 無 | T8 沒有 |
| Device ID | `node-s3-<MAC>` | `e32cam-<MAC>` / `e32mic-<MAC>` | 同一台 server 接多個節點時分辨 |

**PCM 取法**（`infer_config.h` 的 `MIC_PDM_16BIT_HPF`）：
- `0`（預設）：I2S 與 PDM 都讀 32-bit slot，`(int16)(raw32 >> 16) × 30` 飽和，和訓練資料相同。
- `1`：只影響 PDM。改用 commit 8ddab23 的取法：讀 16-bit slot，增益前加一階高通（24 Hz），再 × 30。
  和訓練資料不同，切換後要確認輸出是否合理。

## 來源

| 檔案 | 來自 | 改動 |
|---|---|---|
| `main/audio/model_data_int8.*`、`logmel_features_int8.*`、`preprocess_config_int8.h` | TTGO_I2S_Record_Test/main | 原樣（與 S3 版相同） |
| `main/audio/audio_model.*` | DID_NODE/infer_node（由 `model_runner_int8.cc` 改） | 原樣 |
| `main/vision/model.*` | FD_did_cam_v4/main | 原樣 |
| `main/vision/fd_mix.*`、`stage2.*`、`cam_model.*` | DID_NODE/infer_node（由 FD_did_cam_v4 改） | 原樣；與 FD_did_cam_v4 的差別：分段計時、`expired_candidates`、緩衝改 PSRAM |
| `main/task_usage.*`、`result_report.*`、`sample_store.*`、`sample_upload.*`、`infer_health.*`、`infer_time.h` | DID_NODE/infer_node | 原樣 |
| `main/wifi_manager.*` | DID_NODE/main | 原樣（重連 backoff、watchdog API；legacy 版沒有） |
| `main/infer_app.c`、`infer_config.h`、`mic_infer.c`、`cam_infer.c` | DID_NODE/infer_node | 見上表 |
| `main/node_storage.*`、`board_pins.h` | 新寫 | S3 node_storage 只留掛載、剩餘空間、32 KB I/O 緩衝 |
| `components/tflite-micro`、`esp-nn` | DID_NODE/infer_node/components | 原樣 |
| `../main/camera_registers.c`、`device_identity.c`、`time_sync.c`、`app_config.h`、`camera_pinout.h` | 錄影韌體 | 直接編譯，不複製 |

## 上板要確認的 log

- `HEAP [...]`：內部 RAM 的 free / min（ESP32 比 S3 少很多，最要注意）。
- `AUDIO_MODEL` / `CAM_MODEL` 的 `arena ... used=`：可再縮小 arena。
- `Detect stage avg/max ms`：`detect` 平均超過 200 ms 就會丟幀（`Camera: ... dropped=`）。
- `CNN stage avg/max ms`、`expired=`：CNN 跟不上時候選過期。
- `MIC_INFER` 的 `logmel= ms cnn= ms`：兩者合計要在 2 s 內；`All 3 windows busy` = 來不及。
- `Task CPU per ...` 表與 `busy: core 0 / core 1`。
- `Stack free: cam_detect= cam_infer=`。
