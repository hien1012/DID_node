# infer_node：麥克風 / 相機各自推論的韌體

獨立的 ESP-IDF 專案，和上層的 did_node（`../main`）平行存在，互不影響。
Wi-Fi、server 位址等設定沿用 `../main/app_config.h`；推論專屬的設定在 `main/infer_config.h`。

**功能在燒錄前選**：同一份程式碼，改 `main/infer_config.h` 的 `INFER_NODE_ROLE` 再 build：
`INFER_ROLE_CAM`、`INFER_ROLE_MIC`（只跑一條，另一條不初始化），或 `INFER_ROLE_BOTH`（相機 + 麥克風同時跑，配置見下方）。
開機 log 的 `Device ID: ..., role: ...` 可確認燒的是哪一種。

## Build / flash

```powershell
. C:\Espressif\frameworks\esp-idf-v5.1.2-2\export.ps1
cd "C:\Users\Potato\ESP_project file\DID_NODE\infer_node"
idf.py -B build build                    # 第一次會下載 esp_jpeg (esp32-camera 的相依)
idf.py -B build -p COM16 flash monitor
```

改 `sdkconfig.defaults` 後要刪掉這個資料夾的 `sdkconfig` 才會套用。

**優化等級**：只切 `sdkconfig.defaults` 的一個選項，兩個 component 的 CMakeLists 依它決定。

| 選項 | 專案（main/、FD、logmel） | tflite-micro | esp-nn |
|---|---|---|---|
| `CONFIG_COMPILER_OPTIMIZATION_PERF=y`（目前） | `-O2` | `-O3`，`depthwise_conv.cc` `-O2`（原版設定） | `-O2` |
| `CONFIG_COMPILER_OPTIMIZATION_DEFAULT=y`（除錯） | `-Og` | `-Og` | `-Og` |

tflite-micro 不能整個用 `-O2`：GCC 12.2 編 `reference/portable_tensor_utils.cc` 會 internal compiler error
（`-O1` / `-O3` / `-Os` 都正常）。esp-nn 的 `.S` 組合語言不受優化等級影響。
改 defaults 後要同步改 `sdkconfig`（或刪掉讓它重新產生）。
要試 ESP-NN：`idf.py -B build -DESP_NN_KERNEL_SET=all build`（預設 `none` = 全部 reference kernel）。

### Cache 實驗（A / B / C / D）

每個版本用自己的 build 目錄與 sdkconfig（`-DSDKCONFIG=<目錄>/sdkconfig`），互不覆蓋，根目錄的 `sdkconfig` 也不動。
片段檔疊在 `sdkconfig.defaults` 上；改了片段要刪掉該目錄的 `sdkconfig` 才會套用。

| 版本 | 內容 | 內部 RAM 代價 |
|---|---|---|
| A | 現況：ICache 16 KB、DCache 32 KB / 32 B line、模型在 flash | — |
| B | DCache 64 KB / 64 B line（`sdkconfig.dcache64`） | −32 KB |
| C | B + ICache 32 KB（`sdkconfig.icache32`） | −48 KB |
| D | B + 兩個模型開機複製到 PSRAM（`-DINFER_MODEL_IN_PSRAM=1`，多用 PSRAM 約 295 KB） | −32 KB |

```powershell
idf.py -B build_cache_a -DSDKCONFIG=build_cache_a/sdkconfig -DESP_NN_KERNEL_SET=all build
idf.py -B build_cache_b -DSDKCONFIG=build_cache_b/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.dcache64" -DESP_NN_KERNEL_SET=all build
idf.py -B build_cache_c -DSDKCONFIG=build_cache_c/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.dcache64;sdkconfig.icache32" -DESP_NN_KERNEL_SET=all build
idf.py -B build_cache_d -DSDKCONFIG=build_cache_d/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.dcache64" -DINFER_MODEL_IN_PSRAM=1 -DESP_NN_KERNEL_SET=all build
idf.py -B build_cache_b -p COM16 flash monitor    # 燒哪個版本就用哪個目錄
```

開機 log 自己標明版本：`INFER_APP: Cache: instruction .. KB, data .. KB with ..-byte lines`，
`CAM_MODEL` / `AUDIO_MODEL` 的 ready log 寫 `(flash)` 或 `(PSRAM)`。

要比的數字（同一支影片、同樣次數，與 FD/CNN 拖慢實驗相同做法）：cam_infer 的 `invoke / run / preempt / fd_overlap`
（看 run 對 fd_overlap 的斜率有沒有變小）、`Detect stage avg/max ms`、`MIC_INFER logmel= cnn=`、`CNN op avg us`（FC 層看 D），
以及 `HEAP [running]` 與 heartbeat 的 `heap_min_kb`（C 只剩約 43 KB，上傳時的最低值要確認）。

Server：在上層執行 `python server_mix.py`，瀏覽器開 `http://localhost:5000/`。
沒有板子也能看畫面：另開一個終端機跑 `python tools/infer_fake_node.py`（資料都是假的）。

## 運作方式

相機節點：擷取、FD、追蹤在 Core 1，CNN 在 Core 0。擷取有即時性，要避開 Core 0 的 Wi-Fi；
FD 每一幀都要處理，不能被 CNN 擋住；CNN 晚一點沒關係。
麥克風節點：擷取 + logmel 在 Core 0，CNN + 寫 SD 在 Core 1。擷取每 64 ms 只讀一次 I2S、DMA 有 256 ms 緩衝，
優先權 7 高於 Core 0 上 Wi-Fi 以外的 task；logmel 和 CNN 分在兩顆核心，下一個視窗的 logmel 可以和上一個的 CNN 重疊。

```
麥克風節點（連續）
  mic_capture   Core 0 prio 7  讀 I2S → (raw>>16)×30 飽和 → 填 2 s 視窗，滿了換下一塊（共 3 塊）
  audio_feature Core 0 prio 3  logmel → 特徵寫進該視窗專屬的緩衝，交給 audio_cnn
  audio_cnn     Core 1 prio 3  CNN → 結果馬上回傳；低信賴的視窗交給 audio_writer
  audio_writer  Core 1 prio 2  寫 WAV 到 SD，寫完才還視窗

相機節點（連續）
  cam_task       Core 1 prio 23  DMA → 取 Y → PSRAM fb（驅動）
  camera_capture Core 1 prio 6   取 fb 指標、檢查時戳
  cam_detect     Core 1 prio 5   下採樣 ×2 → FD → 追蹤；沒有候選就還 fb，有候選就交給 cam_infer
  cam_infer      Core 0 prio 1   crop 64×64 → CNN → 結果馬上回傳；沒有低信賴 ROI 就還 fb
  cam_writer     Core 0 prio 5   低信賴的幀：全幀 + ROI + meta 寫 SD，寫完才還 fb

兩種節點都有
  result_report  不綁核心 prio 2   佇列內的記錄批次 POST /infer/results；server 斷線只卡住它
  heartbeat      不綁核心 prio 4   每分鐘 /heartbeat，兼 Wi-Fi watchdog
  main           Core 0 prio 1    滿 20 個樣本就上傳，定期校時
```

**相機 + 麥克風（`INFER_ROLE_BOTH`）**：相機的 task 完全照上表；麥克風改成擷取跟 FD 同核心、推論跟影像 CNN 同核心。

```
Core 1  cam_task 23 > mic_capture 7 > camera_capture 6 > cam_detect 5
Core 0  wifi 23 > cam_writer 5 > audio_infer 3 > audio_writer 2 > cam_infer 1 = main 1
```

- `audio_infer` = logmel + CNN 依序在同一個 task（兩段同核心，分兩個 task 不會重疊，只多一個 stack），每 2 s 約 230 ms。
- 音訊優先於影像 CNN：音訊有期限（有樣本在寫 SD 時 2 s），影像 CNN 晚一點只會讓候選重送；
  代價是 `cam_infer` 每 2 s 被搶占一次約 230 ms，`preempt` 會反映。
- I2S DMA 每 32 ms 一次 EOF 中斷，中斷留在 Core 0（init 在 app_main），喚醒 Core 1 的 `mic_capture` 要跨核心；
  DMA 8 塊 × 32 ms，最多 224 ms 不讀不會被覆蓋。`mic_capture` 每 64 ms 讀一次、約 0.3 ms。
- `INFER_TASK_USAGE_ENABLE` 不能和 BOTH 一起開（統計只支援一條管線）；SD 重播只重播相機。

候選多的時候 `cam_infer` 會一直有工作做，所以優先權不能高於綁在同一核心的 `main`，否則上傳會停擺；
每個工作結束再讓出 1 tick，IDLE0 才餵得到看門狗。

**fb 怎麼分**：8 個 fb = DMA 收 1 + 備用 1 + `cam_detect` 處理中 1 + 等偵測 2 + `frame_job_t` 3。
工作在等 CNN、CNN 中、等寫 SD 時都佔一個 fb，因為低信賴時要存整幀，CNN 做完才知道要不要存。

**跟不上時看哪個數字**：`dropped` = FD 跟不上（等偵測的佇列滿）；`rois_expired` = CNN 跟不上
（候選沒等到空的工作，track 就消失）。工作用完時候選留在追蹤器，該 track 下一次配對成功再送，
用的是當時最新的框，所以 CNN 忙不會丟幀。兩者都是累計值。

開機流程、感測器設定、100 幀暖機、10 秒 FPS 測試都和 did_node 相同，沒有按鈕。

**影像格式**：相機輸入永遠是感測器直出的 GRAY8 640×480。FD 用 fb 直接下採樣 ×2，
CNN 的 ROI 直接從 fb 裁切，整條推論路徑沒有 JPEG。

**JPEG 開關**：`main/infer_config.h` 的 `INFER_CAM_SAMPLE_JPEG`（預設 0）只決定低信賴樣本的全幀
存成什麼格式。設成 1 時由 fb 編碼成 JPEG（品質 `INFER_CAM_JPEG_QUALITY`）再存，檔案小很多，
但影像會失真、不再是推論看到的原始資料，cam_writer 也會多花編碼時間與約 14 KB 內部 RAM。
ROI 一律存 raw。改完要重新 build；SD 上已存的樣本兩種格式都能上傳。

**音訊**：第一個 2 s 只收資料，之後每一輪都是「收這一個、推論上一個」。3 塊視窗緩衝 =
收 1 + 推論 1 + 寫 SD 1，所以只要 logmel + CNN 在 2 s 內做完就跟得上；SD 偶爾慢一次，
還有一整個視窗的餘裕。3 塊都在忙時新資料會被丟掉並計數（`dropped`）。
樣本 meta 的 `save_wait_us` = 交給 writer 到開始寫，`write_us` = 寫 WAV 的時間。

**影像**：暖機後不停擷取，追蹤器一直延續。FD 需要前一幀，所以開機後第 1 幀不推論。
一條 track 配對到第 3 次（`age_threshold`，中間最多可連續漏 2 幀）才送 CNN，每條 track 只送一次。
丟幀不算錯誤，也不會像 did_node 那樣刪掉整段。每 300 幀回報一次累計統計。

**XCLK 一直開著**：相機匯流排會干擾 Wi-Fi（did_node 量過 TCP 從約 3.8 MB/s 掉到 0.4 MB/s，
`docs/debug_sdcard.md` #19），樣本上傳和相機同時進行時會比較慢。

**低信賴度**：預測類別的機率 < `INFER_LOW_CONF_THRESHOLD`（0.5）。影像的預測規則沿用
原專案：drone 機率 > 0.30 就判 drone，否則取最大值；信賴度 = 該類別的機率，所以
「drone 0.35」也算低信賴。一幀只要有一個 ROI 低信賴，整幀就存一個樣本。

**非背景也存**（`INFER_SAVE_NON_BACKGROUND`，預設 1，與 ESP32 版相同）：預測不是 `background` 的
（audio：drone / helicopter；video：drone / bird）不論信賴度都存，video 一幀有一個非背景 ROI 就存整幀。
背景類別開機時依名稱從模型類別表找出（audio 是 0、video 是 2），開機 log 印 `Saving ...`。
meta 的 `save_reason` = `low_conf` / `non_background` / `both`。統計的 `low` 只算低信賴，
所以開啟後 `saved` 會大於 `low`。偵測多時樣本量大增，待上傳達 `INFER_MAX_PENDING_SAMPLES` 就不再存。

**上傳**：滿 `INFER_UPLOAD_AFTER_SAMPLES`（20）才傳，ACK 驗證後才刪。
上傳時擷取和推論照常進行，所以 32 KB I/O 緩衝切成兩半：cam_writer 和上傳各 16 KB。
麥克風節點的上傳挑空檔：每次讀 SD（掃目錄、meta、每 16 KB 資料、刪檔）前呼叫 `mic_infer_wait_gap()`，
要等收滿的視窗都推論完、寫完 WAV，且離下一個視窗收滿還有 300 ms 以上才繼續，避免和寫 WAV 搶 SD、
和 logmel 搶 Core 0。等超過 4 s 還沒有空檔就照樣送，上傳不會停擺。
`Upload audio: ... gap_wait= ms gap_timeouts=` 是這次上傳等空檔的總時間與逾時次數。相機連續擷取，不等。

**模型**：兩個都是 int8。音訊是全 int8（輸入輸出都是 int8）。影像的 `model_int8.tflite`
內部是 int8，輸入輸出是 uint8（模型自帶 QUANTIZE），所以韌體直接放灰階值。

**S3 加速**：預設沒開。TFLM 用 reference kernel（`ESP_NN_KERNEL_SET=none`），FD 和 logmel 是
純 C。實際用到的硬體只有浮點單元、LCD_CAM + GDMA、Octal PSRAM、雙核心。ESP-NN（S3 的 SIMD）
要用 `-DESP_NN_KERNEL_SET=all` 打開，開之前後要比對輸出是否合理：原專案在 ESP32 上開了會輸出均分。

**逐層比對 (debug dump)**：一個 ROI / 音訊視窗的模型輸入與 CNN 每一層經 console 輸出，PC 端與 TFLite 逐層比對
(`debug_dump.c` 共用格式，見 `docs/debug_2026_11_17.md` 第 4、5 項)：

```
# 即時 dump：音訊 (麥克風角色)
idf.py -B build_debug -DINFER_AUDIO_DEBUG_DUMP=1 -DESP_NN_KERNEL_SET=all build flash
python ..\..\TTGO_I2S_Record_Test\tools\receive_infer_node_dump.py --port COM14 --reset
# 即時 dump：相機 (相機角色；要有 FD 候選才觸發)
idf.py -B build_cam_debug -DINFER_CAM_DEBUG_DUMP=1 -DESP_NN_KERNEL_SET=all build flash
python ..\..\TTGO_I2S_Record_Test\tools\receive_infer_node_dump.py --port COM14 --reset -o cam_dump
# 比對 (相機、音訊都用這支；TensorFlow 在沒有 export.ps1 的 Python)
& C:\Espressif\tools\idf-python\3.11.2\python.exe ..\tools\compare_debug_dumps.py --dump-dir cam_dump --png
```

**SD 重播四方比對** (`INFER_DEBUG_REPLAY=1`，`debug_replay.c`，見 `docs/debug_2026_11_17.md` 第 6 項)：
同一組固定輸入分別在 ESP-NN 與 TFLM reference 兩個 build 上推論，PC 端並排比對
node ESP-NN / node reference / PC C++ reference / PC Python int8.tflite (音訊另含 Python logmel-CMVN 前處理)：

```
# 1. 產生輸入檔，複製到 SD 卡的 debug/in/ (相機 cam_<名稱>.gray，麥克風 mic_<名稱>.wav)
python ..\tools\make_debug_inputs.py --image drone.png --wav heli.wav --from-dump cam_dump -o debug_in
# 2. 兩種 kernel 各燒一次、各收一次 (N = 輸入檔數，開機 log 的 "Replaying N input(s)")
idf.py -B build_replay_nn  -DINFER_DEBUG_REPLAY=1 -DESP_NN_KERNEL_SET=all  build flash
python ..\..\TTGO_I2S_Record_Test\tools\receive_infer_node_dump.py --port COM14 --reset --count N -o replay_dump
idf.py -B build_replay_ref -DINFER_DEBUG_REPLAY=1 -DESP_NN_KERNEL_SET=none build flash
python ..\..\TTGO_I2S_Record_Test\tools\receive_infer_node_dump.py --port COM14 --reset --count N -o replay_dump
# 3. 比對
& C:\Espressif\tools\idf-python\3.11.2\python.exe ..\tools\compare_debug_dumps.py --dump-dir replay_dump
```

## 回傳給 server 的資料

`POST /infer/results`（JSON）：

| `t` | 內容 |
|---|---|
| `audio` | 視窗序號、開始 / 結束時間、等待 / logmel / CNN 耗時、overflow、預測、機率 |
| `roi` | session、幀序號、時間、track、bbox、CNN 耗時、`result_us`（幀開始 VSYNC → 這個 ROI 的 CNN 做完）、預測、機率 |
| `frame` | session、幀序號、時間、每階段耗時（見下表）、框數、候選數 |
| `session` | 每 300 幀一次的累計統計：幀數、丟幀、斷層、驅動丟棄幀、ROI 數、過期候選數、低信賴數、存檔數、幀間隔 min/max、逾時次數（`error` 欄位）、等待寫 SD 與寫 SD 的平均 / 最長 |
| `cnn_ops` | 每分鐘一次（期間有推論才送）：CNN 每一層的平均耗時（依執行順序） |
| `task_usage` | 相機每分鐘、麥克風每 30 個視窗一次：推論一次的期間，每個 task 平均用掉的 CPU 時間 |

**`task_usage` 怎麼算**（`main/task_usage.c`）：推論前後各抓一次 `uxTaskGetSystemState()`，把每個 task 的
`ulRunTimeCounter` 相減後取平均。相機的「一次」是一個 64×64 ROI 的 CNN，麥克風是 logmel + CNN。
占比以單一核心 100% 計算，兩顆核心加總應接近 200%。計數器只在 task 切換時更新，所以單次有誤差，要看多次的平均；
中斷沒有獨立計時，時間會算在被打斷的 task 上。

每幀計時欄位（都是實際經過時間，包含被其他 task 和中斷插隊的時間）：

| 欄位 | 範圍 |
|---|---|
| `capture_us` | 幀開始 VSYNC → `camera_capture` 送進佇列（含感測器讀出） |
| `queue_us` | 送進佇列 → `cam_detect` 取出 |
| `downscale_us` | 下採樣 ×2 |
| `fd_us` | 幀差偵測合計，含 `fd_diff_us`（幀差 + 二值化）、`fd_morph_us`、`fd_contour_us`、`fd_refine_us`、`fd_nms_us`；中途沒有框時後面是 0 |
| `track_us` | 追蹤器 |
| `detect_us` | `cam_detect` 處理這幀的時間（以上三段），超過 200 ms 就會開始丟幀 |
| `cnn_wait_us` | `cam_detect` 交出 → `cam_infer` 開始（只有含候選的幀） |
| `roi_us` | 所有 ROI 合計，含 `crop_us`（裁切縮放）、`cnn_us`（CNN）和回報（序列埠 log） |
| `total_us` | `detect_us` + `roi_us`（不含等待）；兩段在不同核心，不必低於 200 ms |

含候選的幀要等 CNN 做完才回報 `frame` 記錄，所以到達順序不等於幀順序。

序列埠每 300 幀印 `Detect stage avg/max ms`，每分鐘印 `CNN stage avg/max ms`、`CNN op avg us` 和 `Task CPU per inference` 表。

**整輪統計（只印 log）**：`task_usage` 只量推論那一段（麥克風約 0.24 s、相機一個 ROI 約 0.24 s），
看不到推論以外的 FD、寫 SD、上傳、回報。所以另外連續量「一輪 = 2 s」：

| 節點 | 一輪 | 標記 | 印表 |
|---|---|---|---|
| 麥克風 | 這個視窗到下一個視窗 | `audio_feature` 收到視窗時 | 每 30 個視窗，`Task CPU per audio cycle` |
| 相機 | 每 10 幀（5 fps = 2 s） | `cam_detect` 每 10 幀（只抓 snapshot，不印） | `cam_infer` 每分鐘，`Task CPU per video cycle`（印 log 不佔 FD 的 200 ms） |

兩顆核心加總約 200%，最後一行 `busy: core 0 / core 1` = 100% − 該核心 IDLE 的占比，就是一輪的忙碌程度。
相機的表包含 `cam_infer` 自己印表的時間（Core 0、優先權最低）。
中斷時間算在被打斷的 task 上；`sum` 少個 1–2% 是快照當下另一核心正在跑的 task 還沒更新計數器。

**判斷延遲**（儀表板「最近的判斷」、即時卡片、樣本卡片；`server_mix.py` 的 `result_latency()`）分兩段：

| 段 | 影像 | 音訊 | 時鐘 |
|---|---|---|---|
| 節點判斷 | `result_us`（從該幀 VSYNC 起） | `queue_us + feature_us + invoke_us`（從 2 s 視窗結束起） | 節點同一個時鐘，準 |
| 傳到 server | server 收到該筆記錄的時間 − (起點 + 節點判斷) | 同左 | 跨時鐘，含校時誤差（可能 < 0） |

樣本卡片另列「樣本回傳：拍攝後多久」，那是樣本檔累積滿 20 個才批次上傳的時間，不是判斷延遲。
樣本靠 `session + frame + track`（音訊 `start_ms + window`）對到結果記錄；舊韌體的 ROI 沒有 `result_us`，只顯示合計。

`POST /infer/sample`：body = meta JSON（`X-Meta-Length` bytes）+ 原始資料（`X-File-Size` bytes）。

- 音訊：`.wav`，就是推論用的那份 PCM。
- 影像：全幀（預設 raw 307,200 B；開 JPEG 時為 `image_bytes` 長度的 JPEG）+ 每個 ROI 4,096 B。
  meta 記錄格式、切法與每個 ROI 的結果。SD 上一律叫 `.raw`；server 依格式存成 `.raw` 或 `.jpg`。

SD 上放在 `/sdcard/samples/{audio,video}/`，server 拒收的移到 `/sdcard/samples/rejected/`。

## 來源

| 檔案 | 來自 | 改動 |
|---|---|---|
| `main/audio/model_data_int8.*`、`logmel_features_int8.*`、`preprocess_config_int8.h` | TTGO_I2S_Record_Test/main（int8 版） | 原樣；三者是一組，要一起換 |
| `main/audio/audio_model.cc` | 同上 `model_runner_int8.cc` | 拿掉 memory_monitor |
| `main/vision/model.*` | FD_did_cam_v4/main | 原樣 |
| `main/vision/fd_mix.*` | 同上 | 只留相機管線用的函式；配置全改 PSRAM |
| `main/vision/stage2.*` | 同上 | `g_pairs`（51 KB）從 .bss 改 PSRAM；緩衝滿時候選留到下一次配對，並計數過期的候選 |
| `main/vision/cam_model.cc` | 同上 `main_functions.cc` | arena 從 1.39 MB 改 320 KB |
| `main/cam_infer.c` | `../main/cam_record.c` | 感測器與暖機原樣；capture + write 換成 capture + detect + infer |
| `components/tflite-micro`、`esp-nn` | TTGO_I2S_Record_Test/managed_components（1.3.7 / 1.2.3） | CMake 加 `ESP_NN_KERNEL_SET`；原本寫死的 `-O3` / `-O2` 改成只在專案 `-O2` 時套用，`-Og` 時跟專案 |

## 上板要確認的 log

- `AUDIO_MODEL` / `CAM_MODEL` 的 `arena ... used=`：實際用量，可再縮小 arena。
- `HEAP [...]`：內部 RAM 的 free / min。
- `MIC_INFER` 的 `logmel= ms cnn= ms handoff= ms`（handoff = logmel 結束到 Core 1 開始 CNN）、`Camera: ... dropped= expired=`：實際耗時與是否跟得上。
- `All 3 windows busy`：麥克風推論或寫 SD 來不及，丟了資料。
- `Upload audio: ... gap_wait= gap_timeouts=`：上傳等空檔花多久；`gap_timeouts` 不是 0 代表推論幾乎沒有空檔。
- `Stack free: cam_detect= cam_infer=`：stack 可否縮小。
