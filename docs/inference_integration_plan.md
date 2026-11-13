# 模型整合規劃：2 秒視窗推論（cam + mic）

範圍：把音訊模型（`TTGO_I2S_Record_Test`）與影像模型（`DID_CAM_SEAN/FD_did_cam_v4`）整合進 did_node，
改成「2 s 視窗 → 推論 → 低信賴度的視窗存 SD → 上傳 server」。這份只看**容量（內部 RAM / PSRAM / flash）與 CPU 時間**，
PSRAM / cache 頻寬之後再看。

- 標「估」的數字沒有上板量測；量測值註明來源（LA 量測 = 2026-10-24 `digital.csv`，RAM = 2026-10-19 `S3 節點 RAM 預算.html`）。
- 實際做出來的是 `infer_node/`（一塊板子只跑 cam 或 mic），上板實測記在 §9。
- 互動版（時間軸動畫、記憶體試算）：同內容的網頁 artifact。

---

## 1. 結論

| 項目 | 結果 | 條件 |
|---|---|---|
| PSRAM | **夠**。建議架構 A3-a 約用 5.0–5.6 MB / 8 MB，剩 2.6 MB 以上 | 音訊用 int8 模型；影像 arena 依實測縮小（原專案保留 1.39 MB，實際峰值約 131 KB） |
| 內部 RAM | **夠，但最緊**。新增約 19 KB（兩個推論 task stack + TFLM 靜態物件） | 10/19 量到錄影最低 2.6 KB，之後已回收約 56 KB，推估現在約 59 KB（**要新 log 確認**）；fd_mix.c 的小型 `malloc` 必須改到 PSRAM |
| Flash | **夠**。app 目前 1.15 MB，加 TFLM + ESP-NN + 兩個模型 + FD 約 1.8–2.0 MB（估），factory 分割區 4 MB | — |
| CPU | **夠**。每 2 s 視窗推論約 0.18–0.67 s（ESP-NN int8，估），放 CPU1 → CPU1 約 15–39% | reference kernels（不開 ESP-NN）約 27–65%；推論優先權低於所有擷取 task |
| 先決驗證 | ① ESP-NN 在 S3 上數值是否正確 ② 影像模型換到 OV7725 的 domain shift ③ 音訊 gain ×30 對齊 | 見 §7 |

建議路線：**A3（串流逐幀推論）+ 低信賴視窗留在 PSRAM（A3-a）+ 上傳先批次（U2）+ 推論全放 CPU1（C1）**。
「2s → model → 2s → model」字面上的停–推論–走（A1）最容易做，可以當第一步驗證模型，但覆蓋率只有約 55–75%。

---

## 2. 兩個模型（從原始碼與 .tflite 驗證）

模型張量大小、MAC 數由 `.cc` 內嵌陣列還原 `.tflite` 後用 TensorFlow 解析（`tf.lite.Interpreter`，關掉 XNNPACK delegate）。
activation 峰值 = 單一 op 的輸入 + 輸出同時存活的最大值，是 TFLM arena 的下限；實際 arena 再加 tensor 結構與 kernel scratch。

### 2.1 音訊：logmel CNN（`TTGO_I2S_Record_Test`）

| 項目 | 值 | 來源 |
|---|---|---|
| 目前 build | `app_audio_infer.c` + `model_data.cc`（float32）+ `logmel_features.c` | `main/CMakeLists.txt` |
| 另有 int8 版 | `model_data_int8.cc` + `logmel_features_int8.c`（`CMakeLists_int8.txt`） | — |
| 輸入 | 2 s、16 kHz mono PCM16 = 32,000 samples | `logmel_features.h` |
| PCM 定義 | `(raw32 >> 16) × 30`，飽和到 int16（**gain 在特徵之前**） | `app_audio_infer.c:359-364` |
| I2S 設定 | 32-bit slot、MONO、LEFT、`ws_pol=false`、`bit_shift=true`，與 did_node 相同 | `app_audio_infer.c:266-286`、`mic_record.c:241-258` |
| 特徵 | n_fft 512、hop 256、Hamming、32 mel（HTK）、124 frame、dB、top_db 80、per-mel CMVN | `logmel_features.c` |
| 模型輸入 | `[1,32,124,1]`，float32 15,872 B / int8 3,968 B | `preprocess_config*.h` |
| 輸出 | 3 類：background / drone / helicopter | 同上 |
| 模型大小 | float32 20,724 B、int8 13,432 B | `model_data*.cc` |
| ops | CONV_2D、DEPTHWISE_CONV_2D、MAX_POOL_2D、AVERAGE_POOL_2D、MEAN、SOFTMAX | `logmel_conversion_summary*.json` |
| MAC | 1.68 M / 次 | 解析 .tflite |
| activation 峰值 | float32 317,440 B、int8 79,360 B（conv0 輸出 `[1,32,124,16]`） | 解析 .tflite |
| 原專案 arena | 512 KB，優先 PSRAM | `model_runner.cc:30` |
| 特徵 workspace | 25,160 B（FFT re/im、power、Hamming、twiddle、log_mel） | `logmel_features.c:55-64` |
| 原專案環境 | ESP32（TTGO）、IDF 5.1.2、esp-tflite-micro 1.3.7、esp-nn 1.2.3 | `dependencies.lock` |

### 2.2 影像：FD + 追蹤 + ROI CNN（`FD_did_cam_v4`）

| 項目 | 值 | 來源 |
|---|---|---|
| 相機格式 | `PIXFORMAT_GRAYSCALE` + `FRAMESIZE_VGA` = **640×480 GRAY8，與 did_node 相同** | `main_functions.cc:806-807` |
| Stage1 FD | nearest 下採樣 ×2 → 320×240；`absdiff(prev,curr)` → threshold 30 → 3×3 dilation（水平+垂直）→ flood fill 連通區 → roi_refine → NMS，最多 100 框 | `main_functions.cc:3195-3240`、`fd_mix.c:3043` |
| Stage2 追蹤 | 32 條 track，`age_threshold = 5`：一條 track 連續存在 5 幀才**送一次** CNN（one-shot） | `stage2.h:135`、`main_functions.cc:1004` |
| Stage3 ROI | bbox 外擴 5 px → 補成正方形 → 從**全解析度**影像 crop + resize 到 64×64 | `main_functions.cc:3300-3320` |
| CNN 輸入 | `[1,64,64,1]` uint8（scale 0.00382、zp 0，直接放灰階值） | 解析 .tflite、`main_functions.cc:258-261` |
| 輸出 | 3 類（uint8） | 解析 .tflite |
| 模型大小 | 282,992 B，其中權重 266,993 B（FC 4096×64 就佔 262,144 B） | `model.cc:17` |
| ops | QUANTIZE、DEPTHWISE_CONV_2D、CONV_2D、MUL、ADD、MAX_POOL_2D、RESHAPE、FULLY_CONNECTED、SOFTMAX | 解析 .tflite |
| MAC | 1.09 M / ROI | 解析 .tflite |
| activation 峰值 | 131,072 B（`[1,64,64,16]` 的 MUL/ADD） | 解析 .tflite |
| 原專案 arena | 13×100×1024 + 60 KB = **1,392,640 B**（約為需要的 10 倍） | `main_functions.cc:186-194` |
| ESP-NN | **關掉**（`ESP_NN_KERNEL_SET=none`）：開著時 softmax 輸出幾乎固定 1/3 | `ESP_NN_DISABLE_NOTES.txt` |
| 原專案環境 | ESP32、IDF 5.2.1、esp-tflite-micro 1.3.3、esp-nn 1.1.1、esp32-camera 2.0.15 | `dependencies.lock` |

### 2.3 合併後需要的 op（一個 resolver 或各自一個）

CONV_2D、DEPTHWISE_CONV_2D、FULLY_CONNECTED、MAX_POOL_2D、AVERAGE_POOL_2D、MEAN、SOFTMAX、RESHAPE、QUANTIZE、MUL、ADD
（影像專案另外註冊了 RELU、DEQUANTIZE，模型沒有用到）。兩個原專案的 esp-tflite-micro 版本不同（1.3.7 / 1.3.3），
did_node 只能用一個版本，**影像模型要在新版本重新跑 PC replay 比對**。

---

## 3. 目標流程與名詞

- **視窗**：從同一個 `trigger_us` 起算，第 k 個視窗 = `[trigger + 2k s, trigger + 2(k+1) s)`。
  影像 10 幀（5 fps）= 3,072,000 B；音訊 32,000 samples = 64,000 B（WAV 64,044 B）。
- **判定**：兩個模型對同一視窗的輸出。低信賴度建議定義為「max prob < 門檻」或「top1 − top2 < 門檻」，
  或兩個模型類別不一致；門檻之後依資料調整。另建議固定抽樣少量高信賴視窗上傳，用來監看漂移。
- **保存**：低信賴視窗寫成 `YYYY_MM_DD_HHMMSS_wNN.raw / .wav`（沿用 `undone/ → done/`），附推論結果（metadata）。
- **上傳**：沿用 `node_upload.c`，新增 header（例如 `X-Window-Index`、`X-Infer-Audio`、`X-Infer-Video`、`X-Save-Reason`），
  `server_mix.py` 要同步修改（CLAUDE.md：兩邊要同步）。

---

## 4. 架構方案

### 4.0 A0：現況（對照組）

錄 20 s（正式 60 s）→ 全部寫 SD → 上傳全部。不推論。錄製期間 CPU0 busy 11.4%、CPU1 5.3%；
上傳期間 CPU0 59%、CPU1 30%（LA 量測）。

### 4.1 A1：停–推論–走（字面上的 2s → model → 2s → model）

```
resume(0.3 s 暖機, 丟 2 幀) → 擷取 2 s(10 fb 全部留在 PSRAM) → pause
  → 影像：10 幀 FD + 追蹤 + CNN（批次）→ 音訊：logmel + CNN
  → 低信賴：寫 raw + wav（約 0.9 s）→ 下一輪
```

- 直接沿用 `rec_session` 三段式：`RECORD_DURATION` 改 2 s，`cam_writer` / `mic_writer` 改成判定後才寫。
- PSRAM：fb 要能同時留住 10 幀 + 驅動至少 2 個空 fb → `fb_count = 12`（3.69 MB）。
- 追蹤器每個視窗重來：第一幀只當 prev，只有 9 次 diff；track 要連續 5 幀才送 CNN → **物體要在視窗前 4–5 幀就出現才會被分類**。
- 覆蓋率 = 2 / (0.3 + 2 + T推論 + p×0.92)，約 55–75%（p = 低信賴比例）。
- 優點：沒有並行問題、最容易除錯、兩個 arena 甚至可以輪流用。缺點：有空窗、視窗邊界的事件會被切斷。
- 適合：**第一步**，先驗證兩個模型在 S3 + 我們的 sensor / mic 上的數值與時間。

### 4.2 A2：批次雙緩衝（不建議）

擷取視窗 N+1 的同時批次推論視窗 N。影像 CNN 需要從**全解析度**影像 crop ROI，所以視窗 N 的 10 幀全解析度都要留住，
加上正在擷取的 N+1 → 至少 20+ fb ≈ 6.1 MB，PSRAM 放不下（還要放 arena 與 Wi-Fi）。
只留 320×240 下採樣影像（10 × 76.8 KB）可以做 FD，但 crop 的 ROI 解析度會降一半，跟訓練資料不一致。

### 4.3 A3：串流逐幀推論（建議）

```
cam_task → fb → camera_capture ──► cam_infer (CPU1, prio 4)
                                    每幀：下採樣 + FD + 追蹤（tracker 跨視窗延續）
                                    候選 → crop 64×64 → CNN（同 task 或 cnn task）
                    fb 放進「視窗暫存」直到該視窗判定
mic_capture → block ─► audio_infer (CPU1, prio 3)：湊滿 2 s → gain ×30 → logmel → CNN
視窗結束 + 判定延遲（≈ 音訊推論時間）→ 判定 → 低信賴：交給 cam_writer / mic_writer；否則 fb 還給驅動
```

- 覆蓋率 100%，tracker 跨視窗延續（不會漏掉視窗邊界的物體）。
- 判定延遲 ≈ 視窗結束後的音訊推論時間（影像在視窗內已逐幀做完）。
- fb 的擁有者變成「camera_capture → cam_infer → 視窗暫存 →（cam_writer）→ 驅動」，要做清楚的所有權交接（類似現在的零拷貝 queue）。
- 兩種保存方式：

| | A3-a：視窗留在 PSRAM | A3-b：先寫 SD，判定後刪 |
|---|---|---|
| 做法 | fb 保留到判定；低信賴才交給 cam_writer | 和現在一樣每幀寫 SD；高信賴視窗判定後刪檔 / 截斷 |
| 額外 PSRAM | `fb_count = 12 + ceil(判定延遲 / 200 ms)`，約 13–15 個（4.0–4.6 MB） | fb 8 個不變 |
| SD 寫入量 | p × 135 GB/天（p = 低信賴比例；p = 0.2 → 27 GB/天） | **135 GB/天**（1.57 MB/s 連續），SD 壽命風險 |
| CPU0 | 只有保存時才寫（每視窗 214 ms） | 持續 10.7%（cam_writer + mic_writer） |
| 風險 | 判定太慢 → fb 不夠 → 掉幀 | SD 長期磨耗、FAT 刪檔延遲 |

  建議 **A3-a**。A3-b 只適合短期實驗（例如要收集全部資料時）。

### 4.4 A4：級聯 / 事件觸發（疊在 A3 上的策略）

- 音訊模型與 FD 永遠跑（便宜）；CNN 只在 tracker 有候選時跑（本來就是）。
- 可加規則：音訊「高信賴 background」且 FD 沒有任何框 → 直接判定高信賴，不必等影像。
- 也可以反過來：音訊低信賴 → 該視窗影像強制保存（即使 FD 沒框）。
- 不改記憶體，只降低平均 CPU。

### 4.5 音訊視窗變化（可選）

- 50% 重疊（hop 1 s）：每 1 s 推論一次 2 s 視窗，邊界事件更穩。音訊推論量加倍（仍小），音訊環形緩衝 3 s = 96 KB。

### 4.6 上傳策略

| | 做法 | CPU / 風險 |
|---|---|---|
| U1 即時上傳 | 存好就上傳，與擷取 + 推論並行 | 上傳期間 CPU0 +59%、CPU1 +30%（量測），加上推論 CPU1 可能到 60–90%；upload_reader（CPU1, prio 2）會被推論搶，吞吐量下降 |
| U2 批次（建議起步） | 低信賴視窗累積在 `done/`，排程空檔（例如每 10 分鐘暫停擷取一段）再上傳 | 與現在的「錄完再傳」相同，最安全 |
| U3 先傳摘要 | 先上傳推論結果 + ROI crop（每個 4 KB）與 metadata，原始視窗之後再補 | 需要新 endpoint；server 端可先看到事件 |

### 4.7 核心配置

| | CPU0 | CPU1 | 評估 |
|---|---|---|---|
| C1（建議） | wifi、cam_writer / mic_writer、heartbeat、main（上傳） | cam_task、mic_capture、camera_capture、**cam_infer、audio_infer** | 擷取與推論同核、推論優先權最低；CPU0 留給 Wi-Fi / SD |
| C2 | + audio_infer | + cam_infer | 上傳時 CPU0 已 59%，音訊推論會拖慢上傳 |
| C3 不綁核 | — | — | 不可預期，LA 量測也難解讀 |

優先權（C1）：cam_task 23 > mic_capture 7 > camera_capture 6 > cam_writer / mic_writer 5 > heartbeat 4 ≥ **cam_infer 4 > audio_infer 3** > upload_reader 2 > main 1。

---

## 5. 記憶體預算

### 5.1 內部 RAM（最緊）

| 項目 | 值 | 狀態 |
|---|---|---|
| 10/19 錄影期間最低 | 2,595 B | 量測（`S3 節點 RAM 預算.html`） |
| 之後回收：mic `s_blocks` → PSRAM | +32,832 B | 已在 code（`mic_record.c:158`） |
| 之後回收：`CAMERA_DMA_BUFFER_SIZE_MAX` 32768 → 16384 | +15,360 B | 已在 `sdkconfig.defaults` |
| 之後回收：mic `WRITE_BUFFER_BYTES` 16 → 8 KB | +8,192 B | 已在 code（`mic_record.c:69`） |
| `SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`（10/19 時未生效） | 上傳瞬態改善，未量 | 已在 sdkconfig |
| **推估目前錄影最低** | **≈ 59 KB** | **估，要新的 HEAP log 確認** |
| 新增：cam_infer stack | 8,192 B + TCB | FD + flood fill + TFLM Invoke |
| 新增：audio_infer stack | 8,192 B + TCB | logmel + TFLM Invoke |
| 新增：TFLM 靜態物件（interpreter ×2、resolver） | ≈ 2 KB（估） | .bss |
| **新增合計** | **≈ 19 KB** | |
| 推估剩餘 | ≈ 40 KB | 估 |

**實作時補上（上表沒算到）**：`stage2.c` 的 `g_pairs` 是 32 × 100 × 16 B = 51 KB 的靜態陣列，原本在內部 RAM (.bss)。
`infer_node` 已改成開機配置到 PSRAM。實際 task 配置（`infer_node/README.md`）比 did_node 多約 18 KB 內部 RAM，與上表相符。

**必改**：`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384` 讓 ≤16 KB 的 `malloc` 優先進內部 RAM。
fd_mix.c 每幀都會 `malloc`：flood fill 初始 stack 8 KB（`fd_mix.c` `INIT_CAPACITY 2048 × 4 B`，最壞可長到 307 KB）、
bbox 陣列 2 KB、NMS 陣列等 → 每幀約 10 KB 內部 RAM 瞬態 + 碎片化。整合時改成啟動時用
`heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` 預先配置，固定上限（例如 flood fill stack 上限 64 KB）。

arena、特徵 workspace、ROI pool 都放 PSRAM（TFLM 的 tensor 結構也在 arena 裡，跟著進 PSRAM）。
task stack 放 PSRAM 理論上可行，但會碰到 cache 關閉期間的限制，先放內部。

### 5.2 PSRAM（8 MB）

| 項目 | A1 | A3-a | A3-b | 備註 |
|---|---|---|---|---|
| 相機 fb | 12 × 307,200 = 3,686,400 | 13–15 個：3,993,600–4,608,000 | 8 × 307,200 = 2,457,600 | 現在 8 個 |
| mic `s_blocks` | 33,024 | 33,024 | 33,024 | 現有 |
| Wi-Fi / lwIP（`TRY_ALLOCATE_WIFI_LWIP`） | ≈ 200,000 | ≈ 200,000 | ≈ 200,000 | 估 |
| FD：ds prev / curr / temp | 230,400 | 230,400 | 230,400 | 3 × 320×240 |
| FD：roi_mask 上限 | 76,800 | 76,800 | 76,800 | 預先配置 |
| ROI crop + pool（10 槽） | 45,056 | 45,056 | 45,056 | 可縮到 4 槽 |
| 影像 arena | 196,608 | 196,608 | 196,608 | 峰值 131,072 + 結構 + scratch；**實測 `arena_used_bytes()` 後再縮** |
| 音訊 PCM 視窗 | 64,000 | 128,000（2 個視窗輪替） | 128,000 | |
| 音訊特徵 workspace + 輸入 | 29,128（int8） | 29,128 | 29,128 | float32 輸入 +11,904 |
| 音訊 arena | 98,304（int8） | 98,304 | 98,304 | float32 依原專案 524,288 |
| **合計（int8 音訊）** | **≈ 4.66 MB** | **≈ 5.03–5.64 MB** | **≈ 3.49 MB** | 8 MB 中 |

影像模型權重 283 KB 預設留在 flash（rodata，經 cache 讀）。如果之後量到 FC 層被 flash 讀取拖慢，可複製到 PSRAM（+283 KB）。

### 5.3 Flash

| 項目 | 大小 |
|---|---|
| 目前 app（`build/s3/did_node.bin`） | 1,148,672 B |
| 影像模型 | 282,992 B |
| 音訊模型 | 13,432 B（int8）/ 20,724 B（float32） |
| TFLM + ESP-NN kernels | ≈ 250–400 KB（估） |
| fd_mix + stage2（ESP 用到的部分） | ≈ 40–80 KB（估） |
| **合計** | **≈ 1.8–2.0 MB / factory 4 MB** |

模型之後要常換的話，可以放到獨立的 data 分割區（16 MB flash 只用了約 4 MB），用 `esp_partition_mmap` 讀，不用重 build app。

---

## 6. CPU 時間預估（ESP32-S3 240 MHz，**全部是估計**）

依運算量（MAC、像素操作次數）與 S3 一般效率推算，上板要用 LA 腳位實測（§8）。

| 工作 | 每次 | 每 2 s 視窗 | 推算依據 |
|---|---|---|---|
| 音訊 logmel（float32，手寫 radix-2 FFT） | 40–120 ms | 1 次 | 124 frame × (512 點 FFT ≈ 2.3 k butterfly + 32 mel) |
| 音訊 CNN int8 + ESP-NN | 10–40 ms | 1 次 | 1.68 M MAC × 1.5–6 cycle |
| 音訊 CNN int8 reference | 80–180 ms | 1 次 | × 12–25 cycle |
| 音訊 CNN float32 reference | 150–350 ms | 1 次 | × 20–50 cycle（ESP-NN 不加速 float） |
| 影像 下採樣 + FD + 追蹤 | 13–43 ms / 幀 | 10 次（A1 為 9 次 diff） | 76.8 k 像素 × 約 6 次掃描，buffer 在 PSRAM |
| 影像 CNN + ESP-NN | 8–25 ms / ROI | 0–N 次 | 1.09 M MAC + 2×64 k 元素 MUL/ADD |
| 影像 CNN reference | 60–150 ms / ROI | 0–N 次 | 原專案目前的設定 |
| 保存一個視窗到 SD | ≈ 0.92 s（SD 忙碌）、CPU0 ≈ 214 ms | 低信賴才做 | 3.07 MB ÷ 3.4 MB/s；cam_writer 每幀 20.1 ms（LA） |
| 上傳一個視窗 | ≈ 1.18 s | 低信賴才做 | 31.4 MB / 11.8 s = 2.66 MB/s（LA） |

每 2 s 視窗合計（ROI 0–3 個）：ESP-NN int8 ≈ 0.18–0.67 s（9–33% 單核），reference ≈ 0.43–1.18 s（22–59%）。
加上現有錄製 CPU1 5.3% → CPU1 約 15–39%（ESP-NN）/ 27–65%（reference）。

錄製期間 CPU0 現在 11.4% 幾乎都是 cam_writer（每幀 20.1 ms ≈ 10%）。A1 / A3-a 只在低信賴視窗才寫，CPU0 反而會下降。

---

## 7. 風險與必須先驗證

1. **ESP-NN 數值正確性（最高）**：影像專案在 ESP32 + esp-nn 1.1.1 上開 ESP-NN 時輸出固定成均分。S3 的 ESP-NN 是另一套組合語言實作，
   而且 did_node 要用的 esp-tflite-micro 版本不同 → 必須用影像專案的 `replay_cnn_rois.py` 流程（SD 上的 ROI raw → PC TFLite 比對）重驗。
   reference kernels 也夠用（§6），ESP-NN 是效能選項，不是必要。
2. **影像 domain shift**：影像模型的訓練資料來自影像專案的相機；did_node 用 OV7725（不同 sensor、曝光、雜訊）。
   FD 門檻 30、CNN 都可能要重新校正 / 重新訓練。建議先用 did_node 錄的 raw 在 PC 上跑影像專案的 Python 流程。
3. **音訊對齊**：did_node 的 block 存的是**未乘 gain** 的 PCM（gain 在 mic_writer 寫檔時才乘）；模型輸入要 `×30` 再飽和，和原專案一致。
   原專案在 ESP32（classic）上，S3 的 I2S 資料位置要用一段 WAV 做 Python / C 特徵比對（原專案已有 `compare_logmel_python_c*.py`）。
4. **內部 RAM**：§5.1 的 59 KB 是推估，整合前先抓一次新的 HEAP log；fd_mix.c 的 malloc 改到 PSRAM。
5. **fb 所有權**（A3）：fb 同時被推論、視窗暫存、writer 使用，要明確的交接順序，不能提早 `esp_camera_fb_return()`。
6. **TFLM 不可重入**：每個模型一個 interpreter + arena，只由一個 task 呼叫。
7. **判定延遲 vs fb 數量**（A3-a）：延遲超過預期 → fb 不夠 → 驅動掉幀。要監看 `fb` 最低可用數。
8. **SD 寫入量**：A3-b 每天 135 GB，不適合長期。
9. **上傳協定**：新增欄位，`server_mix.py` 同步修改。

---

## 8. 下一步（量測計畫）

1. 抓新的 HEAP log（錄影期間、上傳期間的 internal min），確認 §5.1。
2. A1 原型：`RECORD_DURATION_SEC = 2`、影像 + 音訊模型批次推論（reference kernels），印 `arena_used_bytes()` 與每段耗時。
3. LA：沿用 `la_probe`，錄製階段把 task 腳改綁 `cam_infer` / `audio_infer`（或新增 phase），量實際推論時間與 CPU1 idle。
4. ESP-NN on/off 對照：同一批 ROI / WAV，比對 PC 輸出與耗時。
5. A3-a：fb 所有權改造、tracker 跨視窗、判定延遲與 fb 低水位監看。
6. 之後再看：PSRAM / cache 頻寬（cam_task 寫 1.54 MB/s、FD 讀、保存時 cam_writer 讀 1.54 MB/s、arena 存取）。

---

## 9. infer_node 實測

### 9.1 音訊（麥克風節點），2026-11-10

**條件**

| 項目 | 設定 |
|---|---|
| 韌體 | `infer_node`，`INFER_NODE_ROLE = INFER_ROLE_MIC`，I2S 麥克風，16 kHz mono，增益 ×30 |
| 優化 | 兩次量測：① 全部 `-Og`（含 tflite-micro、esp-nn）② `-O2`（tflite-micro `-O3`、`depthwise_conv.cc` `-O2`，esp-nn `-O2`）。下面的表是 ①，② 的對照在本節最後 |
| CNN kernel | `ESP_NN_KERNEL_SET=all`（ESP-NN S3 版）；**輸出尚未和 PC TFLite 比對**（§7-1） |
| 管線 | 3 塊 2 s 視窗；`audio_infer`（Core 0, prio 3）logmel + CNN；`audio_writer`（Core 0, prio 2）寫低信賴 WAV；`mic_capture`（Core 1, prio 7） |
| 上傳 | 讀 SD 前 `mic_infer_wait_gap()` 挑空檔。這段量測期間**沒有上傳**（`main` 只有 0.1 ms / 輪，待上傳未滿 20 個） |

**每個視窗的推論時間**（`MIC_INFER` log，w30）：logmel 224 ms + CNN 57 ms = **281 ms**，佔 2 s 視窗的 14%。
§6 的估計是 logmel 40–120 ms、CNN（int8 + ESP-NN）10–40 ms；實測都比估計慢。`-O2` 的對照在本節最後。

**推論期間**（`Task CPU per inference`，30 次平均，每次 281.3 ms；只量 logmel + CNN 那一段）

| task | core | ms | 占比 |
|---|---|---|---|
| audio_infer | 0 | 280.9 | 99.9% |
| IDLE | 1 | 274.6 | 97.6% |
| mic_capture | 1 | 1.3 | 0.4% |
| wifi | 0 | 0.3 | 0.1% |
| esp_timer | 0 | 0.1 | 0.0% |
| sum | | | 198.1% |
| **busy** | | | **core 0 100.0%，core 1 2.4%** |

**一輪 = 一個 2 s 視窗**（`Task CPU per audio cycle`，29 輪平均，每輪 1999.4 ms；從這個視窗到下一個視窗連續量，兩核心所有 task）

| task | core | ms / 輪 | 占比 |
|---|---|---|---|
| IDLE | 1 | 1982.4 | 99.1% |
| IDLE | 0 | 1703.9 | 85.2% |
| audio_infer | 0 | 284.3 | 14.2% |
| mic_capture | 1 | 8.6 | 0.4% |
| result_report | 不綁 | 6.5 | 0.3% |
| tiT | 不綁 | 5.0 | 0.3% |
| wifi | 0 | 4.8 | 0.2% |
| audio_writer | 0 | 2.6 | 0.1% |
| heartbeat | 不綁 | 0.3 | 0.0% |
| sys_evt | 0 | 0.3 | 0.0% |
| esp_timer | 0 | 0.2 | 0.0% |
| main | 0 | 0.1 | 0.0% |
| sum | | | 200.0% |
| **busy** | | | **core 0 14.8%，core 1 0.9%** |

**解讀**

- **一輪錄音的忙碌程度：Core 0 約 15%、Core 1 約 1%。** Core 0 幾乎全是推論（284 ms），其餘 task 合計約 13 ms；
  Core 1 只有 `mic_capture` 每 64 ms 讀一次 I2S（約 0.3 ms / 次）。
- 推論表的 `audio_infer 99.9%` 是「推論那 281 ms 內」的占比，不是整體負載；整體要看一輪的表。
- 每輪約有 1.7 s 空檔，上傳挑空檔時每輪約有 1.4 s 可讀 SD（扣掉 300 ms 餘裕）。
- `audio_writer` 平均 2.6 ms / 輪，代表這 29 輪中有少數低信賴視窗寫了 WAV；寫入在推論之後，不影響推論時間。
- 29 輪而不是 30：第一次 mark 只開始計時，30 個視窗之間是 29 個完整的輪。
- 推論表的 `sum 198.1%`：快照當下 Core 1 正在跑的 IDLE 還沒更新計數器，少算約 2%；一輪的區間長，誤差攤平到 0%。

**`-Og` 與 `-O2` 對照**（同一塊板子、同樣條件；`-O2` 為 w29 / w30 與 30 次 / 29 輪平均，這段期間同樣沒有上傳）

| 項目 | `-Og` | `-O2` | 變化 |
|---|---|---|---|
| logmel | 224 ms | 180–182 ms | −19% |
| CNN（ESP-NN） | 57 ms | 56 ms | −2% |
| 推論期間平均（`Task CPU per inference`） | 281.3 ms | 239.7 ms | −15% |
| 一輪的 `audio_infer` | 284.3 ms | 242.5 ms | −15% |
| 一輪的 `mic_capture` | 8.6 ms | 7.5 ms | |
| 一輪的 `result_report` / `tiT` / `wifi` / `audio_writer` | 6.5 / 5.0 / 4.8 / 2.6 ms | 5.7 / 4.5 / 4.5 / 2.5 ms | 幾乎不變（多半在等 I/O） |
| 一輪的 IDLE（core 0 / core 1） | 85.2% / 99.1% | 87.4% / 99.2% | |
| **一輪 busy** | **core 0 14.8%，core 1 0.9%** | **core 0 12.6%，core 1 0.8%** | |

- `-O2` 省下的幾乎都在 logmel（純 C 浮點）；CNN 主要是 esp-nn 的 `.S` 組合語言，優化等級幾乎沒影響。
- `-O2` 下 logmel 仍有 180 ms，高於 §6 的 40–120 ms。剩下的主要是寫法：去 DC 每個樣本一次 **double** 除法與加法、
  每個視窗重算 **double** cos / sin 表（S3 FPU 只有單精度，double 是軟體模擬，`-O2` 也救不了），以及用 512 點複數 FFT
  算實數訊號。這三項都能改成幾乎同結果的寫法；目前負載 12.6%，只有要做 1 s hop 的重疊視窗（§4.5）時才需要。

**還沒量**

1. 有上傳時的一輪（`main`、`wifi`、`tiT`、`audio_writer` 實際多少，以及 `Upload audio: ... gap_wait= gap_timeouts=`）。
2. ESP-NN 輸出和 PC TFLite 的比對（用存下的 WAV 樣本）。

### 9.2 影像（相機節點），2026-11-10

**條件**

| 項目 | 設定 |
|---|---|
| 韌體 | `infer_node`，`INFER_NODE_ROLE = INFER_ROLE_CAM`，GRAY8 640×480 5 fps，AE/AGC/AWB 暖機後鎖定 |
| 優化 | `-O2`（tflite-micro `-O3`、`depthwise_conv.cc` `-O2`，esp-nn `-O2`） |
| CNN kernel | `ESP_NN_KERNEL_SET=all`；**輸出尚未和 PC TFLite 比對** |
| 管線 | `cam_task` → `camera_capture` → `cam_detect`（Core 1, prio 5：下採樣 + FD + 追蹤）→ `cam_infer`（Core 0, prio 1：crop + CNN）→ `cam_writer`（Core 0, prio 5） |
| 統計 | 推論期間 = 每個 ROI 的 CNN `Invoke`；一輪 = `cam_detect` 每處理 10 幀標記一次（5 fps 下 2 s），`cam_infer` 每分鐘印 |

**忙碌場景**（一分鐘 194 個 ROI ≈ 3.2 個/s，87 幀有候選，平均每幀 2.2 個 ROI）

推論期間（`Task CPU per inference`，194 次平均，每次 279.5 ms）

| task | core | ms | 占比 |
|---|---|---|---|
| cam_infer | 0 | 258.1 | 92.3% |
| cam_detect | 1 | 233.9 | 83.7% |
| IDLE | 1 | 23.9 | 8.6% |
| cam_task | 1 | 15.7 | 5.6% |
| result_report | 不綁 | 10.2 | 3.6% |
| tiT | 不綁 | 8.7 | 3.1% |
| wifi | 0 | 6.2 | 2.2% |
| cam_writer | 0 | 1.9 | 0.7% |
| sum | | | 200.1% |
| **busy** | | | **core 0 100.0%，core 1 91.4%** |

一輪（`Task CPU per video cycle`，27 輪平均，每輪 **2261.8 ms**）

| task | core | ms / 輪 | 占比 |
|---|---|---|---|
| cam_infer | 0 | 1919.4 | 84.9% |
| cam_detect | 1 | 1914.3 | 84.6% |
| IDLE | 1 | 172.5 | 7.6% |
| IDLE | 0 | 156.8 | 6.9% |
| cam_task | 1 | 127.4 | 5.6% |
| result_report | 不綁 | 90.1 | 4.0% |
| tiT | 不綁 | 73.3 | 3.2% |
| wifi | 0 | 51.4 | 2.3% |
| cam_writer | 0 | 14.3 | 0.6% |
| sys_evt / camera_capture / heartbeat / esp_timer / main | | 3.7 / 0.7 / 0.6 / 0.3 / 0.1 | 約 0.2% |
| sum | | | 200.1% |
| **busy** | | | **core 0 93.1%，core 1 92.4%** |

CNN 分層（194 次平均）與安靜時（同日稍早，5 次平均）對照

| 運算 | 安靜時 | 忙碌時 | 變化 |
|---|---|---|---|
| 各層合計 | 234.1 ms | 277.5 ms | +18% |
| 第一層 CONV_2D | 32.7 ms | 42.0 ms | +28% |
| BN 的 ADD + MUL（6 個 op） | 153.1 ms | 180.4 ms | +18% |
| FULLY_CONNECTED | 24.8 ms | 27.7 ms | +11% |
| `CNN stage` | | `cnn_wait` 979 / 2301 ms，`cnn`（每幀合計）620 / 1214 ms（平均 / 最長） | |

**解讀**

- **兩顆核心都約 93% 忙，同時是瓶頸。** Core 0：3.2 ROI/s × 0.277 s ≈ 0.9，CNN 快滿；Core 1：`cam_detect` 每 10 個處理過的幀用
  1914 ms，平均每幀約 191 ms，貼近 200 ms 上限。
- **有丟幀**：一輪應是 2000 ms，實測 2261.8 ms。一輪以「處理過的 10 幀」計，多出的 13% 約等於每 10 幀丟 1.3 幀
  （由長度推算；這段的 `Camera: dropped=` 沒有記錄）。
- **CNN 變慢 18%**：CNN 期間 Core 1 的 `cam_detect` 從安靜時 56% 升到 84%，兩核心同時大量讀寫 PSRAM，互搶 cache 與頻寬
  （和 10 月 FD 移到 Core 1 時 715 → 845 ms 同一現象）。
- **結果回報的成本變明顯**：`result_report` + `tiT` + `wifi` 約 214 ms / 輪（9.5%），ROI 多時每幀都有 frame / roi 記錄要轉 JSON、POST。
- 安靜時（一分鐘 5 個 ROI）`detect` 平均 78.5 ms，但最長 307 ms（`refine` 158、`contour` 84），2100 幀中丟 1 幀：
  整片變化產生的大框，見 `fd_mix.c` 的 flood fill 與 refine。

**靜止畫面**（同日稍後，畫面沒有動靜，幾乎沒有 ROI）

一輪（`Task CPU per video cycle`，30 輪平均，每輪 1999.2 ms，沒有丟幀）

| task | core | ms / 輪 | 占比 |
|---|---|---|---|
| IDLE | 0 | 1916.0 | 95.8% |
| IDLE | 1 | 1304.3 | 65.2% |
| cam_detect | 1 | 547.6 | 27.4% |
| cam_task | 1 | 96.3 | 4.8% |
| result_report | 不綁 | 54.2 | 2.7% |
| tiT | 不綁 | 41.9 | 2.1% |
| wifi | 0 | 31.3 | 1.6% |
| cam_infer | 0 | 3.4 | 0.2% |
| sys_evt / camera_capture / heartbeat / esp_timer / main | | 2.3 / 0.6 / 0.2 / 0.2 / 0.1 | 約 0.2% |
| sum | | | 200.0% |
| **busy** | | | **core 0 4.2%，core 1 34.8%** |

FD 的固定成本（`Detect stage`，300 幀，沒有框所以 refine / nms 不執行）

| 階段 | 平均 / 最長 ms | 佔 detect |
|---|---|---|
| capture / queue | 200.0 / 200.0、0.0 / 0.0 | |
| downscale | 5.3 / 6.8 | 9% |
| diff | 12.2 / 14.2 | 22% |
| **morph** | **32.5 / 33.5** | **57%** |
| contour | 6.6 / 7.3 | 12% |
| track | 0.1 / 0.2 | |
| **detect** | **56.7 / 59.2** | |

開機以來累計（`Camera:`，2432 幀，包含前面的忙碌時段）：`dropped=32 gaps=7 incomplete=7 stale=1 rois=249 expired=104 low=8 saved=8`，
幀間隔 198.9–399.8 ms；`Stack free: cam_detect=5300 B cam_infer=6904 B`；內部 RAM free 134,159 B、min 129,891 B。

**解讀（靜止）**

- **FD 的固定成本：每幀約 55–57 ms（Core 1 的 27%），其中 morph 32.5 ms 佔 57%。** 沒有任何動靜也要付。
- **結果回報的固定成本：127 ms / 輪（6.4%）。** 沒有 ROI 也照樣每幀送一筆 frame 記錄（5 筆 / s）。
- 忙碌時段累計的 `dropped=32`、`expired=104` 證實上面的推算：FD 和 CNN 都曾跟不上。
- **`incomplete=7`（相機驅動拿到不完整的幀）是新的警訊**：前一次安靜時段 7 分鐘只有 1 次。推測是兩核心同時大量讀寫 PSRAM、
  拖慢 `cam_task` 把 DMA 資料搬進 fb，**未證實**。

**下一步**

1. FD：morph / diff 改寫（任何時候都要付的成本）、整片變化過濾、flood fill 推進前檢查、Sobel 去 `sqrtf` → 降 Core 1 負載，
   也減少 PSRAM 競爭讓 CNN 回到約 234 ms，並觀察 `incomplete` 是否跟著下降。
2. BN 廣播 ADD / MUL 走 ESP-NN（MUL 有 `esp_nn_mul_broadcast_channel_s8`，ADD 要自己寫）→ CNN 估計 277 → 約 130 ms（估）。
3. 回報量：沒有候選的幀不送 frame 記錄，改用統計（靜止時省約 6%）。
4. 補記錄：忙碌時段的 `Detect stage avg/max ms`（看 FD 時間花在 morph、refine 還是 contour）。
5. Stage2 追蹤器的配對規則（累計 vs 連續、距離門檻、雜點、重複分類）：見 `docs/cam_tracker_issues.md`。
