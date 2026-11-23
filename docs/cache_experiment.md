# Cache 實驗紀錄（infer_node，ESP32-S3）

各版本定義與 build 方式見 `infer_node/README.md`「Cache 實驗」。

| 版本 | ICache | DCache | 模型權重 | 內部 RAM 代價 |
|---|---|---|---|---|
| A | 16 KB | 32 KB / 32 B line | flash | — |
| B | 16 KB | 64 KB / 64 B line | flash | −32 KB |
| C | 32 KB | 64 KB / 64 B line | flash | −48 KB |
| D | 16 KB | 64 KB / 64 B line | PSRAM（開機複製） | −32 KB，PSRAM −295 KB |

---

## 2026-11-23　版本 D（相機 + 麥克風）

### 條件

| 項目 | 設定 |
|---|---|
| Build | `idf.py -B build_cache_d_NN -DSDKCONFIG=build_cache_d/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.dcache64" -DINFER_MODEL_IN_PSRAM=1 -DESP_NN_KERNEL_SET=all build`（在 `infer_node/` 下） |
| 韌體 | commit `671694c`（dirty，含 `INFER_ROLE_BOTH`、判斷延遲 `result_us`、cache 實驗開關），`-O2` |
| 開機確認 | `Cache: instruction 16 KB, data 64 KB with 64-byte lines`；`AUDIO_MODEL ... (PSRAM)`、`CAM_MODEL ... (PSRAM)` |
| 角色 | `INFER_ROLE_BOTH`：Core 1 = cam_task / mic_capture / camera_capture / cam_detect；Core 0 = cam_writer(5) > audio_infer(3) > audio_writer(2) > cam_infer(1) = main(1) |
| 相機 | OV7670（PID 0x76），XCLK GPIO9 16 MHz，GRAY8 640×480 5 fps |
| 存檔 | `INFER_LOW_CONF_THRESHOLD 0.7`、`INFER_SAVE_NON_BACKGROUND 1`（非背景的 ROI / 視窗都存） |
| 輸入 | `10bird` 影片播放數次（ROI 集中在開機後 45–64 s、93–108 s、128–146 s、166–184 s、231 s），期間有兩批影像上傳、一批音訊上傳 |
| 長度 | 約 4 分鐘：相機 900 幀、67 個 ROI；音訊 104 個視窗（w0–w103） |

### 1. 記憶體

| | 版本 D | 參考：同日稍早（ICache 16 / DCache 32 / 模型在 flash） |
|---|---|---|
| `HEAP [running]` internal free | 59,143 B | 91,143 B |
| 執行期間 internal min | **53,319 B**（兩次上傳後） | — |
| PSRAM free（running） | 4,334,832 B | 4,629,668 B |

- 內部 RAM 少 32,000 B，和 DCache 多拿走的 32 KB 吻合（`heap_init` 少了 `3FCF0000` 那塊 32 KiB DRAM）。
- PSRAM 少 294,836 B，和兩個模型的大小相符（281,392 + 13,432 B）。
- 上傳期間 internal min 沒有再往下掉，53 KB 的餘裕足夠。
- Stack free：cam_detect 5,300 B，cam_infer 6,824 B。

### 2. 相機 CNN（67 個 ROI，單位 ms）

| 指標 | 平均 | 中位數 | P95 | 最小 | 最大 |
|---|---|---|---|---|---|
| invoke（實際經過時間） | 80.0 | 61.1 | 185.8 | 57.0 | 329.4 |
| run（cam_infer 自己的 CPU 時間） | 60.3 | 57.9 | 72.9 | 57.1 | 78.5 |
| preempt（被同核心搶走） | 19.8 | 0.9 | 118.7 | 0.0 | 257.9 |
| fd_overlap（同期間 FD 在另一核心忙） | 16.4 | 0.0 | 92.4 | 0.0 | 197.2 |

- **沒有干擾時的 CNN（fd_overlap = 0 且 preempt < 2 ms，33 次）：run = 57.6 ms（中位數 57.4）**。
  - 參考：論文 FD/CNN 拖慢實驗的 S3 · ESP-NN（2026-11-21/22，ICache 16 / DCache 32 / 模型在 flash，單相機），1drone 的 run 約 72.7 ms。
  - 少了約 21%。這段沒有 FD 重疊，所以和 FD 的改寫無關，主要是 cache / 權重位置的差異；B 和 D 各占多少，要等 A、B 的數據。
- **FD 重疊時的拖慢**：run 對 fd_overlap 的回歸斜率 **0.113 ms/ms**（r = 0.77，截距 58.5 ms）；
  節點自己的每分鐘統計是 0.10–0.14。論文的 S3 · ESP-NN 是 0.175。FD 在另一核心跑時，CNN 被拖慢得比較少。
- **被搶占的時間占 invoke 的 24.7%**。分布：< 2 ms 38 次、2–20 ms 19 次、20–100 ms 4 次、≥ 100 ms 6 次。
  ≥ 100 ms 的 6 次：

| 幀 | invoke | run | preempt | fd_overlap | 對照 |
|---|---|---|---|---|---|
| 119 | 312.8 | 57.9 | 255.0 | 98.5 | 和音訊 w11（logmel + CNN 252 ms）同時 |
| 699 | 329.4 | 71.6 | 257.9 | 197.2 | 和音訊 w69（255 ms）同時 |
| 91 | 184.5 | 67.5 | 117.1 | 95.3 | 前一幀（90，LOW）剛交給 cam_writer 存檔 |
| 92 | 181.4 | 70.2 | 111.3 | 95.7 | 前一幀（91，LOW）剛交給 cam_writer 存檔 |
| 708 | 195.6 | 63.1 | 132.6 | 85.6 | 前面的 706（LOW）剛交給 cam_writer 存檔 |
| 661 | 186.4 | 67.0 | 119.4 | 84.4 | 未確定 |

  - 搶占 250 ms 的兩次就是音訊推論，符合設計（audio_infer 3 > cam_infer 1）。
  - 110–130 ms 的幾次多半發生在存檔之後，推測是 cam_writer（prio 5）；第 661 幀還沒有對上的事件。

**CNN 各層**（每分鐘的平均，3 次回報，µs）：

| 運算 | 18 次 | 27 次 | 22 次 |
|---|---|---|---|
| CONV_2D（第 2 個 op，最大的一層） | 39,381 | 36,340 | 48,189 |
| FULLY_CONNECTED（4096→64） | 13,907 | 12,293 | 14,408 |
| DEPTHWISE_CONV_2D（第 1 個 op） | 7,977 | 6,565 | 6,332 |
| 其他合計 | 約 31,000 | 約 13,700 | 約 15,300 |

第一次回報的「其他」偏高，是因為有一層 DEPTHWISE_CONV_2D 達 14,764 µs，被搶占的時間也算在層裡。

### 3. FD（cam_detect，Core 1，每 300 幀一次，ms）

| 區間 | downscale | diff | **morph** | contour | refine | detect 平均 / 最長 |
|---|---|---|---|---|---|---|
| 幀 0–299 | 4.2 | 8.0 | **59.5** | 6.6 | 2.3 | 80.9 / 134.5 |
| 幀 300–599 | 4.3 | 8.4 | **61.4** | 6.6 | 2.6 | 83.6 / 141.6 |
| 幀 600–899 | 4.3 | 8.3 | **61.6** | 6.5 | 2.0 | 82.9 / 128.7 |

- detect 最長 141.6 ms，低於 200 ms 的幀週期：**900 幀 dropped = 0、gaps = 0、incomplete = 0**，`expired = 1`。
- **morph 要注意**：2026-11-10 靜止畫面（舊版 fd_mix、ICache 16 / DCache 32）是 diff 12.2 / morph 32.5 / detect 56.7 ms。
  diff 因為和二值化合併而變快（12.2 → 8.3），但 **morph 從 32.5 變成約 60 ms，慢了將近一倍**。
  morph 的成本和畫面內容無關，每幀固定。最可能的原因是 2026-11-23 改寫的環形行緩衝版本（commit `85ee84f`）內層迴圈太慢：
  每個像素都做 `r % ring_rows` 取餘數，以及 dilate / erode 的分支判斷。也可能和 64 B line 有關。要用版本 A 確認。

### 4. 音訊（104 個視窗，ms）

| 指標 | 平均 | 中位數 | P95 | 最小 | 最大 |
|---|---|---|---|---|---|
| logmel | 200.1 | 184.5 | 358.1 | 178 | 435 |
| logmel（去掉 7 次 ≥ 300 ms） | 185.9 | 184.0 | 194.6 | 178 | 254 |
| CNN | 67.6 | 66.0 | 76.0 | 54 | 185 |
| logmel + CNN | 267.7 | 249.5 | 418.5 | 239 | 596 |

- **ovf = 0、dropped = 0**（整段，含上傳期間）。最慢一個視窗 596 ms，2 s 的期限還有很大餘裕。
- **logmel ≥ 300 ms 的 7 次（w29、w32、w34、w35、w53、w70、w71）都剛好在相機存一幀之後開始**
  （例如 w32 從約 99.65 s 開始，第 329 幀 LOW 在 99.585 s 交給 cam_writer）。
  cam_writer（prio 5）比 audio_infer（prio 3）高，存一張 300 KB 的幀會讓 logmel 多出約 170–250 ms。
  這是目前優先權配置的直接結果，不是 cache 的問題。
- **音訊 CNN 隨時間變慢**：w0–28 平均 58.5 ms → w36–58 為 64.9 ms → w72–103 為 72.2 ms（+23%）。logmel 沒有跟著變。
  第一次上傳（105 s）之後開始變慢，上傳結束後也沒有回來。原因還不知道，要看版本 A 是否也一樣。

### 5. 上傳（期間擷取與推論照常）

| 批次 | 樣本 | 每個樣本耗時 平均 / 中位數 / 範圍 | gap_wait |
|---|---|---|---|
| 影像 1（105–130 s） | 25 × 311 KB | 661 / 521 / 362–1,504 ms（約 0.47 MB/s） | 6,578 ms |
| 影像 2（185–203 s） | 20 × 311 KB | 596 / 496 / 390–1,188 ms | 4,535 ms |
| 音訊（226–239 s） | 21 × 64 KB | 381 / 264 / 198–1,108 ms | 3,063 ms |

上傳期間相機沒有丟幀，音訊沒有 overflow 或丟視窗，`failed = 0`、`rejected = 0`。

### 6. 小結與待辦

1. 版本 D 的相機 CNN 在沒有干擾時是 57.4 ms，比論文數據（約 72.7 ms）快約 21%；FD 拖慢的斜率也從 0.175 降到約 0.11 ms/ms。
   **要跑版本 A、B 才能把效果分給「DCache 加大」和「權重搬到 PSRAM」**，並排除其他差異（角色、韌體版本）。
2. morph 從 32.5 ms 變成約 60 ms，懷疑是我改寫的 morphology 內層迴圈。可以把取餘數和分支移出內層迴圈，修完再量。
3. cam_writer（prio 5）比 audio_infer 高，存檔時會讓音訊推論多花約 200 ms，也會搶占相機 CNN。
   可以考慮把 cam_writer 降到 audio_infer 之下（例如 2）。
4. 音訊 CNN 隨時間從約 58 ms 變慢到約 72 ms，原因不明，要用版本 A 對照。
