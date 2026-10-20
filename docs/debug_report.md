# ESP32-S3 採集節點 debug 報告：上傳速度與影像損壞

日期：2026-10-10 ～ 2026-10-13
原始紀錄：`docs/debug_sdcard.md`（以下 `#n` 為該檔時間軸編號）

---

## 0. 系統與量測條件

| 項目 | 內容 |
|---|---|
| 板子 | ESP32-S3-DevKitC-1 v1.0，WROOM-1 N16R8（8 MB Octal PSRAM），杜邦線接線，USB 供電 |
| 相機 | OV7670，640×480 GRAY8（感測器輸出 YUV422，驅動取 Y），5 fps，XCLK 16 MHz（GPIO15），PCLK GPIO13 |
| 麥克風 | I2S 16 kHz mono（BCLK ≈ 16000 × 32 bit × 2 ch = 1.024 MHz） |
| SD | SDMMC 1-bit，40 MHz（CLK 40、CMD 38、D0 39），卡 USDU1 |
| 檔案 | 60 s 錄影 = 300 幀 × 307 200 B = **92.16 MB**（RAW）；錄音 1.92 MB（WAV） |
| 網路 | 2.4 GHz，AP 與 PC 在同一台路由器；PC 端 Flask（`server_mix.py`） |
| 計時方式 | 韌體內 `esp_timer` 分段累計：`sd_read`（讀 SD）、`wifi_send`（`esp_http_client_write`）、`wait_sd`（送出端等待資料）、`ack_wait` |

速度單位：kB/s = 1000 B/s。

---

## 1. 上傳速度

### 1.1 數據總表

| 步驟 | 條件 | 量測結果 | 結論 | 紀錄 |
|---|---|---|---|---|
| 1 | 原始 HTTP 上傳 | WAV：wifi_send 265–317 kB/s，佔上傳時間 77–82%；sd_read 1127–1372 kB/s | 上傳約 0.3 MB/s，瓶頸在送出端 | #9 |
| 2 | ESP-IDF iperf 範例（同板、同 AP） | 59.8 Mbps ≈ **7.5 MB/s** | 無線鏈路與 AP 不是瓶頸 | #11 |
| 3 | PC 本機 POST 92 MB 到 Flask | 0.70 s ≈ 132 MB/s | PC 接收端不是瓶頸 → 問題在 ESP 韌體 | #12, #13 |
| 4 | 韌體內 socket 測速（`net_bench`，8 MB，不經 HTTP） | 相機初始化前 **4894 kB/s**；相機／mic／SD 初始化後 **71 kB/s**，並出現 `errno=104` | 初始化後內部 RAM 只剩 33.8 KB，lwIP 配不到緩衝 | #16 |
| 5 | 開啟 `SPIRAM_TRY_ALLOCATE_WIFI_LWIP` | 初始化後內部 RAM 33.8 → 80.7 KB；`after_init` 204 kB/s | 記憶體問題解除，但仍慢 → 還有其他原因 | #17 |
| 6 | 相機與線材移離天線 | `after_init` 645 kB/s | 相機匯流排對 Wi-Fi 有干擾 | #18 |
| 7 | XCLK A/B（同一次開機） | XCLK 開 **406** → 關 **3851** → 再開 **476** kB/s | 干擾源為相機匯流排（XCLK 與其帶動的 PCLK／資料線），可重現 | #19 |
| 8 | 非錄影期間停止 XCLK 與 I2S 時脈 | 上傳 92 MB：wifi_send **4927 kB/s**；sd_read **1296 kB/s**（佔 79%）；總計 90 s | Wi-Fi 端解決；新瓶頸為讀 SD | #20 |
| 9 | 讀 SD 改無緩衝 `fread`（`setvbuf _IONBF`） | sd_read **40 kB/s** | 更慢，撤回 | #21 |
| 10 | 撤回後基準（有緩衝 `fread`） | 92 MB：總計 **104 s（886 kB/s）** = sd_read 85.8 s（1073 kB/s）+ wifi_send 18.0 s（5113 kB/s） | 讀 SD 佔 83% | #22 |
| 11 | 讀 SD 改 `open()` / `read()` | 92 MB：總計 **42.9 s（2146 kB/s）** = sd_read 23.1 s（**3994 kB/s**）+ wifi_send 19.8 s（4662 kB/s） | 讀 SD 1.07 → 4.0 MB/s；讀與送仍輪流執行 | #23 |
| 12 | 讀 SD 與送出改為兩個 task 並行 | 92 MB：總計 **40.7 s（2265 kB/s）**；sd_read 24.1 s 與 wifi_send 38.2 s 重疊，wait_sd 2.2 s | 總時間 ≈ wifi_send + 2 s，讀 SD 不再增加總時間 | #24 |
| 13 | 相機暫存區與上傳緩衝共用 32 KB | 92 MB：總計 **37.1 s（2492 kB/s）**；wifi_send 2653、sd_read 3797 kB/s | 錄影寫入暫存區恢復 32 KB，上傳不變 | #25 |

**注意（步驟 12、13 的環境差異）**：這兩次 AP 由 channel 1／40 MHz 頻寬換成 channel 11／20 MHz 頻寬，開機 socket 測速由約 4500 降到約 2300 kB/s。因此步驟 12、13 的總時間受 Wi-Fi 限制，不能直接和步驟 11 比較；比較並行效果應看「總時間 vs. sd_read + wifi_send」（40.7 s vs. 62.3 s）。40 MHz 頻寬下預估總時間約為 max(sd_read, wifi_send) ≈ 23–25 s（未實測）。

### 1.2 各步驟原因說明

1. **內部 RAM 不足（步驟 4–5）**：相機驅動 DMA 緩衝（30 KB）、SD 暫存區、mic 等都在內部 RAM，初始化後 lwIP 封包緩衝配置失敗，TCP 重傳到上限後連線被重置。讓 Wi-Fi／lwIP 優先使用 PSRAM 後解除。
2. **相機匯流排干擾（步驟 6–8）**：XCLK 16 MHz 與相機資料線經杜邦線輻射，影響 2.4 GHz 收發。改為只在錄影期間輸出 XCLK 與 I2S 時脈（`cam_record_set_xclk_output()`、`i2s_channel_enable/disable`），上傳時兩者皆停止。
3. **讀 SD 方式（步驟 9–11）**：
   - `fread` 經 newlib stdio 的 128 B 緩衝，16 KB 讀取被拆成 128 次小讀取，FATFS 只能逐 sector（CMD17）讀，約 15 ms／16 KB。
   - 無緩衝 `fread` 時，S3 ROM 內的 newlib 每次只讀 1 byte（推測，與 40 kB/s 吻合）。
   - `read()` 直接把 16 KB 讀進 4-byte 對齊的內部 RAM 緩衝，FATFS 一次讀 32 個 sector（CMD18），SDMMC DMA 直接寫入，約 4 ms／16 KB。
4. **讀與送輪流（步驟 11–12）**：單一 task 時總時間 = 讀 + 送。改成生產者／消費者兩個 task 後總時間 ≈ 較慢的一方。

### 1.3 上傳資料流與 task／core 配置（步驟 12 之後）

```
SD 卡 ──D0 (40 MHz, 1-bit)──► SDMMC 控制器 ──DMA──► 共用 I/O 緩衝 32 KB（內部 RAM，切成 buf[0] / buf[1] 各 16 KB）
                                                   ▲                     │
                                  upload_reader task                     │ s_full_queue {index, length}
                                  Core 1、優先權 2                        ▼
                                  read(fd, buf[i], 16 KB)        main task（Core 0、優先權 1）
                                         ▲                       esp_http_client_write(buf[i])
                                         │ s_free_queue {index}          │
                                         └───────────────────────────────┘
                                                                         ▼
                                     lwIP TCP 傳送佇列（PSRAM）→ Wi-Fi driver（Core 0、static TX buffer 內部 RAM）→ AP → PC
```

| task | core | 優先權 | 工作 |
|---|---|---|---|
| main | 0 | 1 | HTTP 連線、送出、ACK 驗證、刪檔 |
| upload_reader | 1 | 2 | `read()` 16 KB 到空緩衝，交給 main；常駐（static 建立） |
| wifi（系統） | 0 | 23 | Wi-Fi driver |

- 兩塊緩衝輪流：reader 讀 buf[1] 時，main 送 buf[0]。
- 結束：reader 最後送出 length ≤ 0 的結束訊息；main 收到後才 `close(fd)`、歸還緩衝。送出失敗時 main 設 abort 旗標，繼續歸還緩衝直到收到結束訊息。
- 錄影與上傳不會同時進行，所以相機的 32 KB 寫入暫存區與上傳的 2 × 16 KB 共用同一塊（`node_storage_io_buffer_acquire/release`），內部 RAM 由 76 KB 回到 100 KB（#24 → #25）。

### 1.4 時間分配對照（92.16 MB RAW）

| 步驟 | 讀 SD | Wi-Fi 送出 | 方式 | 總時間 | 平均 |
|---|---|---|---|---|---|
| 10 `fread` | 85.8 s | 18.0 s | 輪流 | 104.0 s | 886 kB/s |
| 11 `read()` | 23.1 s | 19.8 s | 輪流 | 42.9 s | 2146 kB/s |
| 12 雙 task | 24.1 s | 38.2 s（BW20） | 並行 | 40.7 s | 2265 kB/s |
| 13 共用緩衝 | 24.3 s | 34.8 s（BW20） | 並行 | 37.1 s | 2492 kB/s |

---

## 2. 錄影影像損壞

### 2.1 現象與判斷指標

- 現象：水平條紋、整片灰（數值約 128，取到 U/V 而非 Y）、畫面左右錯開；錄影越久越嚴重。錄影統計正常（300/300 幀、dropped 0），驅動未報錯。
- PC 端工具：`tools/raw_view.py --stats`（RAW 逐幀）、`tools/cam_stream_view.py`（串流逐幀）。
- 指標：`row_jump` = 相鄰兩行平均亮度差，`col_jump` = 相鄰兩列平均亮度差。正常影像 row_jump ≤ col_jump（比值約 0.9–1.0）；行錯位時 row_jump 明顯變大。

### 2.2 排查流程

| # | 假設 | 測試 | 結果 | 結論 |
|---|---|---|---|---|
| 26 | PC 解碼錯誤 | 同一解碼方式檢查 40 個 RAW | 有檔案 300 幀全部正常；多數檔案損壞 | 解碼正確，資料在擷取或寫入時已壞 |
| 27 | 感測器／PCLK 取樣錯誤 | 邏輯分析儀量 PCLK；串流模式（拍一幀→停 XCLK→TCP 送 PC，不寫 SD、I2S 停止） | 每行 PCLK 1280 個（640 × 2，正確）；串流 70+ 幀正常，比值 0.94–0.98 | 感測器、驅動、解碼正常；問題只出現在錄影流程 |
| 28 | 擷取本身偶發錯誤 | 串流 522 幀 | 7/522（1.3%）可疑，集中一陣，同時出現 `FB-OVF`（每幀收到過多資料） | 串流也有少量錯誤，原因未確定（見第 3 節） |
| 29 | D6 資料線（GPIO17）不良 | 錄影檔逐像素分析：壞像素能否以單一位元翻轉解釋 | D6 翻轉解釋 25.4%，鄰近位元約 2% | 指向 D6，後經 #30 推翻 |
| 30 | 接地／電源（寫 SD 電流造成地彈） | 換 D6 杜邦線後錄影 | 壞像素約 42%，錯誤擴散到 D7 與位元組錯位 | 不是單一條線；當時推測為接地，後經 #31 推翻 |
| 31 | 寫 SD 的電氣干擾 | 串流＋背景寫 SD（同樣 200 ms 節奏、32 KB、Core 0），每 20 幀切換 | 不寫 SD 0/20 可疑；寫 SD **0/14 可疑** | 寫 SD 不會造成損壞。剩下差異：錄影時 I2S 在運作 |
| 32a | mic 杜邦線耦合 | 拔掉 mic 線後錄影 | 仍嚴重損壞 | 與 mic 線無關 |
| 32b | ESP 端 I2S 運作 | `APP_MIC_ENABLE=0`（I2S 不初始化）錄影 | **50/50 幀正常** | 根因在 I2S 運作（拔線時 ESP 仍輸出 BCLK） |
| 33 | GPIO14（BCLK）與 GPIO13（PCLK）相鄰耦合 | BCLK 由 GPIO14 改到 GPIO47（另一側排針），I2S 照常運作錄影 | **50/50 幀正常**：row_jump 1.3–1.9 < col_jump 1.8–2.6；靜止段前後幀差 0.5 | 確定與 BCLK 腳位位置有關 |
| 34 | 完整流程驗證 | 開機 → 錄影＋錄音 → 上傳 | 錄影 50/50、錄音 160000/160000、上傳 ACK | 修正有效 |

### 2.3 根因與修正

- **根因**：mic I2S BCLK（約 1.024 MHz）原本在 GPIO14，與相機 PCLK（GPIO13）在排針上相鄰，杜邦線並排。BCLK 邊緣耦合到 PCLK 線上，造成 PCLK 多出或缺少取樣邊緣：
  - 少量多／少邊緣 → 位元組相位錯一位 → 取到 U/V（整片灰）、整行錯位（條紋）。
  - 也會影響資料線判讀（#29、#30 看到的位元翻轉）。
- **修正**：`MIC_PIN_CLK` 改為 GPIO47（`main/board_pins.h`）。
- **規則**：時脈類訊號（I2S BCLK／WS、SD CLK）不放在相機排針同一側，且不與 PCLK 相鄰。
- **排查中的誤判**：#26–#30 將原因歸於寫 SD、D6 線、接地。原因是早期的對照組（串流模式）同時停止了 I2S，比較時有兩個變因。#31 只改一個變因（加入寫 SD）後才排除。

### 2.4 用示波器驗證耦合

目前的證據是 A/B 測試（只改 BCLK 腳位，結果由損壞變正常），耦合波形本身尚未直接量到。可以用示波器確認，邏輯分析儀較難（#27 的 LA 量到每行 1280 個邊緣，取樣率與門檻設定會漏掉 ns 級突波）。

**設備**：頻寬 ≥ 100 MHz、取樣率 ≥ 500 MSa/s；10× 探棒，使用接地彈簧（不要用長接地夾，長地線本身會產生振鈴）。

**量測點**：CH1 = GPIO13（PCLK），探在 S3 排針端；CH2 = GPIO14。地接在最近的 GND 腳。

**步驟**：
1. **只量耦合（最乾淨）**：相機 XCLK 停止（PCLK 靜止），I2S BCLK 輸出在 GPIO14。以 CH2 上升緣觸發，開持續顯示（persistence）。CH1 上若在 BCLK 邊緣同時出現突波，即為耦合；記錄峰值電壓與寬度。
2. **實際工作狀態**：相機與 I2S 同時運作（錄影中），同樣以 CH2 觸發，看 PCLK 邊緣附近是否出現額外跳變或振鈴。
3. **對照組**：BCLK 改到 GPIO47（現行韌體），重複 1、2。突波應明顯變小或消失。
4. **判斷**：S3 輸入門檻約 VIL ≤ 0.25 × VDD（≈ 0.83 V）、VIH ≥ 0.75 × VDD（≈ 2.48 V）（以 ESP32-S3 datasheet 為準）。PCLK 低電位時突波超過約 0.83 V、或高電位時跌破約 2.48 V，就可能被判讀成多一個邊緣。

**可放進報告的圖**：同一時間軸上的 BCLK 與 PCLK 波形（BCLK 在 14 與在 47 各一張），標出突波峰值。

---

## 3. 未解決與限制

1. **串流模式 1.3% 可疑幀（#28）**：I2S 停止時仍出現，且伴隨 `FB-OVF`。推測與 Wi-Fi 發射或其他干擾有關，未驗證。
2. **錄影期間內部 RAM 最低約 2.7 KB（#34）**：原因推測為這台電腦的 `sdkconfig` 未開 `SPIRAM_TRY_ALLOCATE_WIFI_LWIP`（步驟 5 的設定只存在另一台電腦）。已補進 `sdkconfig.defaults`，待重新 build 驗證。
3. **40 MHz 頻寬下的上傳總時間**：並行架構在 40 MHz 頻寬下尚未實測（預估 23–25 s）。
4. **錄影期間相機匯流排仍會干擾 Wi-Fi**：目前流程錄影時不上傳，不受影響；若未來需要邊錄邊傳，需要處理硬體走線。
5. **開機偶發 warm-up 逾時（#30）**：換 BCLK 腳位後未再出現，原因未證實。
