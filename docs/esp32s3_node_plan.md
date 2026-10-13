# ESP32-S3 整合節點（cam + mic）

狀態：v3（2026-10-10）。韌體已實作，**S3 build 通過；尚未上板測試**。舊 ESP32 韌體在 `backup/esp32_legacy/`。

## 1. 定案條件
| 項目 | 內容 |
|---|---|
| 開發板 | ESP32-S3-DevKitC-1 v1.0（WROOM-1 N16R8：16 MB flash + 8 MB Octal PSRAM），杜邦線接線 |
| 燒錄 / log | 板上 COM 口（USB-UART 晶片 → UART0，GPIO43/44）；原生 USB/JTAG 口為次要 log 輸出 |
| 供電 | USB，不做休眠（之後會加入持續的模型判斷） |
| 錄製 | cam 640x480 GRAY8 RAW 5 fps + mic 16 kHz mono 16-bit，**同時錄 60 秒，每 10 分鐘一次**（開機測試完立即開始第一次，之後以上次開始時間 + 10 分鐘） |
| 上傳 | 每次錄製結束後立即上傳 `done/` 全部檔案（每小時篩選上傳為之後的目標） |
| 開機 | 先連 Wi-Fi 並向 server 校時才開始錄製 |
| SD | SDMMC **1-bit**，40 MHz |
| 相機 | 沿用 OV2640（板載 12 MHz）/ OV7670 / OV7725（MCU 輸出 16 MHz XCLK） |
| RGB LED | GPIO48 WS2812，開機輸出黑色 |

## 2. 腳位（`main/board_pins.h`）
| 功能 | GPIO |
|---|---|
| CAM XCLK / SIOD / SIOC | 15 / 4 / 5 |
| CAM D0..D7 | 11, 9, 8, 10, 12, 18, 17, 16 |
| CAM VSYNC / HREF / PCLK | 6 / 7 / 13 |
| CAM PWDN / RESET | 21 / -1（模組端 RESET 接高） |
| MIC CLK / WS / DIN | 14 / 1 / 2（PDM 不用 WS；`MIC_INTERFACE_I2S` 切換） |
| SD CLK / CMD / D0 | 40 / 38 / 39（CMD、D0 外加 10k 上拉） |
| RGB LED | 48 |

空閒可用：41、42、47（SD 改 1-bit 後空出）。保留不用：26–37（flash / PSRAM）、0/3/45/46（strapping）、19/20（USB）、43/44（UART0）。

## 3. 資料量
每次 cam 92.2 MB + mic 1.9 MB ≈ 94 MB。以 1–2 MB/s 上傳約 47–94 秒，9 分鐘空檔內可完成。
SD 寫入約 1.57 MB/s，1-bit 40 MHz 實際約 2–3 MB/s，**餘量不大**，需上板實測掉幀情況。

## 4. Task
| Task | Core | 優先權 | 用途 |
|---|---|---|---|
| mic_capture | 1 | 7 | I2S DMA → audio block（DMA 只緩衝約 256 ms） |
| camera_capture | 1 | 6 | 取 fb，只傳指標 |
| cam_task（驅動） | 1 | 驅動預設 | LCD_CAM DMA → 取出 Y → PSRAM fb |
| sd_writer / mic_writer | 0 | 5 | 各自寫入自己的預配置檔案（FATFS 鎖序列化存取） |
| heartbeat | – | 4 | 每分鐘 heartbeat + Wi-Fi watchdog |
| main（app_main） | 0 | 1 | 排程：準備 → 錄製 → 上傳 |

原本規劃單一 SD writer；實作改為兩個 writer：mic 每 16 KB 才寫一次，且有約 1 秒 block 緩衝，FATFS 鎖等待不影響，可直接沿用舊的相機寫入管線。

## 5. 流程
```
開機：NVS → LED 黑 → device id → 掛載 SD（undone/ 殘檔 → broken/）
     → Wi-Fi（一直等）→ 校時（一直等）→ heartbeat → mic init → cam init + FPS 測試
迴圈：第一次立即開始；之後等到上次開始 + 10 分 − 8 s（開機前留下的 done/ 在第一次錄完後一起上傳）
     → 準備：建立並預配置 .raw（92 MB）/ .wav
     → 同一個 trigger_us 啟動 cam 與 mic，錄 60 s
     → 各自檢核：合格 → done/；不合格 → broken/<stem>_broken.ext
     → 上傳 done/（舊的先；下一次準備前 10 s 不再開始新檔）→ 重新校時
```
- 檔名：`YYYY_MM_DD_HHMMSS.raw` / `.wav`，損壞：`YYYY_MM_DD_HHMMSS_broken.raw` / `.wav`。
- cam 與 mic **各自判定**：一邊壞掉，另一邊仍會上傳。
- **不合格標準**：
  - cam：SD 寫入錯誤，或存檔幀數少於 299（印出 `Insufficient frames`）。掉幀、斷層只記錄在 log。
  - mic：任何 I/O 錯誤、樣本數不等於 960,000。I2S DMA 溢位只印 warning，不判定損壞。
- server 永久拒收（HTTP 4xx）的檔案移到 `broken/`，避免卡住上傳佇列。
- 同步：兩者共用 `trigger_us`。上傳附 `X-Start-Offset-US`（首幀 / 首個 sample 相對觸發點），只有最近一次錄製有此值。

## 6. 上傳協定（`server_mix.py`）
- 共同 header：`X-Device-ID`、`X-Session-ID`（`<device>-<epoch>`）、`X-Capture-ID`（`<session>-video|audio`）、`X-Recorded-At`、`X-File-Size`、`X-Start-Offset-US`（選用）。
- video 另有 `X-Frame-Count / X-Width / X-Height / X-FPS / X-Format: GRAY8`；audio 另有 `X-Sample-Rate`。
- server 存成 `<root>/<device>/<YYYY-MM-DD>/<YYYY_MM_DD_HHMMSS>.raw|.wav`，用 `GET /sessions` 查配對。
- Heartbeat 與 Wi-Fi 斷線處理：見 [heartbeat.md](heartbeat.md)。

## 7. 尚未驗證 / 風險
1. **1-bit SD 40 MHz 在杜邦線上的訊號品質**：mount 失敗或 CRC 錯誤時先縮短線、加上拉電阻。降到 20 MHz 會跟不上錄影。
2. **S3 GRAY8 的 CPU 負載**：驅動在 `cam_task` 逐像素取出 Y，需實測 5 fps 是否穩定（開機 FPS 測試會檢查）。
3. **`esp32-camera-pause.patch`** 為共用程式碼（`cam_hal.c`），S3 可編譯，pause/resume 行為需上板確認。
4. **PDM mic**：改用 16-bit 取樣（依 Espressif PDM 範例），未實測；I2S mic 沿用舊版已驗證設定。
5. 92 MB 預配置所需時間（log：`Prepared in … ms`）需小於 8 s（`APP_PREPARE_LEAD_MS`）。
