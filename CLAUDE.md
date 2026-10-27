# CLAUDE.md

## 專案概要
- 野外採集節點：**ESP32-S3** 開機測試完立即開始、之後每 10 分鐘同時錄 60 秒 cam（640x480 GRAY8 RAW 5 fps）
  與 mic（16 kHz mono WAV）→ 寫 SD → 錄完立即 HTTP 上傳到 `server_mix.py`（使用者 PC，同一 LAN）。
- 板子：ESP32-S3-DevKitC-1 v1.0，WROOM-1 N16R8（16 MB flash、8 MB **Octal** PSRAM），杜邦線接線，USB 供電，不休眠。
- ESP-IDF 5.1.2，esp32-camera 2.1.7（本地修改版）。
- 規劃與決策：`docs/esp32s3_node_plan.md`；heartbeat 與 Wi-Fi 斷線處理：`docs/heartbeat.md`。
- 舊 ESP32（TTGO T8）audio/camera 韌體在 **`esp32-legacy` 分支**（本機 worktree：`C:\Users\hien\ESP_project\esp32_node`），與本分支平行開發；本分支不再保留副本（2026-10-20 起）。

## Build（使用者自己 build）
- **Claude 不要執行 build、flash、monitor**（2026-10-11 起）。改完 code 後告訴使用者改了哪些檔案、哪幾行，由使用者在 VS Code build 並回報 log。
- 使用者要求才 build 時：
```
. C:\Espressif\frameworks\esp-idf-v5.1.2-2\export.ps1   # PowerShell 預設沒有 idf.py，每個 shell 先 export
idf.py -B build\s3 build
```
- 一律用 `-B build\s3`：`build/` 由 VS Code 擴充套件 configure（cache 內 Python 路徑大小寫不同，命令列 idf.py 會拒絕）；
  不要對 `build/` 下 `fullclean`。build 目錄路徑要短（Windows 260 字元限制）。
- 沒 build 過的修改要特別注意 `-Werror`（format-truncation、未使用變數等），寫 code 時自行檢查。
- target 由 `sdkconfig.defaults` 的 `CONFIG_IDF_TARGET="esp32s3"` 決定；改 defaults 後要刪掉根目錄 `sdkconfig` 才會套用。
- 分割表：自訂 `partitions.csv`（factory 4 MB，其餘預留給之後的模型）。

## 檔案職責（main/）
- `app_main.c`：開機流程（SD → Wi-Fi → 校時 → recorder init）與排程迴圈（準備 → 錄製 → 上傳）。
- `rec_session.c`：把排程時間換成同一個 `trigger_us`，同步啟動 cam / mic，等兩邊結束。
- `cam_record.c`：相機 init / 暖機 / 開機 FPS 測試，capture → 零拷貝 queue → cam_writer；感測器暫存器設定。
- `mic_record.c`：I2S0（標準 I2S 或 PDM）→ block queue → mic_writer → 預配置 WAV。
- 兩個 recorder 都是 `prepare(epoch)` / `start(trigger_us)` / `finish(result)` 三段式；finish 內做品質檢核。
- `node_storage.c`：SD 1-bit 掛載；`undone/` → `done/`（待上傳）或 `broken/`（`<stem>_broken.ext`）；
  檔名 `YYYY_MM_DD_HHMMSS.raw|.wav`；開機時 `undone/` 殘檔視為 broken。
- `node_upload.c`：上傳 `done/`（舊的先），ACK 驗證後刪除；4xx 永久拒收移到 `broken/`；有 deadline 不擋下次錄製。
- `node_health.c`：每分鐘 heartbeat，兼 Wi-Fi watchdog。`wifi_manager.c`：重連 backoff、強制重連、driver 重啟。
- `cam_stream.c`（debug）：`APP_CAM_STREAM_ENABLE=1` 時取代錄影，拍一幀→停 XCLK→TCP 送 PC（`tools/cam_stream_view.py`），可加背景寫 SD A/B。
  `net_bench.c`（debug）：開機 iperf 測速，`APP_NET_BENCH_ENABLE`。`APP_MIC_ENABLE=0` 只錄影。
- `board_pins.h`：所有腳位。`app_config.h`：**Wi-Fi / server URL / 週期等設定的唯一來源**（換場地改這裡，不用 Kconfig）。
- `components/esp32-camera/`：**本地修改版，已納入 git**（2026-10-13 起；之前被 ignore，換電腦會缺檔）。
  `cam_record.c` include 私有 `cam_hal.h` 用 pause/resume。`patches/esp32-camera-pause.patch` 只是對原版的差異參考，可能落後於目前驅動。

## FreeRTOS tasks
| 名稱 | 優先權 | stack | core | 用途 |
|---|---|---|---|---|
| main | 1 | 8192 | 0 | 排程、上傳（HTTP 在這個 task 裡做） |
| camera_init | 6 | 8192 | 1 | 一次性：`cam_record_init()`，notify 後自刪 |
| mic_capture | 7 | 4096 | 1 | 每次錄製建立；I2S DMA 只緩衝約 256 ms，所以最高 |
| camera_capture | 6 | 4096 | 1 | 每次錄製建立；取 fb，只送指標 |
| cam_writer / mic_writer | 5 | 4096 | 0 | 每次錄製建立；寫各自的預配置檔案 |
| heartbeat | 4 | 6144 | 不綁 | heartbeat + Wi-Fi 恢復動作 |
| upload_reader | 2 | 4096 | 1 | 常駐（第一次上傳時建立，static）；讀 SD 與 main 的 HTTP 送出重疊（2 × 16 KB 緩衝） |
| cam_task（驅動） | 預設 | 4096 | 1 | LCD_CAM DMA → 取 Y → PSRAM fb（`CONFIG_CAMERA_CORE1`） |

## 任務間通訊
- cam：static queue `s_ready_queue`（深度 = 8 fb − 3 = 5，傳 `ready_frame_t`）+ static event group（CAPTURE/WRITER_DONE）。
- mic：static queue `s_free_blocks` / `s_full_blocks` 循環傳 `audio_block_t*`（NULL = 結束 sentinel）+ static event group。
- 一次性 init 回報用 task notification；Wi-Fi 狀態用 `wifi_manager` 的 event group；storage / health 各有 mutex。

## GPIO（詳見 `board_pins.h`）
- CAM：XCLK 15、SIOD 4、SIOC 5、D0..D7 = 11,9,8,10,12,18,17,16、VSYNC 6、HREF 7、PCLK 13、PWDN 未接（-1）。
- MIC：CLK 47、WS 1、DIN 2。SD：CLK 40、CMD 38、D0 39。RGB LED（WS2812）：48。空閒：14。
- LA 量測腳（debug，`APP_LA_PROBE_ENABLE`，`la_probe.h` / `la_task_trace.h`；頂層 CMakeLists 以 `-include` 注入 freertos 的
  `traceTASK_SWITCHED_IN/OUT`）：3 / 46 = CPU0 / CPU1 在 IDLE；task 腳 41 / 42 / 21 / 48 = 綁定的 task 正在執行，
  錄製時 cam_task / cam_writer / mic_capture / mic_writer，上傳時 wifi / main / upload_reader / tiT（`la_probe_set_phase`）。
  ISR 期間該核心的 idle / task 腳拉低：`la_isr_trace.S` 以 `-Wl,--wrap=_frxt_int_enter/_exit` 包中斷進出
  （main/CMakeLists 讀 `app_config.h` 決定、定義 `LA_ISR_WRAP`；level 4/5 中斷不算）。
  3、46 是 strapping 腳、48 兼 RGB LED，都只在開機後當輸出。與 `sdkconfig.sysview_uart`（UART1 用 41/42）互斥。
- **GPIO14 不要放時脈訊號**：它緊鄰 PCLK（13），I2S BCLK 放在 14 時耦合進 PCLK，錄影全部損壞（debug_sdcard #32–#33）。時脈類訊號放右側排針。
- 禁用：26–37（flash / Octal PSRAM）、0/3/45/46（strapping）、19/20（原生 USB/JTAG 口）、43/44（UART0 = 板上 COM 口，燒錄與主要 log）。
- OV2640 用板載 12 MHz → `USE_OV2640_ONBOARD_XVCLK=1`；OV7670/OV7725 由 S3 輸出 16 MHz → `=0`（預設）。換模組要切。

## 記憶體
- 相機 fb 8 × 300 KB 放 PSRAM；驅動 DMA buffer（≤ 32 KB）在內部 RAM，GRAY8 由 `cam_task` CPU 取出 Y。
- SDMMC DMA 不能直接吃 PSRAM：寫 SD 一律經內部 RAM 暫存（cam 用 `node_storage_io_buffer_acquire()` 的共用 32 KB static 緩衝，
  錄影 prepare→finish 持有、上傳時切成 2 × 16 KB 讀檔雙緩衝，兩者不可同時；mic 16 KB static）。
- 錄製檔開始前預先撐大（cam 92 MB、wav 1.9 MB），結束時 `ftruncate`，避免 FAT 延遲尖峰。
- 優先 static 配置（`xQueueCreateStatic` / `xEventGroupCreateStatic` / static buffer）。
- Wi-Fi 必須用 static TX buffer（IDF 5.1 下 dynamic TX 與 `SPIRAM_USE_MALLOC` 不相容）；`WIFI_PS_NONE`。

## 上傳協定（ESP32 ↔ server_mix.py）
- POST `/upload/video` | `/upload/audio`，body 為檔案原始位元組。header：`X-Device-ID`、`X-Session-ID`、
  `X-Capture-ID`（`<session>-video|audio`）、`X-Recorded-At`、`X-File-Size`、選用 `X-Start-Offset-US`；
  video 加 `X-Frame-Count/Width/Height/FPS/Format`，audio 加 `X-Sample-Rate`。兩邊要同步修改。
- ACK `{"status":"ok","capture_id":…,"bytes":…}` 驗證通過才刪本地檔；重送回 `duplicate: true`。
- 另有 GET `/time`、POST `/heartbeat`（格式見 `docs/heartbeat.md`）、GET `/devices`、`/sessions`、`/uploads`。
- 舊 `server.py` 只給 legacy 韌體用。

## 程式風格
- 模組 API：`<module>_<verb>()` 回傳 `esp_err_t`；結果用 `*_result_t` / `*_summary_t` out-param。
- 檔內 static 變數 `s_` 前綴；每檔 `static const char *TAG`。
- 錯誤處理：`ESP_RETURN_ON_ERROR`、`ESP_ERROR_CHECK`（只在 app_main 開機流程）。
- 常數 `#define`，無號值加 `U`，單位寫進名稱（`_MS`/`_US`/`_SEC`/`_BYTES`）；`_Static_assert` 檢查大小關係。
- printf 整數 cast `(unsigned)` 或用 `PRIu64`/`PRId64`；`-Werror=format-truncation` 有開，snprintf 緩衝要夠大。
- 4 空格縮排、函式大括號換行、`#pragma once`；註解繁體中文為主（說明為什麼），log 英文。
- 雙核心綁定以 `#if CONFIG_FREERTOS_UNICORE` 保護。

## 待確認
- SD 卡寫入穩定性：進行中，紀錄在 `docs/debug_sdcard.md`（新的測試結果要補進時間軸）。目前 40 MHz + 新卡可運作。
- 已上板確認：S3 GRAY8 5 fps 穩定、pause/resume 正常、I2S mic 正常。尚未驗證：PDM 16-bit 取樣。
- 錄影影像損壞已解決（10/13，BCLK 14→47，見 `docs/debug_sdcard.md` 狀態摘要）。待查：錄影期間內部 RAM 最低約 2.7 KB。
