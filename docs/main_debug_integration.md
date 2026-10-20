# main：legacy 除錯功能整合

本次加入相機 TCP 串流、開機網路吞吐量 A/B 測試，以及音訊／影像上傳讀檔優化與分段計時。
依使用者選擇，`APP_CAM_STREAM_ENABLE` 與 `APP_NET_BENCH_ENABLE` 都預設為 `1`。
audio 與 camera 仍透過 `FIELD_FIRMWARE` 分開編譯，沒有同時啟用兩種擷取。

## 操作

先在 `main/app_config.h` 將 `APP_NET_BENCH_HOST`、`APP_CAM_STREAM_HOST` 改成 PC 的 IPv4。
目前沿用 legacy 的 `10.10.10.7`；Wi-Fi、HTTP server 與錄製排程設定均未變更。

在 PC 啟動兩個接收程式：

```powershell
# iperf2，不是 iperf3；預設 TCP 5001
iperf -s -i 1

# 另一個終端機；預設 TCP 5002，需要 numpy 與 opencv-python
python tools/cam_stream_view.py
```

camera 開機順序：Wi-Fi／校時 → 初始化前測速 → SD／heartbeat → 補傳舊檔 →
原有相機初始化／FPS 測試 → 相機 A/B 測速 → 持續 TCP 串流。
PC 尚未啟動 viewer 時會持續重連；串流不寫入 SD，也不執行新影像的 HTTP 上傳。
若初始化、記憶體配置或周邊狀態有錯誤，仍可能停止或返回原有排程。

audio 開機順序：Wi-Fi／校時 → 初始化前測速 → 原有 mic 初始化 → mic A/B 測速 →
原有 heartbeat／補傳舊檔／錄音及上傳排程。

若要恢復正常錄影，將 `APP_CAM_STREAM_ENABLE` 設為 `0` 後重新編譯；
若要跳過開機測速與其周邊切換，將 `APP_NET_BENCH_ENABLE` 設為 `0`。
上傳讀檔調整與分段計時持續生效。

## A/B 與硬體狀態

- 每個測速狀態從 RAM 透過 TCP 傳送 8 MiB，不經 HTTP 或 SD。
- 相機比較初始化前、接收運作中、停止 DMA／VSYNC 後的網路吞吐量。
- 若由 ESP32 輸出 XCLK，另比較 XCLK 關閉、重新開啟；仍沿用原有 LEDC timer 狀態。
- 目前 `USE_OV2640_ONBOARD_XVCLK=1`：板載振盪器無法由 ESP32 停止，因此明確跳過 XCLK 關／開測試。
- mic 保留初始化狀態：目前 PDM 比較開 → 關 → 開，標準 I2S 則比較關 → 開 → 關。
- mic 測試使用既有錄音 mutex。PDM 重新給時脈後，重新計算原有 7 秒穩定等待；
  原有啟動資料丟棄、每秒音量／削波紀錄與錄音 pipeline 保持不變。
- 網路測速失敗會記錄並繼續後續流程；周邊控制錯誤則回報給啟動流程處理。

## 串流與記憶體

串流沿用 legacy 的 24-byte little-endian `CAMF` header，後接 640×480 GRAY8 資料。
每次恢復擷取後丟棄 3 幀，再複製一幀到 PSRAM、歸還原始 frame buffer、等待 HAL 暫停完成，最後傳送副本。
串流沿用 `main` 原有 `cam_record_pause/resume()`，不取代其 XCLK 暫停、恢復與穩定等待。
板載振盪器模式下，暫停 DMA 不代表感測器的 PCLK／資料線已停止。

| 項目 | 配置與生命週期 |
|---|---|
| 串流影格副本 | 額外 307,200 bytes PSRAM，串流期間保留 |
| 測速緩衝區 | 每次測速動態配置 16 KiB，優先內部 RAM、不足時嘗試 PSRAM，結束後釋放 |
| 上傳緩衝區 | 每種韌體原有的 16 KiB，改以 `DMA_ATTR` 明確指定內部 DMA RAM 與 4-byte 對齊 |

以上不包含 socket／lwIP 的動態配置，也不是實際執行期間記憶體峰值。
串流重視畫面完整性，恢復、丟幀與傳輸都包含在觀察到的 FPS 中，不保證 5 FPS。

## 上傳計時

`upload.c`、`cam_upload.c` 使用 `open/read/close`，避開 stdio 小緩衝。
讀取量限制在剩餘檔案長度內；HTTP 部分寫入仍以迴圈補完。
保留原有 `X-Session-ID`、其他 header、ACK 驗證與成功後刪除本機檔案的條件。

`Upload timing` 紀錄資料傳送至 ACK／連線清理的總時間、SD 讀取累計／最長時間、
HTTP write 累計／最長時間、ACK 等待，以及各段吞吐量。
計時從 HTTP open 成功後開始，不包含連線建立；HTTP write 耗時包含 TCP 背壓，不能視為純無線 airtime。
實際效能需用目前板子、SD 卡與網路量測。

## 修改檔案

- 新增：`main/net_bench.c`、`main/net_bench.h`、`main/cam_stream.c`、`main/cam_stream.h`。
- 設定／啟動：`main/CMakeLists.txt`、`main/app_config.h`、`main/app_main.c`、`main/cam_app_main.c`。
- 周邊測速支援：`main/audio_record.c`、`main/audio_record.h`、`main/cam_record.c`、`main/cam_record.h`。
- 上傳：`main/upload.c`、`main/cam_upload.c`。
- 文件：本檔。

`esp32_legacy`、Wi-Fi 省電設定、相機／mic 的硬體選擇與接腳、原有錄製長度及週期均保持不變。

## 本次驗證

- ESP-IDF 5.1.2、ESP32：audio 與 camera 均完成編譯、連結及分割區大小檢查。
- audio 產物：`build/esp_http_client.bin`；camera 產物：`build_camera_integration/build/esp_http_client.bin`。
- audio 映像為 `0xfc430` bytes，現有 1 MiB app 分割區剩 `0x3bd0` bytes（約 15 KiB）。
- camera 映像為 `0x126670` bytes，現有 app 分割區剩 `0x50990` bytes（約 322 KiB）。
- 交叉編譯確認兩個除錯開關關閉的相關來源，以及標準 I2S、ESP32 輸出 XCLK 的替代分支。
  替代硬體分支只在 build 內的副本調整巨集，未修改正式來源的硬體選擇。
- 主機 C 模擬確認 mic 開／關狀態還原、網路失敗、mutex 排他與驅動失敗回報，
  以及相機 DMA 暫停、板載振盪器判斷和 XCLK 錯誤回報。
- 與 Git 原始版本比較，原有 audio／camera 實作、Wi-Fi、儲存、health 與上傳 header 均保留。
- 尚未刷入硬體；畫面完整性、錄音品質、實際吞吐量與 runtime heap 峰值需板上確認。
