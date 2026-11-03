# Heartbeat 格式（v1）

節點每 60 秒 `POST /heartbeat`（`Content-Type: application/json`）。server 回 200 即為成功。
實作：`main/node_health.c`；server 端：`server_mix.py` 的 `/heartbeat`、`/devices`。

```json
{
  "v": 1,
  "device_id": "node-A1B2C3D4E5F6",
  "time": 1760000400,
  "uptime_s": 3600,
  "state": "idle",
  "reset_reason": "power_on",
  "wifi":    { "rssi": -58, "disconnects": 2, "restarts": 0, "last_reason": 201, "heartbeat_fail_streak": 0 },
  "storage": { "free_mb": 28000, "pending_files": 0, "broken_files": 1 },
  "memory":  { "heap_free_kb": 180, "psram_free_kb": 5600 },
  "last_recording": { "at": 1760000400, "camera": "ok", "camera_fault": "none", "frames": 100, "dropped": 0, "mic": "ok", "samples": 320000 },
  "last_fault": "none",
  "faults": { "frame_drop": 1, "fb_incomplete": 0, "frame_gap": 0, "capture": 0, "storage": 0, "other": 0 },
  "last_upload":    { "at": 1760000520, "uploaded": 2, "failed": 0, "rejected": 0, "deferred": false }
}
```

| 欄位 | 意義 |
|---|---|
| `v` | 格式版本；欄位有不相容變更時加 1 |
| `time` | 節點的 Unix 時間（秒），可和 server 時間比對時鐘偏差 |
| `uptime_s` | 開機至今秒數；變小代表重開機過 |
| `state` | `booting` / `idle` / `recording` / `uploading` |
| `reset_reason` | 上次重開原因：`power_on` / `software` / `panic` / `watchdog` / `brownout` / `other`。出現 `panic`、`watchdog` 代表韌體當機過 |
| `wifi.rssi` | 訊號強度 dBm（-127 = 未連線）；低於約 -75 上傳會明顯變慢 |
| `wifi.disconnects` / `restarts` | 開機至今斷線次數 / Wi-Fi driver 重啟次數 |
| `wifi.last_reason` | 最後一次斷線原因碼（`wifi_err_reason_t`，例如 201 = 找不到 AP） |
| `wifi.heartbeat_fail_streak` | 連續幾次 heartbeat 沒有收到任何 HTTP 回應 |
| `storage.pending_files` | `done/` 中等待上傳的檔案數；持續增加代表上傳跟不上或斷線 |
| `storage.broken_files` | `broken/` 中的檔案數：損壞的 WAV 與被 server 拒收的檔案。損壞的影片直接刪除，不算在這裡（看 `faults`） |
| `last_recording` | 最近一次錄製；`camera` / `mic` 為 `ok`、`broken` 或 `disabled`（開機初始化失敗）。`camera_fault` 是該次相機失敗類別（成功為 `none`）。尚未錄製時為 `null` |
| `last_fault` | 最近一次相機錄影的失敗類別，成功為 `none` |
| `faults` | 開機以來相機錄影失敗次數，依類別累計、只增不減（重開機歸零，看 `uptime_s`）。類別依序判定、取第一個符合的：`capture`（取幀逾時 / 時戳錯誤 / 影格大小不對）→ `storage`（建檔、預配置、寫入、搬檔失敗）→ `frame_drop`（佇列滿丟幀，SD 卡頓）→ `fb_incomplete`（時戳斷層或幀數不足，且驅動有長度不符的幀）/ `frame_gap`（同上但驅動沒有）→ `other` |
| `last_upload` | 最近一次上傳；`deferred` = 為了趕下一次錄製而延後；尚未上傳時為 `null` |

## Wi-Fi 斷線檢查與應對

| 層級 | 觸發條件 | 動作 |
|---|---|---|
| 1. 斷線重連 | Wi-Fi driver 回報斷線 | `wifi_manager` 自動重連，間隔 1 s → 2 s → … 最多 30 s，連上後重置 |
| 2. 連線假死 | 已連上 AP，但連續 3 次 heartbeat 完全沒有 HTTP 回應 | 主動斷開並重新連線。server 有回應（即使是錯誤碼）不計入 |
| 3. 長時間離線 | 離線超過 10 分鐘 | 重啟 Wi-Fi driver，仍離線則每 10 分鐘再重啟一次 |

- 錄製**不受** Wi-Fi 影響：照常錄製，檔案留在 `done/`，連線恢復後的下一次上傳會依時間順序補傳。
- **錄製與上傳期間暫停 heartbeat**：不送出、不讀 SD、也不執行第 2、3 層動作；回到 idle 後的下一個週期恢復。
  因此 `state` 實際上只會看到 `booting` / `idle`；錄製加上傳最長約 9 分鐘沒有 heartbeat。
- server 端：`server_mix.py` 超過 300 秒沒收到 heartbeat **也沒收到上傳**，會印出 `WARNING <device> offline`，恢復時印出 `back online`。
  `GET /devices` 的 `online` 欄位只看 heartbeat（同樣 300 秒）。300 秒是為了涵蓋暫停 heartbeat 的錄製 + 上傳期間。
- 參數在 `main/app_config.h`：`APP_WIFI_RECONNECT_*`、`APP_HEARTBEAT_FAILS_BEFORE_RECONNECT`、`APP_WIFI_RESTART_AFTER_OFFLINE_S`。
