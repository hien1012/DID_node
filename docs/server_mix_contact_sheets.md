# RAW 自動總覽圖與 SQLite 檢視

## 啟動

在執行伺服器的 Python 環境安裝相依套件，然後重新啟動伺服器：

```powershell
python -m pip install -r requirements.txt
python server_mix.py
```

預設資料庫仍為 `server_mix_data/field_mix.db`。若設定了 `FIELD_MIX_DATA_ROOT`，
請到該目錄開啟 `field_mix.db`。
啟動時自動新增 `sheet_jobs` 資料表，不修改原有 `uploads`、`heartbeats` 欄位。

## 執行流程

RAW 接收、驗證及存檔成功後，在登錄 `uploads` 的同一筆交易內建立產圖工作，隨即回覆原有 ACK。
單一背景 worker 另外讀取 RAW、產生 PNG，再將結果寫回 `sheet_jobs`。
ACK 不等待 PNG 編碼；圖片依賴缺少或產圖失敗，也不會讓已成功保存的 RAW 變成上傳失敗。
音訊不建立產圖工作，原有 WAV 與 heartbeat 流程保留。

目前為單一伺服器行程搭配 Flask 多執行緒與一個產圖 worker。
若未來改成多個 WSGI worker 行程，應將產圖 worker 獨立成單一服務，再調整工作復原機制。

輸出與 RAW 放在同一目錄：

```text
<DATA_ROOT>/<device_id>/<YYYY-MM-DD>/
    YYYY_MM_DD_HHMMSS.raw
    YYYY_MM_DD_HHMMSS.sheet.png
```

每張總覽圖最多均勻取 20 幀，包含首尾；5 欄 × 4 列，每格 320×240，整張 1600×960。
標示原始影格索引。少於 20 幀時不重複抽取，剩餘格子保持黑色。
PNG 先寫入同目錄暫存檔，完整寫入後才取代正式檔名。

產圖只定位讀取選中的影格，不將整份 RAW 載入記憶體。
以 640×480 GRAY8 為例，每次原始影格讀取約 300 KiB，固定 BGR 畫布約 4.4 MiB；
另外仍有影格縮放、PNG 編碼及 Python/OpenCV 暫存，這些數字不代表 runtime 記憶體峰值。

## 工作狀態與復原

`sheet_jobs` 的主要欄位：

| 欄位 | 意義 |
|---|---|
| `capture_id` | 對應 `uploads.capture_id`，也是產圖工作唯一鍵 |
| `status` | `pending`、`processing`、`ready`、`failed` |
| `sheet_path` | 完成的 PNG 路徑 |
| `last_error` | 失敗原因 |
| `updated_at` | 最近更新時間 |

同一筆上傳重送不會建立重複工作。
伺服器啟動時會補上舊影像缺少的工作，並將中斷中或失敗的工作重新排隊一次。
失敗工作不在背景無限重試；修正套件或輸入問題後，重新啟動伺服器即可重試。
`GET /uploads` 也會附上 `sheet_status`、`sheet_path`、`sheet_error`。

## 用 UI 查看 SQLite

可使用 [DB Browser for SQLite](https://sqlitebrowser.org/dl/) 的 Windows 版本。

1. 開啟程式，選擇 **Open Database**。
2. 選取專案的 `server_mix_data/field_mix.db`。
3. 切換到 **Browse Data**，以資料表選單選擇 `uploads`、`heartbeats` 或 `sheet_jobs`。
4. 伺服器新增紀錄後重新整理資料。PNG 是磁碟檔案，位置在 `sheet_jobs.sheet_path`。

若要在 **Execute SQL** 同時查看上傳與產圖結果，可執行：

```sql
SELECT u.received_at, u.device_id, u.media_type,
       u.file_path, u.file_bytes,
       j.status AS sheet_status, j.sheet_path, j.last_error
FROM uploads AS u
LEFT JOIN sheet_jobs AS j ON j.capture_id = u.capture_id
ORDER BY u.received_at DESC;
```

## 命令列與驗證

原有手動指令仍可使用：

```powershell
python tools/raw_view.py FILE.raw --sheet sheet.png
python -m unittest discover -s tests -v
```

測試使用暫存資料庫與 RAW，不改動正式的 `server_mix_data`。
