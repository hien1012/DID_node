# Audio / Camera 架構圖

此圖依照目前 `esp_http_client` 程式碼產生，包含：

- Audio：錄音 5 分鐘，每 10 分鐘一輪。
- Camera：錄影 10 秒，每 5 分鐘一輪。
- SD 卡 `undone` / `done` 狀態。
- Wi-Fi 中斷、server 無 ACK 與 heartbeat 失敗行為。
- 開機校時及 backlog 阻塞條件。
- Flask server 的檔案分類、SQLite 紀錄與 ACK。

## 重新輸出圖檔

需要安裝 Graphviz，然後在此目錄執行：

```powershell
dot -Tsvg audio_camera_architecture.dot -o audio_camera_architecture.svg
dot -Tpng -Gdpi=180 audio_camera_architecture.dot -o audio_camera_architecture.png
dot -Tpdf audio_camera_architecture.dot -o audio_camera_architecture.pdf
```
