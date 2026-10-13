# SD 卡寫入問題 debug 紀錄

硬體：ESP32-S3-DevKitC-1 v1.0，SDMMC 1-bit（CLK 40、CMD 38、D0 39），杜邦線接 SD 轉接模組。
模組上有 8 腳「1002」元件：應為 10 kΩ 排阻（上拉），**沒有穩壓 IC、沒有電平轉換晶片** → VCC 只能接 3.3V。

錯誤碼對照：`0x107` = `ESP_ERR_TIMEOUT`（SD 卡對指令沒有回應）；`errno=5` = I/O error。

## 狀態摘要（最後更新 2026-10-13）

**目前完整流程正常**（#34）：開機 FPS 測試 PASS → 錄影 50/50 幀（10 s）＋錄音 160000/160000 → 上傳 RAW 15.36 MB 約 7.2 s（2.1 MB/s）＋WAV，皆 ACK。SD 40 MHz、卡 USDU1。

**10/12–10/13 已解決：錄影影像損壞**（#26–#33）。現象：水平條紋、整片灰（取到 U/V）、畫面錯位，越錄越嚴重。
**根因：mic I2S BCLK 放在 GPIO14，緊鄰相機 PCLK（GPIO13）**，約 1 MHz 的 BCLK 耦合進 PCLK 杜邦線（拔掉 mic 線也一樣，因為 ESP 仍輸出 BCLK）。
解法：`MIC_PIN_CLK` 改 **GPIO47**（右側排針）。規則：時脈類訊號不要放在相機排針那一側；GPIO14 不放時脈。
排查過程中「寫 SD 干擾」「D6 線」「接地」（#26–#30）都是誤判 —— 串流測試乾淨只是因為那時 I2S 停止。

**debug 工具（這次新增）**
- `main/cam_stream.c` + `tools/cam_stream_view.py`：`APP_CAM_STREAM_ENABLE=1` 時不錄影，拍一幀→停 XCLK→TCP（port 5002）送 PC，逐幀算 row_jump／col_jump；`APP_CAM_STREAM_SD_LOAD=1` 時每 20 幀切換背景寫 SD 做 A/B。
- `APP_MIC_ENABLE=0`：完全不初始化 mic（只錄影）。
- `tools/raw_view.py --stats`：RAW 逐幀統計；正常幀 row_jump ≤ col_jump，損壞幀 row_jump 明顯較大（或 mean≈128 且 ratio 很低 = 整幀 U/V）。

**待處理**
1. **內部 RAM**：錄影期間最低只剩約 2.7 KB（`HEAP [before upload] min=2783`；掛 SD 後還有約 15 KB）→ Wi-Fi／lwIP／SD 配置可能失敗；10/13 曾出現上傳讀 SD `0x107`（#30 那輪）。
2. 開機偶發 `Warm-up timed out after 0/10`（#30）：換 BCLK 腳位後未再出現，原因未證實，持續觀察。
3. 正式使用前把 `APP_NET_BENCH_ENABLE` 改 0（目前每次開機多約 20 s；XCLK 開著的兩次測速會因相機匯流排干擾失敗，屬預期）。

（10/11 的 SD 寫入問題：換卡＋SD VCC 改接另一支 3V3＋短跳線＋40 MHz 後可運作，見 #1–#25；當時同時改了多個條件，未單獨確認是哪一個起作用。）

## 時間軸

| # | 日期 | 條件 | 現象 | 判斷 / 動作 |
|---|---|---|---|---|
| 1 | 10/10 | console = 原生 USB | COM 口只看到開機到 `Starting app cpu` 就重開 | log 送到 JTAG 口，看不到。console 改 UART0（COM 口） |
| 2 | 10/10 | 卡 SB16G、40 MHz | 64 s 時讀取 `0x107`，取剩餘空間失敗 | 只有片段 log |
| 3 | 10/10 | SB16G、**20 MHz**，先掛 SD 再初始化相機 | 1214 ms 掛載正常；相機上電（PWDN 拉低、XCLK 啟動）後 4154 ms 起**所有** SD 讀取逾時，不再恢復 | 不是時脈問題；像是相機上電時電源擾動把卡重置。改成**相機初始化後才掛載 SD** |
| 4 | 10/10 | SB16G、40 MHz，相機先初始化 | 掛載、預配置 92 MB、heartbeat 讀取都正常；錄製開始後第 0 幀寫入 `0x107`，之後所有操作（含單一 task 的 close/rename）都失敗 | **不是 task 競爭**：FATFS 與 sdmmc driver 都有 mutex；`0x107` 是卡不回應；錄製結束只剩一個 task 時仍失敗。觸發點是第一次大量連續寫入 |
| 5 | 10/10 | 縮短跳線 | 寫入 85 幀（約 17 s、26 MB）後 `0x107` | 只改線長就從 0 幀進步到 85 幀 → 實體連接（電源／地線／訊號）有關 |
| 6 | 10/10 | — | SD 模組上的 LED 錄製時亮度會變 | 若為電源指示燈 → 模組端 3.3V 在掉 |
| 7 | 10/11 | SD VCC 改接板上另一支 3V3 腳 | 可以運作 | 兩支 3V3 是同一顆 LDO；改善應來自不再和相機共用同一段線與接點 |
| 8 | 10/11 | **換卡 USDU1**、20 MHz | 3 輪皆 300/300 幀、0 掉幀；mic 960000/960000（overflow 0–2，已忽略）；`sd_frame_max` 約 223 ms、`peak_ready=1/5` | 錄製可用，但單幀最慢寫入 > 200 ms 週期，餘量小 |
| 9 | 10/11 | USDU1，上傳 1.9 MB WAV 的計時 | 20 MHz：讀 SD 1127 kB/s（22% 時間）、Wi-Fi 送出 317 kB/s（77%）。40 MHz：讀 SD 1372 kB/s（16%）、Wi-Fi 265 kB/s（82%）。單次最慢：讀 13–16 ms/16KB、送 149–319 ms/16KB | **上傳瓶頸是 Wi-Fi 送出，不是 SD 讀取**；SD 讀取穩定無卡頓。上傳速度另案處理 |
| 10 | 10/11 | USDU1，錄製寫入速度（staging 8 KB） | **40 MHz**：avg 122 ms/幀（2500 kB/s）、max 160 ms、slow 0/300，無錯誤。**20 MHz**：avg 184 ms/幀（1668 kB/s）、max 229 ms、slow 82/300 | 20 MHz 平均餘量僅約 8%，靠佇列硬撐；**40 MHz 餘量約 39%，建議使用**。待確認 40 MHz 長時間不出 `0x107` |
| 11 | 10/11 | ESP-IDF iperf 範例（ESP32-S3 TCP TX → PC iperf2；PC 連同一台 AP 的 5 GHz） | 30 秒平均 59.8 Mbps（ESP 端）／62.6 Mbps（PC 端），約 7.5 MB/s，穩定 | **RF／AP／PC 網路都沒問題**。我們的上傳只有約 2.3 Mbps（慢約 25 倍）→ 瓶頸在 HTTP 上傳路徑：PC 端 Flask 接收，或 ESP 韌體。下一步：PC 本機 POST 測 Flask 接收速度 |
| 12 | 10/11 | `tools/bench_upload.py` 對本機 server_mix（暫存資料夾、port 5099）上傳 92 MB × 2 | 0.36–0.48 s，約 190–260 MB/s | Flask 本身收得很快 |
| 13 | 10/11 | `bench_upload.py` 對**真實** server_mix（port 5000、真實資料夾）上傳 92 MB | 0.70 s，約 132 MB/s | **PC 端排除**。iperf 7.5 MB/s + server 132 MB/s → 上傳瓶頸在 ESP32 韌體。下一步：韌體內直接用 socket 送資料給 iperf2（不經 HTTP） |
| 14 | 10/11 | `net_bench` 第一版 | `[after_init] Cannot allocate 16384 bytes`：相機／mic／SD 初始化後，內部 RAM 連一塊連續 16 KB 都沒有 | **內部 RAM 嚴重不足**，上傳時 lwIP／Wi-Fi 緩衝都要從內部 RAM 配置 → 列為上傳慢的頭號嫌疑。加上各階段 `HEAP [...]` log 找出誰吃掉內部 RAM；測試緩衝改為不足時用 PSRAM |
| 15 | 10/11 | `net_bench` 第二版，PC 端 iperf2 紀錄 | 連線 [1]：收到 858 KB 後 `tcp recv failed: Connection timed out`（ESP 端無預警消失）。連線 [2]：8 MB 完整，**35.4 Mbps ≈ 4.4 MB/s** | 本韌體的 Wi-Fi／lwIP 直接用 socket 可達 4.4 MB/s，比 HTTP 上傳快約 15 倍 → 瓶頸偏向 HTTP 上傳寫法或上傳時的記憶體狀況。[1] 中斷原因與 [1]/[2] 對應哪次測試，待 ESP 端 log 確認 |
| 16 | 10/11 | `net_bench` 第二版，ESP 端 log 對照 | `wifi connected`：internal free 112 KB / largest 31.7 KB → `[boot]` **4894 kB/s**、max 9 ms。`camera init`：free **33.8 KB** / largest 14.8 KB。`sd mounted`：free 40.9 KB → `[after_init]` **71 kB/s**、max 565 ms、送 272 KB 後 `errno=104`（連線被重置）。開機以來 internal 最低 15.9 KB | **根本原因確定：相機初始化後內部 RAM 不足，lwIP 配不到封包緩衝** → 停停走走甚至斷線。PC 端的 [1][3][5] 都是 `after_init`。boot 測試傳送時約用掉 79 KB 內部 RAM（16 KB 緩衝 + 64 KB TCP 傳送佇列） |
| 17 | 10/11 | 開啟 `SPIRAM_TRY_ALLOCATE_WIFI_LWIP` | log `WiFi/LWIP prefer SPIRAM`。`[boot]` 4551 kB/s，測速前後 internal 175 KB 不變（只用約 17 KB）。`camera init` 後 internal **80.7 KB**（之前 33.8）、largest 31.7 KB；**SD 暫存區恢復 32 KB**。`[after_init]` 仍只有 **204 kB/s**、max 408 ms、送 1.69 MB 後 `errno=104`，期間 internal 最低 **63.8 KB** | 記憶體不足已解決，但 `after_init` 仍慢 → **還有第二個原因**。新嫌疑：相機初始化後 XCLK 16 MHz（GPIO15）與 PCLK/資料線在杜邦線上輻射，干擾 2.4 GHz 收訊（收不到 TCP ACK → 重傳到上限 → 重置）。待測：相機線遠離天線；或韌體 A/B（XCLK 開／關） |
| 18 | 10/11 | 相機模組與杜邦線移離天線 | `[after_init]` **645 kB/s**、8 MB 完整送完、max 143 ms、slow 2，內部 RAM 最低 63.8 KB（同前）。暖機時 Wi-Fi 剛好在連線，出現 2 幀 `FB-SIZE: 0`、暖機 4.0 fps（之後 FPS 測試正常）。`[boot]` 因 Wi-Fi 尚未連上而失敗（errno=118） | **干擾確認存在**：只移位置就快 3.2 倍且不再斷線；但仍只有 boot 的約 1/7。Wi-Fi 與相機彼此干擾（雙向）。發現 bug：軟體重開後系統時間保留，開機跳過了等 Wi-Fi 與校時 → 已修正 |
| 19 | 10/11 | XCLK A/B（同一次開機、同一擺放） | `[boot]` 4592 kB/s；`[after_init]`（相機匯流排運作）**406 kB/s**；`[xclk off]` **3851 kB/s**、max 12 ms、slow 0；`[xclk on again]` **476 kB/s**。開機 Wi-Fi 等待與校時修正已生效 | **確定干擾源是相機匯流排**（XCLK 16 MHz 與其產生的 PCLK／資料線）：停掉就快約 9 倍且可重現。這也是 HTTP 上傳只有 0.3 MB/s 的主因。解法候選：不錄影時停 XCLK（上傳時不受干擾）、降低 GPIO15 驅動強度、硬體走線改善 |
| 20 | 10/11 | 非錄製期間停 XCLK + I2S 時脈（40 MHz、staging 32 KB） | `[boot]` 3740；`[after_init]`（XCLK 開、I2S 已停）**717 kB/s**；`[xclk off]` **3938**；`[xclk on again]` 709。錄製 300/300、0 掉幀、**SD 寫入 avg 96 ms/幀（3179 kB/s）**、max 138 ms、slow 0；mic 960000、overflow 5。上傳 92 MB RAW：**wifi_send 4927 kB/s**（原約 300）、**sd_read 1296 kB/s 佔 79%**，總計 90 s；WAV 同樣 wifi 5269 / sd 1364 kB/s | **停時脈有效**：XCLK 停止後恢復錄影正常、上傳 Wi-Fi 快約 16 倍。I2S 停止讓「XCLK 開」的情況從約 440 → 710 kB/s，mic 時脈也有少量干擾。**新瓶頸是 SD 讀取**：`fread` 走 stdio 預設 128 B 緩衝，16 KB 被拆成 128 次小讀取、FATFS 逐 sector 讀 → 已改為無緩衝（`setvbuf _IONBF`） |
| 21 | 10/11 | `setvbuf _IONBF` 試驗 | WAV sd_read **40 kB/s**、max 404 ms/16KB | 更慢 → 撤回。推測：S3 的 newlib 在 ROM（以 size 優化編譯），無緩衝 FILE 的 fread 每次只 refill 1 byte → 每 16 KB 變成 16384 次 VFS 呼叫（約 25 µs/次，與 40 kB/s 吻合） |
| 22 | 10/11 | 撤回後（有緩衝 `fread`）、40 MHz | `[boot]` 4591；`[after_init]` 774；`[xclk off]` 4593；`[xclk on again]` 590。錄製 300/300、SD 寫 avg 96 ms/幀（3176 kB/s）、mic overflow 0。上傳 92 MB：total **104 s** = sd_read 85.8 s（**1073 kB/s**，max 17 ms/16KB）+ wifi_send 18.0 s（5113 kB/s）；WAV sd 1065 / wifi 5302 kB/s | **目前瓶頸 = SD 讀取（83% 時間）**。每 16 KB 平均約 15 ms、最大 17 ms → 不是卡頓，是每次讀取的固定開銷：stdio 128 B 緩衝 → FATFS 逐 sector（CMD17，單 sector 約 470 µs）。寫入走 `write()` 直接多 sector，所以有 3.2 MB/s。下一步候選：上傳改用 `open()/read()`（同寫入路徑）；再進一步讀／送重疊（雙緩衝） |
| 23 | 10/11 | 上傳改 `open()/read()` | 92 MB RAW：total **42.9 s（2146 kB/s）** = sd_read 23.1 s（**3994 kB/s**，max 5 ms/16KB）+ wifi_send 19.8 s（4662 kB/s）+ ack 0.04 s；WAV 0.95 s（2008 kB/s，sd 3975 / wifi 4962）。整輪上傳 94 MB 約 44 s。錄製 300/300、SD 寫 avg 93 ms/幀（3288 kB/s）；mic overflow 3 | **有效**：SD 讀取 1.07 → 4.0 MB/s（3.7 倍），上傳總時間 104 → 43 s（2.4 倍）。讀與送現在約 54% / 46%，兩者輪流執行；若改為重疊（雙緩衝 + 讀檔 task），理論可降到約 23 s（約 4 MB/s） |
| 24 | 10/11 | 讀 SD／送出重疊（`upload_reader` task） | 92 MB：total **40.7 s（2265 kB/s）**，sd_read 24.1 s（3829 kB/s）與 wifi_send 38.2 s（2411 kB/s）重疊（相加 62 s），wait_sd 2.2 s。**但這次 AP 在 channel 11、BW20**（之前 channel 1、HT40）：`[boot]` 只有 2275 kB/s（之前約 4500）。副作用：static 多約 20 KB → app start internal 239 → 218 KB，相機 32 KB 暫存區配置失敗退回 **16 KB**：SD 寫 avg 101 ms/幀、max 206 ms、slow 1/300；錄製中 internal 最低 41 KB（之前 61 KB） | **重疊有效**：total ≈ wifi_send + 2 s，瓶頸已完全是 Wi-Fi，SD 讀取不再拖慢。這輪 Wi-Fi 慢是 AP 換到 20 MHz 頻寬（環境因素）；HT40 時預估約 4 MB/s。待處理：相機 SD 暫存區被擠成 16 KB → 可讓相機暫存區與上傳緩衝共用同一塊 32 KB（錄製與上傳不會同時進行） |
| 25 | 10/11 | 共用 32 KB I/O 緩衝（仍是 channel 11、BW20） | `camera init` internal free **100.6 KB**（#24：76 KB）；`sd mounted` 99.4 KB；錄製中最低 **61.9 KB**（#24：41 KB）。staging 32768、SD 寫 avg 97 ms/幀、max 139 ms、slow 0。無 `I/O buffer … held by` 錯誤。上傳 92 MB **37.1 s（2492 kB/s）**，wifi_send 2653 kB/s、sd_read 3797 kB/s、wait_sd 2.0 s。XCLK 開著的兩次測速都 `errno=104`（65 KB、180 KB 後被重置），XCLK 關 2466 kB/s。這輪 `saved=301`（first +14 ms） | **共用緩衝成功**，記憶體與寫入速度都回到最佳。BW20 下相機匯流排干擾更致命（直接斷線），但正式流程上傳時 XCLK 已停，不受影響。301 幀：第一幀距觸發點僅 14 ms，60 s 視窗（含邊界）可放進 301 幀 → 檔案比預配置多 1 幀 |
| 26 | 10/12 | PC 端解碼檢查（`tools/raw_view.py`，40 個已上傳 RAW） | 解碼方式正確：`211824.raw` 300 幀全部乾淨。但多數檔案影像損壞：最新 `001739.raw` 只有第 0 幀乾淨，之後每幀**上方約 1/3–1/2** 一行行錯位、出現整片灰（≈128 = 取到 U/V 而非 Y）、畫面左右錯開；下方正常但常有水平位移。損壞資料是新的（不是殘留舊影格）。各檔乾淨比例差很大（0/30 到 30/30） | **是擷取時的資料錯誤，不是解碼問題**。特徵是位元組錯位（PCLK 多／少取樣 → Y/UV 相位翻轉、整行位移）。損壞集中在每幀前段 ≈ 上一幀寫 SD 的那約 97 ms，第 0 幀（還沒寫 SD）乾淨 → 懷疑 SD 寫入時的電氣干擾（40 MHz SD CLK／電流尖峰耦合到 PCLK 杜邦線）。不同次接線結果差很多也支持接線因素。待測：SD 降 20 MHz、SD 線與相機線分開並給 PCLK 旁加地線 |
| 27 | 10/13 | LA 量 PCLK + 串流模式（`cam_stream.c`：拍一幀→停 XCLK→TCP 送 PC，不寫 SD） | LA：每行 PCLK 1280 個（640×2，數量正確）。串流 70+ 幀畫面完全正常（人臉清楚、無條紋），ratio（row_jump／col_jump）約 0.94–0.98。同一套接線的錄影檔則整段出現水平條紋 | **感測器、杜邦線、PCLK 取樣、驅動 GRAY8 取 Y、PC 解碼都正常**；損壞只發生在錄影路徑。串流與錄影的差別：錄影時同時寫 SD（SD 電氣干擾／PSRAM 與 DMA 頻寬競爭）、相機連續跑 5 fps（cam_task 須跟上每個 DMA 半緩衝事件）、錄影中有 heartbeat。待測：錄影但不寫 SD（區分「寫 SD」與「連續擷取」） |
| 28 | 10/13 | 串流長時間（522 幀） | 7/522（1.3%）SUSPECT，集中成一陣：frame 520（ratio 1.81）、521（mean 126、col_jump 12.1、ratio 0.36 = 整幀幾乎都是 U/V，ratio 指標漏判）、522（ratio 3.50）、523（3.38）；frame 522→523 之間連續 4 次 `FB-OVF len=307200 cnt=40 event_q=0`（一幀收滿 480 行後、VSYNC 前又收滿第 41 個 15360 B 區塊），該循環多花約 800 ms（每次溢位約多 1 幀）。約 20 次循環才一陣 | **串流（不寫 SD）也會壞，且條紋與 FB-OVF 同時成陣出現 → 同一機制**：PCLK 多出邊緣（干擾）→ 少量多出 = Y/UV 錯位（條紋），大量多出（> 15360 B/幀）= FB-OVF 丟幀。`event_q=0` → 不是 cam_task 落後。LA 每行 1280 個是乾淨時段且 LA 取樣率看不到 ns 級突波。串流時干擾源推測為 Wi-Fi 發射（send 返回時 lwIP 仍在送尾段／重傳），錄影時為持續的 SD 寫入 → 錄影損壞比例遠高於串流。根本對策：PCLK 線路（串聯電阻、縮短、旁邊 GND、遠離 SD 線與天線） |
| 29 | 10/13 | 錄影 `2026_10_13_204916.raw`（50 幀，靜止場景 0–28、另一場景 30–49）逐像素分類：以時間中位數為參考，看壞像素能否用「單一資料位元翻轉」解釋 | 錄影中**完全沒有 FB-OVF**，但影像嚴重損壞。幀 0–28（較亮場景）壞像素 14–45%：**D6 單一位元翻轉解釋 25.4%**，D5 1.9%、D7 2.0%（D6 是相鄰位元的 12 倍，兩個方向都有）；接近 128（色度）只有 13%。幀 40–47（暗場景）：接近 128 佔 54.6%（位元組錯位），D6 仍 14.7%。幀 30／33／49 有大片灰色 + 位移（錯位延續到幀尾）。參考影像本身也受損（ref row_jump 10.1），比例為低估 | **錄影時有兩種錯誤**：(1) **D6（GPIO17）資料線錯誤**，是亮場景條紋的主因（單線問題：接觸不良或與其他線耦合）；(2) 位元組相位錯位（PCLK）。串流時 D6 沒有出錯 → 很可能是寫 SD 時才被干擾。對策：先換／重插 D6 杜邦線、與 SD 線分開；之後再看 PCLK |
| 30 | 10/13 | 換 D6 杜邦線後錄影 `2026_10_13_210317.raw`（SD 40 MHz；#29 為 20 MHz），同樣分析幀 0–15（靜止） | 16 幀中約 42% 像素損壞（比 #29 更糟）。D6 解釋 15.0%（翻成 0 佔 80%）、**D7 11.0%（翻成 1 佔 90%）**、D5 1.8%、接近 128 **38.7%**。幀 46／49 有整片灰色帶。另外開機偶發 `Warm-up timed out after 0/10 complete frames`（`cam_hal: Failed to get frame: timeout`），reset 後又正常 | **換 D6 沒有改善，錯誤擴散到 D6、D7 與位元組錯位** → 不是單一條線，而是整組訊號共同的問題：相機與 ESP 之間的 **GND／電源品質**（寫 SD 的電流尖峰造成地彈，使資料線判讀與 PCLK 同時出錯；啟動偶發完全收不到幀也符合供電／接地不穩）。40 MHz 比 20 MHz 更糟也符合。對策：相機與 ESP 之間加多條短 GND、相機模組與 SD 模組電源端加去耦電容、SD 降回 20 MHz |
| 31 | 10/13 | 串流 + SD 負載 A/B（`APP_CAM_STREAM_SD_LOAD`，SD 40 MHz） | 第 0–19 幀不寫 SD：0/20 可疑；第 20–33 幀寫 SD：**0/14 可疑**，ratio 0.91–1.05，畫面清楚。寫 SD 的幀 dt 從約 1090 增加到 1150–1220 ms（負載有在運作） | **單純寫 SD（同樣節奏、同樣緩衝、Core 0）不會弄壞影像** → #30 的「SD 電氣干擾」推論不成立。錄影與此測試剩下的差別：(1) **mic I2S 時脈在運作**（BCLK GPIO14 與 PCLK GPIO13 在排針上相鄰，杜邦線並排）＋ mic_writer；(2) 相機連續運作 10 s 以上（此測試每次只運作約 1 s），且錄影損壞會隨時間變嚴重；(3) 零拷貝 writer 直接從 fb 寫出。下一步：拔掉 mic 線錄影 |
| 32 | 10/13 | (a) 拔掉 mic 杜邦線錄影；(b) `APP_MIC_ENABLE=0`（I2S 完全不初始化）錄影，SD 40 MHz | (a) 仍嚴重損壞（與 #30 相同）。(b) **50 幀全部正常**，手部動作清楚、無條紋、無灰色帶 | **根因在 ESP 端的 I2S（mic）運作**，與 SD 寫入、PCLK 線路、mic 杜邦線無關。拔線無效 → 不是 mic 線之間的耦合。相機驅動以 `gdma_new_channel()` 取得獨立 GDMA 通道，不會與 I2S 共用。剩下兩種可能：(1) **GPIO14（I2S BCLK，約 1 MHz）在排針／板上緊鄰 GPIO13（PCLK）**，即使沒接線，ESP 仍輸出 BCLK，耦合到旁邊插著的 PCLK 杜邦線；(2) I2S 周邊／mic task 在晶片內部的影響。區分方法：把 `MIC_PIN_CLK` 從 14 移到遠離相機的 GPIO47（右側排針），I2S 照常運作再錄 |
| 33 | 10/13 | `MIC_PIN_CLK` 14 → **47**，`APP_MIC_ENABLE=1`（I2S 照常運作），錄影 `2026_10_13_213852.raw` | **50/50 幀正常**：row_jump 1.3–1.9 < col_jump 1.8–2.6（每幀），無條紋、無灰色帶、手部動作清楚；靜止段 diff_prev 0.5 | ✅ **根因確定：GPIO14（I2S BCLK，約 1 MHz）緊鄰 GPIO13（PCLK）**，BCLK 耦合進 PCLK 杜邦線 → PCLK 多／少邊緣（位元組錯位）與資料位元錯誤。#26–#30 的「寫 SD 干擾」「D6 線」「接地」推論都是誤判：串流模式乾淨是因為 I2S 停止。規則：時脈類訊號（I2S BCLK／WS、SD CLK）不要放在相機排針那一側 |
| 34 | 10/13 | mic BCLK 實際接到 GPIO47，完整流程（開機 → 錄影 → 上傳） | 錄音、錄影都正常。開機 warm-up、FPS 測試 PASS（50/50、5.000 fps）；錄影 50/50、dropped 0、SD avg 97 ms/幀；上傳 RAW 15.36 MB 7.2 s（2125 kB/s）、WAV 正常 ACK。換腳位後尚未再出現 warm-up 逾時。內部 RAM：`sd mounted` min 15179 → 錄影後 `before upload` **min 2783** | 影像問題已解決。warm-up 逾時（#30）是否與 BCLK 有關**未證實**：開機時 I2S 時脈是停止的（`clock stopped until recording`），持續觀察。**待查：錄影期間內部 RAM 最低只剩約 2.7 KB**（之前約 15 KB），有 Wi-Fi／lwIP／SD 配置失敗風險 |


### 變更紀錄（待上板驗證）

| 日期 | 變更 | 目的 | 驗證方式 |
|---|---|---|---|
| 10/13 | `APP_MIC_ENABLE`（暫設 0）：`app_main` 不呼叫 `mic_record_init()`，I2S 完全不啟動、不產生 WAV，只錄影並上傳 RAW | 拔 mic 線後錄影仍損壞；排除 ESP 端 I2S 時脈（GPIO14 緊鄰 PCLK GPIO13）與 mic_writer 寫 SD | 開機 log `Microphone disabled`；錄影後不應有 `MIC_RECORD`／`I2S clock` log；`raw_view.py --stats` 與 bitcheck 比較 #30 |
| 10/13 | 串流加 SD 負載 A/B（`APP_CAM_STREAM_SD_LOAD`）：背景 task（Core 0、prio 5、共用 32 KB I/O 緩衝）每 200 ms 寫 307200 B 亂數到 `/sdcard/stream_load.bin`，每 20 幀切換開／關；負載開著擷取的幀 magic 為 `CAMS`，viewer 分開統計 | #30：錄影檔在 SD 上就已損壞且越錄越糟，但錄影統計正常（dropped 0、SD avg 98 ms）。同一次開機、同一套接線下區分「寫 SD 的電氣干擾」與「錄影流程本身」 | viewer 結尾 `no SD x/N, SD y/M`：只有 SD 組壞 → 電氣（接地／電源）；兩組都乾淨 → 錄影流程 |
| 10/13 | 新增 `cam_stream.c` + `tools/cam_stream_view.py`：`APP_CAM_STREAM_ENABLE=1` 時開機測速後不進錄影排程，每擷取一幀直接從 frame buffer 用 TCP（port 5002）送 PC，不經 SD／HTTP。PC 端逐幀算 row_jump／col_jump | #26 影像損壞；LA 確認每行 PCLK 1280 個（數量正確）→ 區分「擷取當下就壞」與「錄影／寫 SD 路徑造成」 | PC 端 `suspect` 比例：串流也壞 → 擷取問題（取樣時序／資料線）；串流乾淨 → 錄影路徑（寫 SD 干擾） |
| 10/11 | Wi-Fi／lwIP RX 縮小：static RX 16→8、dynamic RX 64→16、RX BA 窗口 32→6、TCP_WND 65534→16384（TX 不變） | 釋放內部 RAM（上傳只收 ACK） | 開機 log `wifi:Init static rx buffer num: 8`；`SD DMA staging buffer` 是否從 8192 變大；`Upload timing` 的 wifi_send 是否變差 |
| 10/11 | 新增 `net_bench.c`：開機時用 socket 從 RAM 送 8 MB 給 PC iperf2（`boot`：相機初始化前；`after_init`：全部初始化後）。`APP_NET_BENCH_ENABLE` 控制 | 判斷 ESP 端瓶頸是 HTTP 上傳路徑，還是 Wi-Fi／lwIP 設定與記憶體 | log `NET_BENCH: [boot] … kB/s`、`[after_init] …`，對照 iperf 7.5 MB/s 與 HTTP 0.3 MB/s |
| 10/11 | ✅ 已驗證（#25）共用 32 KB I/O 緩衝：`node_storage_io_buffer_acquire/release()`（static、4-byte 對齊、owner 檢查）。相機在 `cam_record_prepare` 取得、`finish` 歸還（不再 `heap_caps_aligned_alloc`）；上傳每個檔案取得，切成 2 × 16 KB | #24 副作用：相機暫存區被擠成 16 KB、錄製中 internal 最低 41 KB | `Zero-copy recording ready … SD staging=32768 bytes shared I/O buffer`；`SD write speed … staging=32768`、slow 0；`HEAP [camera init]` free 應約 92 KB（#24 為 76 KB）；不應出現 `I/O buffer requested by … while held by …` |
| 10/11 | ✅ 已驗證（#24）`node_upload.c` 讀 SD／送出重疊：常駐 `upload_reader` task（Core 1、prio 2、static）讀進 2 × 16 KB 緩衝，main 送出；`s_free_queue`／`s_full_queue` 傳緩衝 index，結束訊息 length ≤ 0 | #23：讀與送輪流執行，總時間 = 兩者相加 | `Upload timing` 的 total 應接近 max(sd_read, wifi_send)；92 MB 約 23–25 s（約 4 MB/s）；`wait_sd` 小代表瓶頸在 Wi-Fi，大代表在 SD |
| 10/11 | ✅ 已驗證（#23）`node_upload.c` 上傳讀檔改 `open()` / `read()` / `close()`（不經 stdio） | 16 KB 一次直接讀進對齊的內部 RAM 緩衝，FATFS 走多 sector（CMD18），與錄製 `write()` 相同路徑（#22 瓶頸） | `Upload timing` 的 sd_read 應從約 1.1 MB/s 升到約 3 MB/s 以上；92 MB 總時間約 104 s → 約 50 s |
| 10/11 | ❌ 已撤回：`node_upload.c` `fopen` 後 `setvbuf(file, NULL, _IONBF, 0)` | 讓 16 KB 讀取直接進對齊緩衝、走多 sector 讀取（#20 sd_read 只有 1.3 MB/s） | 實測 WAV sd_read **40 kB/s**、max 404 ms/16KB（比原本慢約 33 倍），wifi_send 5472 kB/s 正常 → 改回原本有緩衝的 `fread` |
| 10/11 | ✅ 已驗證（#20）**非錄製期間停止時脈**：相機 XCLK 在 `rec_session` 錄完後停止、下次準備時恢復；mic 的 I2S 通道（BCLK/WS）在 `mic_record_prepare` 啟動、`finish` 停止；開機測速後也停止 | 上傳時不受相機匯流排／I2S 時脈干擾（#19） | `XCLK output stopped/restored`、`I2S clock started/stopped` log；`Upload timing` 的 wifi_send 應接近 3–4 MB/s；首輪錄製 `Recording finished` 300/300、`SD write speed` 正常 |
| 10/11 | `cam_record_set_xclk_output()` + 開機測速改為 `after_init` → `xclk off` → `xclk on again` | A/B 量化相機匯流排（XCLK 16 MHz + 由它產生的 PCLK／資料線）對 Wi-Fi 的干擾 | 比較三次 `NET_BENCH` 速度；`xclk off` 明顯較快 → 干擾源是相機 |
| 10/11 | `app_main.c` `synchronize_clock()`：開機一律等 Wi-Fi 並成功校時一次（不再因系統時間「看起來有效」而跳過） | 修正軟體重開後跳過 Wi-Fi 等待與校時的 bug | 開機 log 中 `WIFI: Connected` 與 `TIME_SYNC: Clock synchronized` 出現在 `HEAP [wifi connected]` 之前；`[boot]` 測速能連上 |
| 10/11 | `s_upload_buffer` 加 `WORD_ALIGNED_ATTR` | 保證 SD 讀取走 DMA 直讀，不退回 512 B 逐 sector | `Upload timing` 的 sd_read 速度 |

## 已排除

- 兩個寫入 task 互相競爭（見 #4）
- S3 腳位衝突：38/39/40 不是 strapping，未與相機／mic／LED 重複；39/40 的 JTAG 功能未啟用
- 模組上的電平轉換或穩壓 IC：模組上沒有（見最上方）

## 未確定 / 待驗證

1. **舊卡 SB16G 是否本身有問題？** 用目前的接線（#7）+ 20 MHz 再插舊卡錄一輪：
   - 失敗 → 卡的問題（品質或寫入電流大）
   - 成功 → 之前主要是接線／電源問題
2. **新卡 40 MHz 長時間穩定性**：單輪已通過（#10）。需連續跑數小時（≥ 10 輪）確認沒有 `0x107`。
3. **SD 暫存區只有 8 KB**（`SD DMA staging buffer: 8192 bytes`）：內部 RAM 不足，32 KB 配置失敗。每次只寫 8 KB 會降低寫入效率。可能需要縮小 Wi-Fi／lwIP 緩衝來釋放 RAM。
4. 板子很燙：原因清單見對話紀錄（LDO 耗散、Wi-Fi 全功率不省電、CPU 240 MHz 不降頻、相機持續運作）。

## 新增的 debug log（2026-10-11，尚未 build 驗證）

| log | 來源 | 看什麼 |
|---|---|---|
| `cam_record: SD write speed: avg=… ms/frame (… kB/s), max=… ms, slow(>200ms)=N/300, staging=… bytes` | `main/cam_record.c`，每輪錄製結束 | avg 要明顯 < 200 ms；slow 越少越好 |
| `UPLOAD: Upload timing: total=… \| sd_read=… ms (… kB/s, max …) \| wifi_send=… ms (… kB/s, max …) \| ack_wait=… ms` | `main/node_upload.c`，每個檔案上傳完 | sd_read 和 wifi_send 誰佔大部分時間，就是上傳瓶頸 |

## 每次測試請記錄

卡片型號、SD 時脈（20/40 MHz）、接線變更、`SD write speed` 那一行、`Recording finished` 那一行、有無 `0x107`。
