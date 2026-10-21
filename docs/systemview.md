# SystemView（S3 內建 USB-JTAG）紀錄

目的：用 SEGGER SystemView 看錄製 / 上傳期間各 task（與 ISR）的排程與 CPU 競爭。
狀態（2026-10-21）：**能抓、但只在系統閒置時穩定**；Wi-Fi 大量傳輸或錄製一開始，OpenOCD 幾秒內就因資料損壞結束。
原因與證據見〈根本原因〉；下一步見最後一節。

## 結論摘要
- 開了 trace 的韌體本身正常：32 KB 相機 DMA、FPS test PASS、錄製 100/100 幀、mic overflow 0、internal RAM 最低約 50 KB（未連 OpenOCD 時驗證）。
- 失敗的是 **JTAG 傳輸**：OpenOCD 讀回的區塊標頭出現協定上不可能的值（`Incomplete block sz 3, wr 216`），接著解析錯位、結束。
- 觸發條件是**系統有大量 DMA / 匯流排活動**（NET_BENCH 的 Wi-Fi 1.7 MB/s 傳送、錄製時 cam GDMA + SD + I2S），不是相機本身，也和緩衝大小無關。
- Espressif 在 IDF 5.2.6 承認 S3 有此問題並加了 CRC16 + 延遲（只降低失敗率）；IDF 5.1.2 兩者都沒有，換新版 OpenOCD 也沒用。
- 想看**錄製期間**的排程，SystemView over JTAG 在 IDF 5.1.2 上做不到，改用 FreeRTOS run-time stats + 既有 timing trace。

## 設定：`sdkconfig.sysview`（overlay，只給 trace build）
正式韌體不受影響；`SDKCONFIG_DEFAULTS` 裡 overlay 排在後面，覆蓋 `sdkconfig.defaults` 的同名值。

| 設定 | 值 | 原因 |
|---|---|---|
| `APPTRACE_DEST_JTAG` / `APPTRACE_SV_ENABLE` / `APPTRACE_SV_DEST_JTAG` | y | SystemView 走 JTAG |
| `APPTRACE_SV_MAX_TASKS` | 32 | 預設 16 不夠（系統 + 本專案約 18 個 task） |
| `APPTRACE_SV_BUF_WAIT_TMO` | 0 | buffer 滿直接丟事件，不讓 task 等 JTAG（預設 500 µs 會讓 cam_task 卡超過 DMA 預算） |
| `APPTRACE_PENDING_DATA_SIZE_MAX` | 8192 | 兩塊 TRAX 都忙時暫存事件。必須 < 16384（切換時整包搬進空區塊，太大會跨區塊） |
| `APPTRACE_SV_EVT_ISR_ENTER/EXIT/TO_SCHED_ENABLE` | y | 三個要一起開或一起關；關掉時 ISR 時間會算進被中斷的 context |
| `APPTRACE_SV_EVT_TIMER_ENTER/EXIT_ENABLE` | n | FreeRTOS 軟體 timer，本專案沒用（`xTimerCreate` 無） |
| `CAMERA_DMA_BUFFER_SIZE_MAX` | 32768 | cam_task 每個 half buffer 的預算 2.35 → 4.70 ms，EOF 中斷 400 → 200 次/s（見〈相機時序〉） |

**檔案只能用 ASCII**：IDF 5.1 的 kconfgen 用 Windows 預設編碼（cp950）讀 defaults 檔，中文註解會讓 cmake configure 失敗
（`UnicodeDecodeError: 'cp950' codec can't decode byte 0xe5`）。`.c/.h` 不受影響。

開 JTAG trace 的固定代價：
- **保留 32 KB 內部 DRAM** 當 trace memory（`ESP32S3_TRACEMEM_RESERVE_DRAM=0x8000`，`0x3FCD0000`、`0x3FCE0000` 各 16 KB），無法調整。
- 開了 `APPTRACE_SV_ENABLE` 後，所有非共享中斷都經過 `intr_alloc.c` 的 `non_shared_intr_isr()` wrapper，**在全域 spinlock 裡執行整個 ISR**
  → 兩核心的 ISR 互相排隊。所以 SystemView 看到的 ISR 延遲 / 抖動不準，要量 ISR 觸發時間用 GPIO 41/42 + 邏輯分析儀或 timing trace。
- FreeRTOS API 事件（每次 queue / semaphore / notify，ID ≥ 32）**不能用 Kconfig 關**；相機每個 DMA half buffer 都會有 `xQueueSendFromISR` + receive 事件。

## Build / flash
```powershell
. <IDF 5.1.2>\export.ps1           # 這台：C:\Espressif_5.1.2\frameworks\esp-idf-v5.1.2\export.ps1
Remove-Item build\sv\sdkconfig      # overlay 改過才需要；build\sv 是新的就不用
idf.py -B build\sv -D SDKCONFIG=build\sv\sdkconfig -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.sysview" build
idf.py -B build\sv -D SDKCONFIG=build\sv\sdkconfig -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.sysview" -p COMxx flash monitor
Select-String "PENDING_DATA|SV_EVT_ISR|CAMERA_DMA_BUFFER" build\sv\sdkconfig   # 確認套用
```
開機 log 應有 `esp_apptrace: Initialized TRAX on CPU0/CPU1`，以及
`s3 ll_cam: ... dma_half_buffer: 15360, lines_per_half_buffer: 12, dma_buffer_size: 30720`。
燒錄走 UART0（板上 COM 口）；JTAG 要另一條線接「USB」口（GPIO19/20）。

## 抓 trace
需要：USB-JTAG 驅動（`idf-env driver install --espressif` 或 Zadig 把 Interface 2 設 WinUSB）、
SEGGER SystemView 3.x（mcore 合併檔）、Windows telnet 用戶端（`Enable-WindowsOptionalFeature -Online -FeatureName TelnetClient`）。

```powershell
# 視窗 2：OpenOCD（同時只能一個，否則 LIBUSB_ERROR_ACCESS）
Get-Process openocd -ErrorAction SilentlyContinue | Stop-Process
openocd -f board/esp32s3-builtin.cfg -l openocd_sv.log     # 5.1.2 附的 v0.12.0-esp32-20230921
# 或 IDF 5.2.6 附的 v0.12.0-esp32-20250707（這台有，結果相同）：
# $OO = "C:\Espressif\tools\openocd-esp32\v0.12.0-esp32-20250707\openocd-esp32"
# & "$OO\bin\openocd.exe" -s "$OO\share\openocd\scripts" -f board/esp32s3-builtin.cfg -l openocd_sv.log

# 視窗 3
telnet localhost 4444
```
telnet 內：
```
reset run; sleep 2000; esp sysview_mcore start file://taskN.SVDat     # 從開機約 2 s 開始
esp sysview_mcore start file://taskN.SVDat                            # 或：不 reset，從現在開始
esp sysview_mcore stop
```
- **不能在 app 初始化前開始**：`reset halt` → `start` → `resume` 會讓 app 開機時重新初始化 apptrace，
  OpenOCD 判定斷線、中途重連，出現 `sysview: Invalid init seq len 6!`、0 筆資料。`sleep 2000` 讓 app 先跑完 apptrace 初始化（相機 init 在約 7.3 s）。
- `start` / `stop` 會**暫停兩個核心**一下（設定 host-connected 位元），盡量不要在錄製視窗內下。
- 開始後不要按 RST、不要用 COM 口重燒（USB-JTAG 會重新列舉）。
- 檔案寫在 OpenOCD 的工作目錄。telnet 上印的數字是已收到的位元組數。
- mcore（20250707 版）：資料先寫 `taskN.SVDat_core0/_core1`，**`stop` 時才合併**成 `taskN.SVDat`。中途結束時合併檔是 0 bytes，
  但 `_coreN` 都是完整的 SystemView 檔，可各自開（SystemView 選 All files）。分檔模式：`esp sysview start file://a_pro.SVDat file://a_app.SVDat`。
- 預設無大小上限、無 timeout，會一直收到 `stop` 或 OpenOCD 結束。

## 看結果
- Context 欄的 **Idle** = 該核心沒有 task 在跑（IDF 在切到 IDLE task 時呼叫 `SEGGER_SYSVIEW_OnIdle()`，不當成一般 task）。
- 核心閒置時每 10 ms 會看到一次 Idle（tick，`FREERTOS_HZ=100`）。時間戳時脈 = APB 80 MHz / 2 = **40 MHz**（400,000 tick = 10 ms）。
- ISR 事件關掉時，ISR 時間算進被中斷的 context（含 Idle）→ Idle 會高估空閒。
- IDF 附的 `tools/esp_app_trace/sysviewtrace_proc.py` 解不了這些檔（`Invalid event xQueueGenericReceive(49) payload len 15! Must be 19.`），用 SystemView GUI。

## 運作機制（為什麼會壞）
```
hook → SEGGER_SYSVIEW_Record*()（全域 spinlock）→ s_events_buf[255] → esp_apptrace_write()
     → TRAX trace memory：BLK0 0x3FCD0000 / BLK1 0x3FCE0000（各 16 KB，CPU 寫一塊、host 讀另一塊）
     → 每包前加 {block_sz, wr_sz}（uint8）；開始寫 wr_sz=0，寫完 wr_sz=block_sz
     → 寫滿：swap_start（確認 host 已 ack）→ 改 SENSITIVE_INTERNAL_SRAM_USAGE_2_REG 換 mux → swap_end 寫 ctrl reg {id, len}
     → OpenOCD 經 USB-Serial-JTAG → TAP → TRAX 暫存器逐 word 讀回 → 檢查 wr_sz==block_sz → 拆核心 → 寫檔 → ack
```
協定設計上，量太大只會丟事件，不會出現壞資料。來源：IDF `components/app_trace/port/xtensa/port.c` 開頭的說明、`app_trace_membufs_proto.c`、
`sys_view/esp/SEGGER_RTT_esp.c`。

## 根本原因
1. **症狀違反協定**：`sz 3, wr 216`、`sz 9, wr 248`（wr > sz）。CPU 只會寫出 wr_sz = 0 或 = block_sz，所以是 host 讀到的內容和 SRAM 不一致或錯位；
   第一個錯之後標頭全部錯開 → `Unsupported predef event 0!` 一串 → OpenOCD 結束。
2. **已排除**：
   - 靜態變數 / heap 蓋到 trace memory：`.bss` 結束於 `0x3FCB6A38`；heap 用 `SOC_RESERVE_MEMORY_REGION` 保留兩塊（開機 log 的 heap 區段也避開了 `0x3FCD0000-0x3FCD3FFF`、`0x3FCE0000-0x3FCE3FFF`）。
   - 兩核心同時寫：所有寫入都在 SystemView 全域鎖內。
   - core 1 事件用 bit 6 當標記造成 ID 混淆：API fast ID 最大 32+29=61 < 64。
   - reset 流程、OpenOCD 版本、BUF_WAIT_TMO、pending buffer、ISR 事件開關：都試過，都會壞。
3. **觸發條件**：時間對照（見測試紀錄）每次都落在大量 DMA 活動期間：NET_BENCH `[boot]`（開機 2.39–7.28 s，Wi-Fi 送 8 MB）或錄製開始。唯一長時間成功（941 KB）是在閒置時。
4. **原廠證據**：IDF 5.2.6 `components/app_trace/app_trace_membufs_proto.c` 的 `esp_apptrace_membufs_swap_waitus()`：
   ```c
   #if CONFIG_IDF_TARGET_ESP32S3
       /* ESP32S3 has a serious data corruption issue with the transferred data to host.
        * This delay helps reduce the failure rate by temporarily reducing heavy memory writes
        * from RTOS-level tracing and giving OpenOCD more time to read trace memory ...
        * While this doesn't completely prevent memory access from other threads/cores/ISRs,
        * it has shown to significantly improve reliability when combined with CRC checks in OpenOCD. */
       esp_rom_delay_us(100);
   #endif
   ```
   5.2.6 的 `port/xtensa/port.c` 另在每次切換時對整塊算 CRC16 寫進 `ERI_PERFMON_PM1`；OpenOCD 20250707 有 `CRC mismatch!` 重讀。
   能靠「重讀」改善 → SRAM 內容是對的，壞在讀出路徑。具體硬體原因原廠沒說明。
5. 新版 OpenOCD 對 IDF 5.1.2 韌體無效：韌體端沒寫 CRC，OpenOCD 無從驗證。

換 IDF ≥ 5.2.6 的評估：只降低失敗率；CRC16 對 16 KB 在 SystemView 鎖內（中斷遮蔽）估計每次 0.3–0.6 ms（未實測），會吃掉相機 DMA 預算；
整個專案升級要重驗 esp32-camera 本地修改版、I2S、Wi-Fi、SD → 只為 SystemView 不划算。比 5.2.6 新的版本是否都有此修正未確認。

## 相機時序（OV7725，示波器實測）
| 項目 | 實測 / 換算 |
|---|---|
| PCLK | 250 ns → 4 MHz（XCLK 16 MHz ÷ CLKRC 4），1 PCLK = 1 byte（8 條資料線） |
| HREF | 週期 392 µs、high 320 µs → 每行有效 1280 B（640 px × 2 B YUV422），水平 blanking 72 µs |
| VSYNC | 週期 199.948 ms（5.001 fps），約 510 行 = 480 有效 + 30 blanking（約 11.8 ms，含 VSYNC 脈衝約 1.57 ms） |
| 資料率 | HREF 內 4 MB/s；每行平均 3.27 MB/s；整幀平均 3.07 MB/s |

DMA ping-pong 永遠 2 個 half（`ll_cam.c` 的 `ll_cam_calc_rgb_dma()` 取不超過 MAX/2 且整除 480 行的最大行數）：

| `CAMERA_DMA_BUFFER_SIZE_MAX` | half | 行數 | cam_task 預算 | EOF |
|---|---|---|---|---|
| 16384（正式韌體） | 7680 B | 6 | 2.35 ms | 400 次/s |
| 32768（trace build，Kconfig 上限） | 15360 B | 12 | 4.70 ms | 200 次/s |

每個 half 的實際轉換工作只有幾十 µs，預算幾乎都是「能被卡多久」。第一次失敗（`FB-SIZE ... cnt == 52`，80 個 half 少 28 個）= cam_task 一幀內多次被卡超過 2.35 ms
（當時 ISR 事件開、BUF_WAIT_TMO=500、16 KB DMA）。

## 測試紀錄（2026-10-21）
trace 檔（`*.SVDat*`）與 `openocd_sv.log` 只在原本那台 PC 的專案根目錄，不進 git。

| 檔案 | 設定 / 流程 | 結果 |
|---|---|---|
| `sv_trace.SVDat`（15:03，已被覆寫） | ISR 開、TMO 500、16 KB DMA；app 執行中直接 start（閒置期間） | **941 KB**，之後停止增長 |
| `sv_trace.SVDat`（15:24） | `reset halt` → start → `resume` | `apptrace connection is lost` → `Invalid init seq len 6`，只有 88 B 檔頭 |
| `boot1.SVDat` | 同上設定；`reset run; sleep 2000; start` | 28 KB 後 OpenOCD 結束；相機 warm-up `FB-SIZE != 307200`、`Warm-up timed out`（trace 干擾） |
| `task1.SVDat` | ISR 關、TMO 0 | **相機正常**；42 KB 後 `Incomplete block sz 182, wr 30` → 結束 |
| `task2.SVDat_core0/1` | 換 OpenOCD 20250707、mcore | 21 KB，同樣錯誤後結束；合併檔 0 B |
| task3（覆寫） | 同上 | 3 KB 後結束 |
| `task4_pro/app.SVDat` | 不 reset，app 跑穩後才 start | 123 + 32 KB；**錄製開始那一刻**一連串亂碼標頭（`sz 3, wr 216`…）後結束，MCU 照常執行 |
| `task5_pro/app.SVDat` | — | 14 + 5 KB，未分析 |
| `task6.SVDat_core0/1` | ISR 開、TMO 0、32 KB DMA、mcore | start 後約 4 s（≈ 開機 6 s，NET_BENCH 中）`Incomplete block sz 3, wr 17` → 結束 |
| `task7.SVDat_core0/1` | 再加 pending 8192 | start 後約 1 s（≈ 開機 3 s，NET_BENCH 中）結束 |

同一版 trace 韌體（未連 OpenOCD）的 COM log：`camera init` internal free 69.5 KB / min 53.0 KB；FPS test `PASS complete=50 gaps=0`；
錄製 `saved=100 dropped=0`；mic `overflow=0`；`before upload` min 50.6 KB。

## 下一步（待決定）
1. trace build 關 NET_BENCH（`APP_NET_BENCH_ENABLE=0`），先看開機 → 相機 init → FPS test。錄製 / 上傳期間預期仍會壞。
2. **錄製期間改用 FreeRTOS run-time stats**（建議）：開 `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS` + `CONFIG_FREERTOS_USE_TRACE_FACILITY`，
   在 `rec_session.c` 錄製開始 / 結束各取一次 `uxTaskGetSystemState()` 相減 → 每個 task 在各核心的 CPU 時間、stack high water mark。不走 JTAG、不干擾相機；沒有時間軸。
3. SystemView 只用在閒置 / 輕負載階段（heartbeat、Wi-Fi 重連）。ISR 精確時序用 GPIO 41/42 翻轉 + 邏輯分析儀（不要用 14，緊鄰 PCLK）。
