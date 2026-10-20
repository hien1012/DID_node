#pragma once

/*
 * 一輪「準備 → 錄製 → 上傳」的時序追蹤 (debug，APP_TIMING_TRACE)。
 *
 * 三種資料，全部存在同一個 static struct，錄製期間只寫記憶體、不印 log
 * (UART 印一行要數 ms，會改變被量的時序)；一輪結束後 timing_trace_print() 一次印出：
 *   mark  ：單次事件的 esp_timer 時間 (整輪時間軸)，印出時以 trigger 為 0 點
 *   stat  ：重複事件的耗時統計 (count / min / avg / max，以及最慢那次的序號)
 *   gauge ：最高水位或計數 (queue 深度、overflow 次數)
 *
 * mark / stat / gauge 的每一格只由一個 task 寫入，讀取在所有錄製 task 結束後，因此不需要鎖。
 * APP_TIMING_TRACE = 0 時所有函式都是空的 inline，呼叫端不必加 #if。
 */

#include <stdint.h>

#include "app_config.h"

/* 整輪時間軸的交界點。印出順序 = 列舉順序；每一點的「階段」是它和指定參考點的差 (見 timing_trace.c) */
typedef enum {
    TT_MARK_WAKE = 0,           /* sleep_until 醒來 (觸發點前 APP_PREPARE_LEAD_MS) */
    TT_MARK_XCLK_ON,            /* 相機 XCLK 恢復輸出 */
    TT_MARK_CAM_PREPARED,       /* cam_record_prepare 返回：取得 I/O 緩衝 + 建立並預配置 .raw */
    TT_MARK_MIC_PREPARED,       /* mic_record_prepare 返回：I2S 時脈啟動 + 建立並預配置 .wav */
    TT_MARK_WAIT_DONE,          /* 空等到 trigger − 300 ms */
    TT_MARK_CAM_STARTED,        /* cam_record_start 返回：恢復 VSYNC + 建立 sd_writer / camera_capture */
    TT_MARK_MIC_STARTED,        /* mic_record_start 返回：建立 mic_writer / mic_capture */
    TT_MARK_TRIGGER,            /* 觸發點 (0 點)，cam / mic 共用 */
    TT_MARK_MIC_FIRST_SAMPLE,   /* mic 丟完觸發前資料，開始收第一個 sample */
    TT_MARK_CAM_FIRST_FRAME,    /* cam 第一個落在視窗內的幀 (VSYNC 時間) */
    TT_MARK_MIC_CAPTURE_DONE,   /* mic_capture 收滿 sample 數 */
    TT_MARK_MIC_WRITER_DONE,    /* mic_writer 寫完尾段 + 回寫 header + ftruncate + fsync */
    TT_MARK_CAM_CAPTURE_DONE,   /* camera_capture 收到 deadline 之後的幀而結束 */
    TT_MARK_CAM_WRITER_DONE,    /* sd_writer 清空 ready queue + ftruncate + fsync */
    TT_MARK_CAM_FINISHED,       /* cam_record_finish 返回：暫停相機 + 統計 log + rename 到 done/ */
    TT_MARK_XCLK_OFF,           /* 相機 XCLK 停止 */
    TT_MARK_MIC_FINISHED,       /* mic_record_finish 返回：rename 到 done/ + I2S 時脈停止 */
    TT_MARK_UPLOAD_START,       /* upload_pass 開始 */
    TT_MARK_UPLOAD_END,         /* node_upload_process_pending 返回 */
    TT_MARK_RESYNC_DONE,        /* 上傳後校時完成，本輪結束 */
    TT_MARK_COUNT
} tt_mark_t;

/* 重複事件的耗時。序號 (index) 由呼叫端給：幀序號、block 序號、檔案序號… */
typedef enum {
    /* --- cam DMA 層 (相機驅動 cam_task 內累計，錄完匯入；沒有序號) --- */
    TT_STAT_CAM_DMA_EOF_INTERVAL = 0,   /* 同一幀相鄰 EOF 中斷間隔 = 填滿一個 half buffer 的時間 */
    TT_STAT_CAM_DMA_EOF_TO_TASK,        /* EOF 中斷 → cam_task 取到事件 */
    TT_STAT_CAM_DMA_COPY,               /* cam_task 取到 EOF → 從 DMA buffer 取 Y 寫進 PSRAM fb 完成 */
    TT_STAT_CAM_DMA_VSYNC_TO_TASK,      /* VSYNC 中斷 → cam_task 取到事件 */
    /* --- cam 每幀 (序號 = 視窗內幀序號) --- */
    TT_STAT_CAM_READOUT,                /* 幀開始 VSYNC → 驅動收完整幀 (感測器送出一幀的時間) */
    TT_STAT_CAM_DELIVER,                /* 驅動收完整幀 → camera_capture 的 fb_get 返回 */
    TT_STAT_CAM_ENQUEUE,                /* fb_get 返回 → 檢查完成、送進 ready queue */
    TT_STAT_CAM_QUEUE_WAIT,             /* 送進 ready queue → sd_writer 取出 (排隊時間) */
    TT_STAT_CAM_SD_WRITE,               /* sd_writer 取出 → 300 KB 寫進 SD 完成 */
    TT_STAT_CAM_FB_HELD,                /* 幀開始 VSYNC → fb_return (這個 fb 被佔用的總時間) */
    /* --- mic DMA 層 (序號 = DMA 描述元序號，從觸發點起算) --- */
    TT_STAT_MIC_DMA_INTERVAL,           /* 相鄰兩個 I2S DMA 描述元完成的間隔 (應 = 32 ms) */
    TT_STAT_MIC_DMA_TO_READ,            /* DMA 描述元完成 (on_recv 中斷) → i2s_channel_read 交出這筆資料 */
    /* --- mic 每次讀取 / 每個 block (序號 = 讀取次數 / block 序號) --- */
    TT_STAT_MIC_READ_BLOCKED,           /* i2s_channel_read 呼叫內等待 (短 = DMA 已有積壓) */
    TT_STAT_MIC_CONVERT,                /* read 返回 → 32→16 bit 轉進 block 完成 (不含等空 block) */
    TT_STAT_MIC_FREE_WAIT,              /* mic_capture 等空閒 block (> 0 代表 block 池用完) */
    TT_STAT_MIC_QUEUE_WAIT,             /* block 填滿送出 → mic_writer 取出 (排隊時間) */
    TT_STAT_MIC_GAIN,                   /* mic_writer 處理一個 block 的增益 + 複製 (不含 SD 寫入) */
    TT_STAT_MIC_SD_WRITE,               /* 一次 WRITE_BUFFER_BYTES 寫進 SD */
    /* --- 上傳每個檔案 (序號 = 本輪第幾個檔案) --- */
    TT_STAT_UP_SCAN,                    /* node_storage_get_next_pending：掃描 done/ 找最舊的檔 */
    TT_STAT_UP_OPEN,                    /* esp_http_client_open：TCP 連線 + 送出 header */
    TT_STAT_UP_BODY,                    /* 送出整個 body (SD 讀取與 Wi-Fi 送出重疊) */
    TT_STAT_UP_ACK,                     /* 送完 → 收到並驗證 ACK */
    TT_STAT_UP_DELETE,                  /* ACK 後刪除本地檔 */
    TT_STAT_COUNT
} tt_stat_id_t;

typedef enum {
    TT_GAUGE_CAM_DMA_EVENTQ_PEAK = 0,   /* 相機驅動 event queue 最高積壓 */
    TT_GAUGE_CAM_DMA_EVENTQ_SIZE,       /* 相機驅動 event queue 容量 */
    TT_GAUGE_CAM_DMA_HALF_BYTES,        /* 每次 EOF 的資料量 (位元組) */
    TT_GAUGE_CAM_READY_PEAK,            /* ready queue 最高積壓 */
    TT_GAUGE_MIC_DMA_BACKLOG_PEAK,      /* I2S 已完成但還沒被讀走的描述元最多幾個 (容量 = 描述元數 − 1) */
    TT_GAUGE_MIC_FULLQ_PEAK,            /* mic full-block queue 最高積壓 */
    TT_GAUGE_MIC_OVF_RECORDING,         /* 觸發點 → 收滿之間的 I2S overflow (真的掉資料) */
    TT_GAUGE_MIC_OVF_TAIL,              /* 收滿 → 時脈停止之間的 overflow (沒人讀，不影響檔案) */
    TT_GAUGE_COUNT
} tt_gauge_id_t;

#if APP_TIMING_TRACE

/* 新一輪開始：清空所有資料 */
void timing_trace_begin(void);
/* 記錄交界點為現在 / 指定的 esp_timer 時間 */
void timing_trace_mark(tt_mark_t mark);
void timing_trace_mark_at(tt_mark_t mark, int64_t time_us);
/* 加入一筆耗時 */
void timing_trace_stat(tt_stat_id_t id, int64_t duration_us, uint32_t index);
/* 匯入在別處累計好的統計 (相機驅動) */
void timing_trace_stat_import(tt_stat_id_t id, uint32_t count, int64_t sum_us,
                              int64_t min_us, int64_t max_us);
void timing_trace_gauge_max(tt_gauge_id_t id, uint32_t value);
void timing_trace_gauge_set(tt_gauge_id_t id, uint32_t value);
/* 印出整輪結果 (====start==== ... ====end====) */
void timing_trace_print(void);

#else

static inline void timing_trace_begin(void) {}
static inline void timing_trace_mark(tt_mark_t mark) { (void)mark; }
static inline void timing_trace_mark_at(tt_mark_t mark, int64_t time_us)
{
    (void)mark; (void)time_us;
}
static inline void timing_trace_stat(tt_stat_id_t id, int64_t duration_us, uint32_t index)
{
    (void)id; (void)duration_us; (void)index;
}
static inline void timing_trace_stat_import(tt_stat_id_t id, uint32_t count, int64_t sum_us,
                                            int64_t min_us, int64_t max_us)
{
    (void)id; (void)count; (void)sum_us; (void)min_us; (void)max_us;
}
static inline void timing_trace_gauge_max(tt_gauge_id_t id, uint32_t value)
{
    (void)id; (void)value;
}
static inline void timing_trace_gauge_set(tt_gauge_id_t id, uint32_t value)
{
    (void)id; (void)value;
}
static inline void timing_trace_print(void) {}

#endif
