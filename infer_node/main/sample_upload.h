#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "sample_store.h"

typedef struct {
    size_t uploaded_count;
    size_t failed_count;
    size_t rejected_count;
    uint64_t uploaded_bytes;
    uint32_t gap_wait_ms;       /* 等管線空檔的總時間 */
    uint32_t gap_timeouts;      /* 等不到空檔、照樣送出的次數 */
    esp_err_t last_error;
} sample_upload_summary_t;

/* 讀檔緩衝：共用 I/O 緩衝的一半，另一半給 cam_writer */
#define SAMPLE_UPLOAD_BUFFER_BYTES (16U * 1024U)

/* 等管線空檔；true = 有空檔，false = 等了 max_wait_ms 還沒有 (上傳照樣繼續) */
typedef bool (*sample_upload_gap_fn)(uint32_t max_wait_ms);

/*
 * wait_gap：每次讀 SD (目錄、meta、每 16 KB 資料、刪檔) 前呼叫。
 * 麥克風節點傳 mic_infer_wait_gap；相機連續擷取沒有空檔，傳 NULL = 不等。
 */
esp_err_t sample_upload_init(uint8_t *read_buffer, sample_upload_gap_fn wait_gap);

/*
 * 上傳這個 media 所有待上傳樣本 (舊的先)，每個 ACK 驗證後才刪除。
 * 只在 main task 呼叫；擷取與推論都繼續進行，有 wait_gap 時讀 SD 會避開推論與寫檔。
 */
esp_err_t sample_upload_pending(sample_media_t media, sample_upload_summary_t *summary);
