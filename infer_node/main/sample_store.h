#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/*
 * 低信賴樣本在 SD 上的存放處 (由 node_storage_init() 掛載的 /sdcard)：
 *   /sdcard/samples/audio/<stem>.wav + <stem>.json
 *   /sdcard/samples/video/<stem>.raw + <stem>.json    (.raw = 全幀 + ROI 拼接)
 *   /sdcard/samples/rejected/                         (server 永久拒收)
 * stem = YYYYMMDD_HHMMSS_mmm_NNNNN。.json 最後才以 rename 產生，所以有 .json 的
 * 才是完整樣本；開機時沒有 .json 的資料檔視為斷電殘留而刪除。
 */

typedef enum {
    SAMPLE_MEDIA_AUDIO = 0,
    SAMPLE_MEDIA_VIDEO,
    SAMPLE_MEDIA_COUNT,
} sample_media_t;

#define SAMPLE_STEM_MAX 48U
#define SAMPLE_PATH_MAX 96U

/* 寫入中的樣本：begin → write(fd) → commit 或 abort */
typedef struct {
    sample_media_t media;
    char stem[SAMPLE_STEM_MAX];
    char data_path[SAMPLE_PATH_MAX];
    int fd;
} sample_file_t;

/* 待上傳的完整樣本 */
typedef struct {
    sample_media_t media;
    char stem[SAMPLE_STEM_MAX];
    char data_path[SAMPLE_PATH_MAX];
    char meta_path[SAMPLE_PATH_MAX];
    size_t data_bytes;
} sample_entry_t;

/** 建立目錄、清除殘留、盤點待上傳數量。SD 要先掛載。 */
esp_err_t sample_store_init(void);

/** 建立資料檔；待上傳數量已達上限時回傳 ESP_ERR_NO_MEM。 */
esp_err_t sample_store_begin(sample_file_t *file, sample_media_t media, int64_t epoch_ms);
/** fsync + 關檔、寫入 meta，樣本才算完整。 */
esp_err_t sample_store_commit(sample_file_t *file, const char *meta_json);
void sample_store_abort(sample_file_t *file);

size_t sample_store_pending(sample_media_t media);
/** 任一 media 的待上傳數量達到 threshold 就回傳 true。 */
bool sample_store_any_ready(size_t threshold);
/** 等到有樣本新增 (或逾時) 才返回，給排程迴圈用。 */
void sample_store_wait_change(TickType_t timeout_ticks);

/** 取最舊的待上傳樣本；沒有時回傳 ESP_ERR_NOT_FOUND。 */
esp_err_t sample_store_next(sample_media_t media, sample_entry_t *entry);
/** 上傳確認後刪除。 */
esp_err_t sample_store_remove(const sample_entry_t *entry);
/** server 永久拒收：移到 rejected/ 留作檢查。 */
esp_err_t sample_store_mark_rejected(const sample_entry_t *entry);

const char *sample_store_media_name(sample_media_t media);

/**
 * 在類別表中找名稱為 INFER_BACKGROUND_LABEL 的索引；找不到回傳 -1。
 * INFER_SAVE_NON_BACKGROUND=0 時一律回傳 -1 (不以非背景為存檔條件)。
 */
int sample_store_find_background(const char *const *class_names, int class_count);
/** 樣本 meta 的 save_reason："low_conf" / "non_background" / "both" */
const char *sample_store_save_reason(bool low_conf, bool non_background);
