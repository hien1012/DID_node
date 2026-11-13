#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

#define CAM_IMAGE_WIDTH          640U
#define CAM_IMAGE_HEIGHT         480U
#define CAM_IMAGE_FRAME_BYTES    (CAM_IMAGE_WIDTH * CAM_IMAGE_HEIGHT)
#define CAM_TARGET_FPS           5U

/* cam_writer 的 SD 寫入暫存：共用 I/O 緩衝的一半，另一半給上傳 */
#define CAM_INFER_STAGING_BYTES  (16U * 1024U)

typedef struct {
    uint32_t frames;            /* 啟動以來處理的幀數 */
    uint32_t dropped;           /* 佇列滿丟幀 */
    uint32_t rois;              /* CNN 推論次數 */
    uint32_t low_conf;          /* 含低信賴 ROI 的幀數 */
    uint32_t saved;
    uint32_t last_fd_us;
    uint32_t last_invoke_us;
} cam_infer_stats_t;

/* 相機上機、暖機、開機 5 s FPS 測試與模型載入，與錄影韌體 main/cam_record.c 相同流程，不需要按鈕。
 * 要在 Core 1 呼叫；結束時相機是運作中，接著要呼叫 cam_infer_pause()。 */
esp_err_t cam_infer_init(void);

/*
 * 開始連續擷取與推論，之後不會停 (立即返回)：每幀做 FD + 追蹤，候選 ROI 送 CNN，
 * 低信賴度的幀存 SD。staging_buffer 是內部 RAM、CAM_INFER_STAGING_BYTES 大小。
 * 要 SD 與 result_report 都準備好才呼叫。
 *
 * XCLK 因此一直開著：did_node 在麵包板上量過，運作中的相機匯流排會把 Wi-Fi TCP 吞吐量
 * 從約 3.8 MB/s 降到 0.4 MB/s (docs/debug_sdcard.md #19)，樣本上傳會比較慢。
 */
esp_err_t cam_infer_start(uint8_t *staging_buffer);

esp_err_t cam_infer_pause(void);

void cam_infer_get_stats(cam_infer_stats_t *stats);
