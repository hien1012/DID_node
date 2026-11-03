#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

#define CAM_IMAGE_WIDTH          640U
#define CAM_IMAGE_HEIGHT         480U
#define CAM_IMAGE_FRAME_BYTES    (CAM_IMAGE_WIDTH * CAM_IMAGE_HEIGHT)
#define CAM_TARGET_FPS           5U
#define CAM_RECORD_DURATION_SEC  20U
#define CAM_RECORD_PATH_MAX      96U

/* 每次錄影只歸一類，heartbeat 回報各類累計次數，由 server 統計 */
typedef enum {
    CAM_RECORD_FAULT_NONE = 0,
    CAM_RECORD_FAULT_FRAME_DROP,     /* 佇列滿丟幀（SD 卡頓） */
    CAM_RECORD_FAULT_FB_INCOMPLETE,  /* 驅動收到長度不符的幀而丟棄（FB-SIZE） */
    CAM_RECORD_FAULT_FRAME_GAP,      /* 感測器時戳斷層或幀數不足 */
    CAM_RECORD_FAULT_CAPTURE,        /* 取幀逾時、時戳錯誤、影格大小不對 */
    CAM_RECORD_FAULT_STORAGE,        /* 建檔 / 預配置 / 寫入 / 搬檔失敗 */
    CAM_RECORD_FAULT_OTHER,
    CAM_RECORD_FAULT_COUNT,
} cam_record_fault_t;

typedef struct {
    bool success;
    cam_record_fault_t fault;
    uint32_t saved_frames;
    uint32_t dropped_frames;
    uint32_t frame_gaps;
    uint32_t incomplete_frames;      /* 本次錄影期間驅動丟棄的 FB-SIZE 幀數 */
    int64_t elapsed_ms;
    time_t recorded_at_epoch;
    char file_path[CAM_RECORD_PATH_MAX];
    size_t file_bytes;
    esp_err_t error;
} cam_record_result_t;

esp_err_t cam_record_init(void);
esp_err_t cam_record_once(cam_record_result_t *result);
/* heartbeat JSON 用的固定代碼，與 server_mix.py 對應 */
const char *cam_record_fault_name(cam_record_fault_t fault);
esp_err_t cam_record_pause(void);
esp_err_t cam_record_resume(void);
/* 僅供開機 A/B：在沒有 capture/writer 使用影格時，只停 HAL/VSYNC/DMA，
 * 保留目前 XCLK。一般錄影與串流仍使用原有 pause/resume 完整管理時脈。 */
esp_err_t cam_record_pause_dma(void);
/* 僅允許在 DMA 已暫停時控制 ESP32 的 XCLK，沿用既有 LEDC timer 管理。
 * 板載振盪器模式回傳 ESP_ERR_NOT_SUPPORTED，無法由 ESP32 停止該時脈。 */
esp_err_t cam_record_set_xclk_output(bool enable);
void cam_record_deinit(void);
