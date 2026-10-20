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

typedef struct {
    bool success;
    uint32_t saved_frames;
    uint32_t dropped_frames;
    uint32_t frame_gaps;
    int64_t elapsed_ms;
    time_t recorded_at_epoch;
    char file_path[CAM_RECORD_PATH_MAX];
    size_t file_bytes;
    esp_err_t error;
} cam_record_result_t;

esp_err_t cam_record_init(void);
esp_err_t cam_record_once(cam_record_result_t *result);
esp_err_t cam_record_pause(void);
esp_err_t cam_record_resume(void);
/* 僅供開機 A/B：在沒有 capture/writer 使用影格時，只停 HAL/VSYNC/DMA，
 * 保留目前 XCLK。一般錄影與串流仍使用原有 pause/resume 完整管理時脈。 */
esp_err_t cam_record_pause_dma(void);
/* 僅允許在 DMA 已暫停時控制 ESP32 的 XCLK，沿用既有 LEDC timer 管理。
 * 板載振盪器模式回傳 ESP_ERR_NOT_SUPPORTED，無法由 ESP32 停止該時脈。 */
esp_err_t cam_record_set_xclk_output(bool enable);
void cam_record_deinit(void);
