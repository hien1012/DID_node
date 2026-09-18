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
#define CAM_RECORD_DURATION_SEC  10U
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
void cam_record_deinit(void);
