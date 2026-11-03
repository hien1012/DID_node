#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

#include "app_config.h"
#include "node_storage.h"

#define CAM_IMAGE_WIDTH          640U
#define CAM_IMAGE_HEIGHT         480U
#define CAM_IMAGE_FRAME_BYTES    (CAM_IMAGE_WIDTH * CAM_IMAGE_HEIGHT)
#define CAM_TARGET_FPS           5U
#define CAM_RECORD_DURATION_SEC  APP_RECORD_DURATION_SEC

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
    bool prepared;                  /* file was created; finish() must run */
    bool success;                   /* complete and moved to done/ (broken video is deleted) */
    cam_record_fault_t fault;
    uint32_t saved_frames;
    uint32_t dropped_frames;
    uint32_t frame_gaps;
    uint32_t incomplete_frames;     /* 本次錄影期間驅動丟棄的 FB-SIZE 幀數 */
    int64_t first_frame_offset_us;  /* first saved frame relative to trigger */
    int64_t elapsed_ms;
    time_t recorded_at_epoch;
    char file_path[NODE_STORAGE_PATH_MAX];
    size_t file_bytes;
    esp_err_t error;
} cam_record_result_t;

/* Camera bring-up, warm-up and the 10 s startup FPS test. SD must be mounted. */
esp_err_t cam_record_init(void);

/* One recording: prepare (create + preallocate), start at trigger_us
 * (esp_timer time), then finish (blocks until the window has elapsed). */
esp_err_t cam_record_prepare(time_t recorded_at_epoch);
esp_err_t cam_record_start(int64_t trigger_us);
esp_err_t cam_record_finish(cam_record_result_t *result);
/* heartbeat JSON 用的固定代碼，與 server_mix.py 對應 */
const char *cam_record_fault_name(cam_record_fault_t fault);

esp_err_t cam_record_pause(void);
esp_err_t cam_record_resume(void);

/*
 * Stop (drive low) or restore the XCLK output while the camera is paused.
 * The OV7670 derives PCLK from XCLK, so stopping it silences the whole camera
 * bus. rec_session keeps XCLK stopped between recordings: measured on the
 * breadboard, the running camera bus cut Wi-Fi TCP throughput from about
 * 3.8 MB/s to 0.4 MB/s (docs/debug_sdcard.md #19).
 * Returns ESP_ERR_NOT_SUPPORTED when the sensor has its own oscillator.
 */
esp_err_t cam_record_set_xclk_output(bool enable);
void cam_record_deinit(void);
