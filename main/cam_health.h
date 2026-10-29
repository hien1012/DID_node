#pragma once

#include "cam_record.h"
#include "cam_upload.h"
#include "esp_err.h"

/* heartbeat 的 state 欄位（字串見 docs/heartbeat.md） */
typedef enum {
    CAM_HEALTH_STATE_BOOTING = 0,
    CAM_HEALTH_STATE_IDLE,
    CAM_HEALTH_STATE_RECORDING,
    CAM_HEALTH_STATE_UPLOADING,
} cam_health_state_id_t;

esp_err_t cam_health_monitor_start(void);
void cam_health_set_state(cam_health_state_id_t state);
void cam_health_note_recording(const cam_record_result_t *result);
void cam_health_note_upload(const cam_upload_summary_t *summary);
