#pragma once

#include "cam_record.h"
#include "cam_upload.h"
#include "esp_err.h"

esp_err_t cam_health_monitor_start(void);
void cam_health_note_recording(const cam_record_result_t *result);
void cam_health_note_upload(const cam_upload_summary_t *summary);
