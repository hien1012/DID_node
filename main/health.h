#pragma once

#include "audio_record.h"
#include "esp_err.h"
#include "upload.h"

esp_err_t health_monitor_start(void);
void health_note_recording(const audio_record_result_t *result);
void health_note_upload(const upload_summary_t *summary);

