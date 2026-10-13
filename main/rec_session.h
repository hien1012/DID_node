#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

#include "cam_record.h"
#include "mic_record.h"

typedef struct {
    time_t started_at;          /* scheduled start; also the file name */
    int64_t start_late_ms;      /* > 0 when preparation overran the lead time */
    bool camera_enabled;
    bool mic_enabled;
    esp_err_t camera_prepare_error;
    esp_err_t mic_prepare_error;
    cam_record_result_t camera;
    mic_record_result_t mic;
} rec_session_result_t;

/* Declare which recorders passed their boot-time initialization. */
void rec_session_init(bool camera_ready, bool mic_ready);

/*
 * Record camera and microphone together. Call about APP_PREPARE_LEAD_MS before
 * start_epoch: files are preallocated first, then both start at start_epoch.
 * Blocks until both recordings are closed and moved to done/ or broken/.
 */
esp_err_t rec_session_run(time_t start_epoch, rec_session_result_t *result);
