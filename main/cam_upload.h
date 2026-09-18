#pragma once

#include <stddef.h>

#include "esp_err.h"

typedef struct {
    size_t uploaded_count;
    size_t failed_count;
    size_t uploaded_bytes;
    esp_err_t last_error;
} cam_upload_summary_t;

esp_err_t cam_upload_process_pending(cam_upload_summary_t *summary);
