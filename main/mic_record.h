#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

#include "app_config.h"
#include "node_storage.h"

#define MIC_SAMPLE_RATE          16000U
#define MIC_RECORD_DURATION_SEC  APP_RECORD_DURATION_SEC

typedef struct {
    bool prepared;                   /* file was created; finish() must run */
    bool success;                    /* complete and moved to done/ */
    uint32_t samples;
    uint32_t overflow_count;         /* I2S DMA overruns: audio was lost */
    int64_t first_sample_offset_us;  /* first saved sample relative to trigger */
    int64_t elapsed_ms;
    time_t recorded_at_epoch;
    char file_path[NODE_STORAGE_PATH_MAX];
    size_t file_bytes;
    esp_err_t error;
} mic_record_result_t;

/* Start the I2S/PDM channel; it keeps running between recordings. */
esp_err_t mic_record_init(void);

/* Same three-step session API as cam_record. */
esp_err_t mic_record_prepare(time_t recorded_at_epoch);
esp_err_t mic_record_start(int64_t trigger_us);
esp_err_t mic_record_finish(mic_record_result_t *result);
