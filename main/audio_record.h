#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

#define AUDIO_RECORD_PATH_MAX 96

typedef struct {
    bool success;
    bool capture_done;
    bool writer_done;
    bool write_error;
    esp_err_t error;
    char file_path[AUDIO_RECORD_PATH_MAX];
    time_t recorded_at_epoch;
    size_t total_samples;
    size_t file_bytes;
    uint32_t overflow_count;
    uint32_t elapsed_ms;
} audio_record_result_t;

esp_err_t audio_record_init(void);
esp_err_t audio_record_once(audio_record_result_t *result);
/* 僅供開機測速：獨占 recorder 切換 I2S/DMA，結束後還原原本時脈狀態。
 * PDM 若曾停止時脈，會重新計算第一輪錄音前的穩定等待時間。 */
esp_err_t audio_record_run_network_benchmarks(void);
void audio_record_deinit(void);
