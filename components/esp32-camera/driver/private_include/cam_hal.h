// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "esp_camera.h"
#include "freertos/FreeRTOS.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Uninitialize the lcd_cam module
 *
 * @param handle Provide handle pointer to release resources
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Uninitialize fail
 */
esp_err_t cam_deinit(void);

/**
 * @brief Initialize the lcd_cam module
 *
 * @param config Configurations - see lcd_cam_config_t struct
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM No memory to initialize lcd_cam
 *     - ESP_FAIL Initialize fail
 */
esp_err_t cam_init(const camera_config_t *config);

esp_err_t cam_config(const camera_config_t *config, framesize_t frame_size, uint16_t sensor_pid);

/* Stop DMA and wait until cam_task has discarded pending events and frames. */
esp_err_t cam_stop(void);

void cam_start(void);

camera_fb_t *cam_take(TickType_t timeout);

void cam_give(camera_fb_t *dma_buffer);

void cam_give_all(void);

bool cam_get_available_frames(void);

void cam_set_psram_mode(bool enable);
bool cam_get_psram_mode(void);

/*
 * DMA 層時序統計 (本地修改，debug)。由 cam_task 累計，只在 start 到 stop 之間記錄。
 * start / stop 只能在相機暫停時呼叫 (cam_task 閒置)，因此不需要鎖。
 */
typedef struct {
    uint32_t count;
    int64_t sum_us;
    int64_t min_us;
    int64_t max_us;
} cam_timing_stat_t;

typedef struct {
    cam_timing_stat_t eof_interval;     /* 同一幀內相鄰兩次 EOF 中斷的間隔 (= 填滿一個 half buffer 的時間) */
    cam_timing_stat_t eof_isr_to_task;  /* EOF 中斷 → cam_task 收到事件 */
    cam_timing_stat_t eof_copy;         /* cam_task 收到 EOF → 從 DMA buffer 取出 Y 寫進 PSRAM fb 完成 */
    cam_timing_stat_t vsync_isr_to_task;/* VSYNC 中斷 → cam_task 收到事件 */
    uint32_t event_queue_peak;          /* cam_task 取事件時 queue 內最多有幾個 (含取出的那個) */
    uint32_t event_queue_size;          /* event_queue 容量 */
    uint32_t half_buffer_bytes;         /* 每次 EOF 的資料量 (YUV422，取 Y 前) */
} cam_dma_timing_t;

void cam_dma_timing_start(void);
void cam_dma_timing_stop(cam_dma_timing_t *out);

/* fb 收完整幀的時間 (cam_task 處理結尾 VSYNC 時)；fb 不屬於驅動時回傳 0 */
int64_t cam_get_frame_done_us(const camera_fb_t *fb);

#ifdef __cplusplus
}
#endif
