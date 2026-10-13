#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

#define CAM_STORAGE_PATH_MAX 96U

typedef struct {
    char undone_path[CAM_STORAGE_PATH_MAX];
    char done_path[CAM_STORAGE_PATH_MAX];
    time_t recorded_at_epoch;
    int fd;
    bool active;
} cam_storage_recording_t;

esp_err_t cam_storage_init(void);
esp_err_t cam_storage_begin_recording(cam_storage_recording_t *recording,
                                      time_t recorded_at_epoch,
                                      size_t expected_bytes);
esp_err_t cam_storage_finish_recording(cam_storage_recording_t *recording,
                                       bool commit);
esp_err_t cam_storage_get_file_size(const char *path, size_t *size_out);
esp_err_t cam_storage_get_next_done(char *path, size_t path_size,
                                    time_t *recorded_at_epoch);
esp_err_t cam_storage_count_done(size_t *count_out);
esp_err_t cam_storage_delete_done(const char *path);
esp_err_t cam_storage_get_free_bytes(uint64_t *free_bytes_out);
void cam_storage_deinit(void);
