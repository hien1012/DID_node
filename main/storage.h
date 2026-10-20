#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

#define STORAGE_PATH_MAX 96

typedef struct {
    char undone_path[STORAGE_PATH_MAX];
    char done_path[STORAGE_PATH_MAX];
    time_t recorded_at_epoch;
    bool active;
} storage_recording_t;

esp_err_t storage_init(void);
esp_err_t storage_begin_recording(storage_recording_t *recording,
                                  time_t recorded_at_epoch);
esp_err_t storage_finish_recording(storage_recording_t *recording, bool commit);
esp_err_t storage_get_file_size(const char *path, size_t *size_out);
esp_err_t storage_get_next_done(char *path, size_t path_size,
                                time_t *recorded_at_epoch);
esp_err_t storage_count_done(size_t *count_out);
esp_err_t storage_delete_done(const char *path);
esp_err_t storage_get_free_bytes(uint64_t *free_bytes_out);
void storage_deinit(void);
