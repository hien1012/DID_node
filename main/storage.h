#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#define STORAGE_PATH_MAX 96

typedef struct {
    char undone_path[STORAGE_PATH_MAX];
    char done_path[STORAGE_PATH_MAX];
    bool active;
} storage_recording_t;

esp_err_t storage_init(void);
esp_err_t storage_begin_recording(storage_recording_t *recording);
esp_err_t storage_finish_recording(storage_recording_t *recording, bool commit);
esp_err_t storage_get_file_size(const char *path, size_t *size_out);
void storage_deinit(void);
