#include "storage.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define STORAGE_MOUNT_POINT   "/sdcard"
#define STORAGE_UNDONE_DIR    STORAGE_MOUNT_POINT "/undone"
#define STORAGE_DONE_DIR      STORAGE_MOUNT_POINT "/done"
#define STORAGE_UPLOADED_DIR  STORAGE_MOUNT_POINT "/uploaded"

static const char *TAG = "STORAGE";

static sdmmc_card_t *s_sd_card = NULL;
static SemaphoreHandle_t s_storage_mutex = NULL;
static bool s_initialized = false;

static bool path_exists(const char *path)
{
    struct stat info;
    return stat(path, &info) == 0;
}

static esp_err_t ensure_directory(const char *path)
{
    if (mkdir(path, 0775) == 0 || errno == EEXIST) {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Cannot create directory %s (errno=%d)", path, errno);
    return ESP_FAIL;
}

static int get_next_file_index(void)
{
    char path[STORAGE_PATH_MAX];

    for (int index = 1; index < 1000000; ++index) {
        snprintf(path, sizeof(path), STORAGE_MOUNT_POINT "/%d.wav", index);
        if (path_exists(path)) {
            continue;
        }

        snprintf(path, sizeof(path), STORAGE_UNDONE_DIR "/%d.wav", index);
        if (path_exists(path)) {
            continue;
        }

        snprintf(path, sizeof(path), STORAGE_DONE_DIR "/%d.wav", index);
        if (path_exists(path)) {
            continue;
        }

        snprintf(path, sizeof(path), STORAGE_UPLOADED_DIR "/%d.wav", index);
        if (!path_exists(path)) {
            return index;
        }
    }

    return -1;
}

esp_err_t storage_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_storage_mutex = xSemaphoreCreateMutex();
    if (!s_storage_mutex) {
        return ESP_ERR_NO_MEM;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = 20000;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t err = esp_vfs_fat_sdmmc_mount(
        STORAGE_MOUNT_POINT, &host, &slot_config, &mount_config, &s_sd_card);
    if (err != ESP_OK) {
        vSemaphoreDelete(s_storage_mutex);
        s_storage_mutex = NULL;
        return err;
    }

    err = ensure_directory(STORAGE_UNDONE_DIR);
    if (err == ESP_OK) {
        err = ensure_directory(STORAGE_DONE_DIR);
    }
    if (err == ESP_OK) {
        err = ensure_directory(STORAGE_UPLOADED_DIR);
    }
    if (err != ESP_OK) {
        esp_vfs_fat_sdcard_unmount(STORAGE_MOUNT_POINT, s_sd_card);
        s_sd_card = NULL;
        vSemaphoreDelete(s_storage_mutex);
        s_storage_mutex = NULL;
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "SD mounted; recording directories are ready");
    return ESP_OK;
}

esp_err_t storage_begin_recording(storage_recording_t *recording)
{
    if (!recording) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(recording, 0, sizeof(*recording));

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_storage_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    int index = get_next_file_index();
    if (index < 0) {
        xSemaphoreGive(s_storage_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    snprintf(recording->undone_path, sizeof(recording->undone_path),
             STORAGE_UNDONE_DIR "/%d.wav", index);
    snprintf(recording->done_path, sizeof(recording->done_path),
             STORAGE_DONE_DIR "/%d.wav", index);
    recording->active = true;

    ESP_LOGI(TAG, "Reserved recording: %s", recording->undone_path);
    return ESP_OK;
}

esp_err_t storage_finish_recording(storage_recording_t *recording, bool commit)
{
    if (!recording || !recording->active || !s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ESP_OK;
    if (commit) {
        if (path_exists(recording->done_path)) {
            ESP_LOGE(TAG, "Done file already exists: %s", recording->done_path);
            err = ESP_ERR_INVALID_STATE;
        } else if (rename(recording->undone_path, recording->done_path) != 0) {
            ESP_LOGE(TAG, "Cannot commit %s (errno=%d)", recording->undone_path, errno);
            err = ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "Recording done: %s", recording->done_path);
        }
    } else if (path_exists(recording->undone_path)) {
        ESP_LOGW(TAG, "Incomplete recording kept as: %s", recording->undone_path);
    } else {
        ESP_LOGW(TAG, "Recording failed before a partial file was created");
    }

    recording->active = false;
    xSemaphoreGive(s_storage_mutex);
    return err;
}

esp_err_t storage_get_file_size(const char *path, size_t *size_out)
{
    if (!path || !size_out) {
        return ESP_ERR_INVALID_ARG;
    }

    struct stat info;
    if (stat(path, &info) != 0) {
        return ESP_ERR_NOT_FOUND;
    }

    *size_out = (size_t)info.st_size;
    return ESP_OK;
}

void storage_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    if (xSemaphoreTake(s_storage_mutex, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Storage is busy; deinit skipped");
        return;
    }

    esp_vfs_fat_sdcard_unmount(STORAGE_MOUNT_POINT, s_sd_card);
    s_sd_card = NULL;
    s_initialized = false;

    xSemaphoreGive(s_storage_mutex);
    vSemaphoreDelete(s_storage_mutex);
    s_storage_mutex = NULL;
}
