#include "cam_storage.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdmmc_cmd.h"

#define CAM_STORAGE_MOUNT_POINT      "/sdcard"
#define CAM_STORAGE_UNDONE_DIR       CAM_STORAGE_MOUNT_POINT "/cam_undone"
#define CAM_STORAGE_DONE_DIR         CAM_STORAGE_MOUNT_POINT "/cam_done"
#define CAM_STORAGE_MIN_VALID_EPOCH  1704067200LL
#define CAM_STORAGE_FILENAME_FORMAT  "%Y%m%d_%H%M%S.raw"
#define CAM_STORAGE_FILENAME_MAX     32U

static const char *TAG = "CAM_STORAGE";
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

static bool is_done_path(const char *path)
{
    static const char prefix[] = CAM_STORAGE_DONE_DIR "/";
    return path && strncmp(path, prefix, sizeof(prefix) - 1U) == 0 &&
           strchr(path + sizeof(prefix) - 1U, '/') == NULL;
}

static bool format_recording_filename(time_t epoch, char *name, size_t name_size)
{
    struct tm local_time = {0};
    return localtime_r(&epoch, &local_time) != NULL &&
           strftime(name, name_size, CAM_STORAGE_FILENAME_FORMAT, &local_time) > 0;
}

static bool parse_local_time_filename(const char *name, time_t *epoch_out)
{
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    int consumed = 0;
    int matched = sscanf(name, "%4d%2d%2d_%2d%2d%2d.raw%n",
                         &year, &month, &day, &hour, &minute, &second,
                         &consumed);
    if (matched != 6 || consumed <= 0 || (size_t)consumed != strlen(name)) {
        return false;
    }

    struct tm parsed = {
        .tm_year = year - 1900,
        .tm_mon = month - 1,
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = minute,
        .tm_sec = second,
        .tm_isdst = -1,
    };
    time_t epoch = mktime(&parsed);
    if ((int64_t)epoch < CAM_STORAGE_MIN_VALID_EPOCH) {
        return false;
    }

    struct tm verified = {0};
    if (localtime_r(&epoch, &verified) == NULL ||
        verified.tm_year != year - 1900 || verified.tm_mon != month - 1 ||
        verified.tm_mday != day || verified.tm_hour != hour ||
        verified.tm_min != minute || verified.tm_sec != second) {
        return false;
    }
    *epoch_out = epoch;
    return true;
}

static bool parse_legacy_epoch_filename(const char *name, time_t *epoch_out)
{
    char *end = NULL;
    errno = 0;
    long long value = strtoll(name, &end, 10);
    if (errno != 0 || value < CAM_STORAGE_MIN_VALID_EPOCH || !end ||
        strcmp(end, ".raw") != 0) {
        return false;
    }
    *epoch_out = (time_t)value;
    return true;
}

static bool parse_recorded_epoch(const char *name, time_t *epoch_out)
{
    if (!name || !epoch_out) {
        return false;
    }
    return parse_local_time_filename(name, epoch_out) ||
           parse_legacy_epoch_filename(name, epoch_out);
}

static esp_err_t write_all(int fd, const uint8_t *data, size_t bytes)
{
    size_t offset = 0;
    while (offset < bytes) {
        ssize_t written = write(fd, data + offset, bytes - offset);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            return ESP_FAIL;
        }
        if (written == 0) {
            return ESP_FAIL;
        }
        offset += (size_t)written;
    }
    return ESP_OK;
}

static esp_err_t preallocate_file(int fd, size_t expected_bytes)
{
    if (expected_bytes == 0 || expected_bytes > INT32_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }
    off_t final_byte_offset = (off_t)(expected_bytes - 1U);
    const uint8_t marker = 0;
    if (lseek(fd, final_byte_offset, SEEK_SET) != final_byte_offset ||
        write_all(fd, &marker, sizeof(marker)) != ESP_OK ||
        fsync(fd) != 0 || lseek(fd, 0, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "RAW preallocation failed (errno=%d)", errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t cam_storage_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    s_storage_mutex = xSemaphoreCreateMutex();
    if (!s_storage_mutex) {
        return ESP_ERR_NO_MEM;
    }

    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16U * 1024U,
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t err = esp_vfs_fat_sdmmc_mount(
        CAM_STORAGE_MOUNT_POINT, &host, &slot, &mount_config, &s_sd_card);
    if (err != ESP_OK) {
        vSemaphoreDelete(s_storage_mutex);
        s_storage_mutex = NULL;
        return err;
    }
    sdmmc_card_print_info(stdout, s_sd_card);

    err = ensure_directory(CAM_STORAGE_UNDONE_DIR);
    if (err == ESP_OK) {
        err = ensure_directory(CAM_STORAGE_DONE_DIR);
    }
    if (err != ESP_OK) {
        esp_vfs_fat_sdcard_unmount(CAM_STORAGE_MOUNT_POINT, s_sd_card);
        s_sd_card = NULL;
        vSemaphoreDelete(s_storage_mutex);
        s_storage_mutex = NULL;
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "SD mounted; camera recording directories are ready");
    return ESP_OK;
}

esp_err_t cam_storage_begin_recording(cam_storage_recording_t *recording,
                                      time_t recorded_at_epoch,
                                      size_t expected_bytes)
{
    if (!recording || recorded_at_epoch <= 0 || expected_bytes == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(recording, 0, sizeof(*recording));
    recording->fd = -1;
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_storage_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    char filename[CAM_STORAGE_FILENAME_MAX] = {0};
    if (!format_recording_filename(recorded_at_epoch, filename,
                                   sizeof(filename))) {
        xSemaphoreGive(s_storage_mutex);
        return ESP_FAIL;
    }
    snprintf(recording->undone_path, sizeof(recording->undone_path),
             CAM_STORAGE_UNDONE_DIR "/%s", filename);
    snprintf(recording->done_path, sizeof(recording->done_path),
             CAM_STORAGE_DONE_DIR "/%s", filename);
    recording->recorded_at_epoch = recorded_at_epoch;

    if (path_exists(recording->undone_path) || path_exists(recording->done_path)) {
        ESP_LOGE(TAG, "Recording timestamp already exists: %lld",
                 (long long)recorded_at_epoch);
        xSemaphoreGive(s_storage_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    int fd = open(recording->undone_path, O_CREAT | O_EXCL | O_WRONLY, 0644);
    if (fd < 0) {
        ESP_LOGE(TAG, "Cannot create %s (errno=%d)",
                 recording->undone_path, errno);
        xSemaphoreGive(s_storage_mutex);
        return ESP_FAIL;
    }
    esp_err_t err = preallocate_file(fd, expected_bytes);
    if (err != ESP_OK) {
        close(fd);
        unlink(recording->undone_path);
        xSemaphoreGive(s_storage_mutex);
        return err;
    }

    recording->fd = fd;
    recording->active = true;
    ESP_LOGI(TAG, "Prepared recording: %s (%u bytes)",
             recording->undone_path, (unsigned)expected_bytes);
    xSemaphoreGive(s_storage_mutex);
    return ESP_OK;
}

esp_err_t cam_storage_finish_recording(cam_storage_recording_t *recording,
                                       bool commit)
{
    if (!recording || !recording->active || !s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_storage_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_OK;
    if (recording->fd >= 0 && close(recording->fd) != 0) {
        ESP_LOGE(TAG, "Cannot close %s (errno=%d)", recording->undone_path, errno);
        err = ESP_FAIL;
        commit = false;
    }
    recording->fd = -1;

    if (commit) {
        if (path_exists(recording->done_path)) {
            err = ESP_ERR_INVALID_STATE;
        } else if (rename(recording->undone_path, recording->done_path) != 0) {
            ESP_LOGE(TAG, "Cannot commit %s (errno=%d)",
                     recording->undone_path, errno);
            err = ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "Recording done: %s", recording->done_path);
        }
    } else if (path_exists(recording->undone_path)) {
        ESP_LOGW(TAG, "Incomplete recording kept as: %s", recording->undone_path);
    }

    recording->active = false;
    xSemaphoreGive(s_storage_mutex);
    return err;
}

esp_err_t cam_storage_get_file_size(const char *path, size_t *size_out)
{
    if (!path || !size_out) {
        return ESP_ERR_INVALID_ARG;
    }
    struct stat info;
    if (stat(path, &info) != 0 || info.st_size < 0) {
        return ESP_ERR_NOT_FOUND;
    }
    *size_out = (size_t)info.st_size;
    return ESP_OK;
}

esp_err_t cam_storage_get_next_done(char *path, size_t path_size,
                                    time_t *recorded_at_epoch)
{
    if (!path || path_size == 0 || !recorded_at_epoch) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_storage_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    DIR *directory = opendir(CAM_STORAGE_DONE_DIR);
    if (!directory) {
        xSemaphoreGive(s_storage_mutex);
        return ESP_FAIL;
    }
    time_t oldest = (time_t)LLONG_MAX;
    char oldest_name[NAME_MAX + 1] = {0};
    struct dirent *entry = NULL;
    while ((entry = readdir(directory)) != NULL) {
        time_t epoch = 0;
        if (parse_recorded_epoch(entry->d_name, &epoch) && epoch < oldest) {
            oldest = epoch;
            strlcpy(oldest_name, entry->d_name, sizeof(oldest_name));
        }
    }
    closedir(directory);

    esp_err_t err = ESP_ERR_NOT_FOUND;
    if (oldest_name[0] != '\0') {
        int length = snprintf(path, path_size, CAM_STORAGE_DONE_DIR "/%s",
                              oldest_name);
        if (length > 0 && (size_t)length < path_size) {
            *recorded_at_epoch = oldest;
            err = ESP_OK;
        } else {
            err = ESP_ERR_INVALID_SIZE;
        }
    }
    xSemaphoreGive(s_storage_mutex);
    return err;
}

esp_err_t cam_storage_count_done(size_t *count_out)
{
    if (!count_out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_storage_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    DIR *directory = opendir(CAM_STORAGE_DONE_DIR);
    if (!directory) {
        xSemaphoreGive(s_storage_mutex);
        return ESP_FAIL;
    }
    size_t count = 0;
    struct dirent *entry = NULL;
    while ((entry = readdir(directory)) != NULL) {
        time_t ignored = 0;
        if (parse_recorded_epoch(entry->d_name, &ignored)) {
            ++count;
        }
    }
    closedir(directory);
    *count_out = count;
    xSemaphoreGive(s_storage_mutex);
    return ESP_OK;
}

esp_err_t cam_storage_delete_done(const char *path)
{
    if (!is_done_path(path)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_storage_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = ESP_OK;
    if (unlink(path) != 0) {
        ESP_LOGE(TAG, "Cannot delete acknowledged file %s (errno=%d)", path, errno);
        err = ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Deleted acknowledged file: %s", path);
    }
    xSemaphoreGive(s_storage_mutex);
    return err;
}

esp_err_t cam_storage_get_free_bytes(uint64_t *free_bytes_out)
{
    if (!free_bytes_out) {
        return ESP_ERR_INVALID_ARG;
    }
    uint64_t total_bytes = 0;
    return esp_vfs_fat_info(CAM_STORAGE_MOUNT_POINT, &total_bytes,
                            free_bytes_out);
}

void cam_storage_deinit(void)
{
    if (!s_initialized) {
        return;
    }
    if (xSemaphoreTake(s_storage_mutex, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Storage is busy; deinit skipped");
        return;
    }
    esp_vfs_fat_sdcard_unmount(CAM_STORAGE_MOUNT_POINT, s_sd_card);
    s_sd_card = NULL;
    s_initialized = false;
    xSemaphoreGive(s_storage_mutex);
    vSemaphoreDelete(s_storage_mutex);
    s_storage_mutex = NULL;
}
