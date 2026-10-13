#include "node_storage.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "driver/sdmmc_host.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdmmc_cmd.h"

#include "board_pins.h"

#define STORAGE_MOUNT_POINT      "/sdcard"
#define STORAGE_UNDONE_DIR       STORAGE_MOUNT_POINT "/undone"
#define STORAGE_DONE_DIR         STORAGE_MOUNT_POINT "/done"
#define STORAGE_BROKEN_DIR       STORAGE_MOUNT_POINT "/broken"
#define STORAGE_MIN_VALID_EPOCH  1704067200LL
#define STORAGE_STEM_FORMAT      "%Y_%m_%d_%H%M%S"
#define STORAGE_STEM_MAX         24U
#define STORAGE_BROKEN_SUFFIX    "_broken"
#define STORAGE_MAX_OPEN_FILES   8

/*
 * 1-bit SDMMC must sustain about 1.57 MB/s (camera + mic), which needs the
 * 40 MHz high-speed clock. Drop to SDMMC_FREQ_DEFAULT (20 MHz) only to debug
 * wiring: it is too slow for recording.
 */
#define STORAGE_SD_FREQ_KHZ      20000

static const char *TAG = "STORAGE";
static sdmmc_card_t *s_sd_card = NULL;
static SemaphoreHandle_t s_storage_mutex = NULL;
static bool s_initialized = false;

static WORD_ALIGNED_ATTR uint8_t s_io_buffer[NODE_STORAGE_IO_BUFFER_BYTES];
static const char *s_io_buffer_owner = NULL;
static portMUX_TYPE s_io_buffer_lock = portMUX_INITIALIZER_UNLOCKED;

uint8_t *node_storage_io_buffer_acquire(const char *owner)
{
    const char *current = NULL;
    taskENTER_CRITICAL(&s_io_buffer_lock);
    if (s_io_buffer_owner == NULL) {
        s_io_buffer_owner = owner ? owner : "?";
    } else {
        current = s_io_buffer_owner;
    }
    taskEXIT_CRITICAL(&s_io_buffer_lock);
    if (current != NULL) {
        ESP_LOGE(TAG, "I/O buffer requested by %s while held by %s",
                 owner ? owner : "?", current);
        return NULL;
    }
    return s_io_buffer;
}

void node_storage_io_buffer_release(const uint8_t *buffer)
{
    if (buffer != s_io_buffer) {
        return;
    }
    taskENTER_CRITICAL(&s_io_buffer_lock);
    s_io_buffer_owner = NULL;
    taskEXIT_CRITICAL(&s_io_buffer_lock);
}

static const char *media_extension(node_media_t media)
{
    return media == NODE_MEDIA_AUDIO ? ".wav" : ".raw";
}

const char *node_storage_media_name(node_media_t media)
{
    return media == NODE_MEDIA_AUDIO ? "audio" : "video";
}

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

static bool format_stem(time_t epoch, char *stem, size_t stem_size)
{
    struct tm local_time = {0};
    return localtime_r(&epoch, &local_time) != NULL &&
           strftime(stem, stem_size, STORAGE_STEM_FORMAT, &local_time) > 0;
}

/*
 * Parse "YYYY_MM_DD_HHMMSS<suffix>.raw|.wav". suffix may be NULL (done/) or
 * STORAGE_BROKEN_SUFFIX (undone/ recovery does not need it).
 */
static bool parse_filename(const char *name, const char *suffix,
                           time_t *epoch_out, node_media_t *media_out)
{
    int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
    int consumed = 0;
    if (sscanf(name, "%4d_%2d_%2d_%2d%2d%2d%n", &year, &month, &day,
               &hour, &minute, &second, &consumed) != 6 || consumed <= 0) {
        return false;
    }

    const char *rest = name + consumed;
    if (suffix) {
        const size_t suffix_len = strlen(suffix);
        if (strncmp(rest, suffix, suffix_len) != 0) {
            return false;
        }
        rest += suffix_len;
    }
    node_media_t media;
    if (strcmp(rest, ".raw") == 0) {
        media = NODE_MEDIA_VIDEO;
    } else if (strcmp(rest, ".wav") == 0) {
        media = NODE_MEDIA_AUDIO;
    } else {
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
    const time_t epoch = mktime(&parsed);
    if ((int64_t)epoch < STORAGE_MIN_VALID_EPOCH) {
        return false;
    }
    if (epoch_out) {
        *epoch_out = epoch;
    }
    if (media_out) {
        *media_out = media;
    }
    return true;
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

/* Grow the file once so FAT never allocates clusters while recording. */
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
        ESP_LOGE(TAG, "Preallocation failed (errno=%d)", errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Files left in undone/ were cut off by a reset or power loss. */
static void mark_interrupted_recordings(void)
{
    DIR *directory = opendir(STORAGE_UNDONE_DIR);
    if (!directory) {
        return;
    }
    struct dirent *entry = NULL;
    while ((entry = readdir(directory)) != NULL) {
        time_t epoch = 0;
        node_media_t media = NODE_MEDIA_VIDEO;
        if (!parse_filename(entry->d_name, NULL, &epoch, &media)) {
            continue;
        }
        char stem[STORAGE_STEM_MAX] = {0};
        char from[sizeof(STORAGE_UNDONE_DIR) + NAME_MAX + 1] = {0};
        char to[NODE_STORAGE_PATH_MAX] = {0};
        if (!format_stem(epoch, stem, sizeof(stem))) {
            continue;
        }
        snprintf(from, sizeof(from), STORAGE_UNDONE_DIR "/%s", entry->d_name);
        snprintf(to, sizeof(to), STORAGE_BROKEN_DIR "/%s" STORAGE_BROKEN_SUFFIX "%s",
                 stem, media_extension(media));
        if (!path_exists(to) && rename(from, to) == 0) {
            ESP_LOGW(TAG, "Interrupted recording marked broken: %s", to);
        } else {
            ESP_LOGE(TAG, "Cannot move interrupted recording %s (errno=%d)",
                     from, errno);
        }
    }
    closedir(directory);
}

esp_err_t node_storage_init(void)
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
        .max_files = STORAGE_MAX_OPEN_FILES,
        .allocation_unit_size = 16U * 1024U,
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = STORAGE_SD_FREQ_KHZ;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;
    slot.clk = SD_PIN_CLK;
    slot.cmd = SD_PIN_CMD;
    slot.d0 = SD_PIN_D0;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t err = esp_vfs_fat_sdmmc_mount(
        STORAGE_MOUNT_POINT, &host, &slot, &mount_config, &s_sd_card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_storage_mutex);
        s_storage_mutex = NULL;
        return err;
    }
    sdmmc_card_print_info(stdout, s_sd_card);

    err = ensure_directory(STORAGE_UNDONE_DIR);
    if (err == ESP_OK) {
        err = ensure_directory(STORAGE_DONE_DIR);
    }
    if (err == ESP_OK) {
        err = ensure_directory(STORAGE_BROKEN_DIR);
    }
    if (err != ESP_OK) {
        esp_vfs_fat_sdcard_unmount(STORAGE_MOUNT_POINT, s_sd_card);
        s_sd_card = NULL;
        vSemaphoreDelete(s_storage_mutex);
        s_storage_mutex = NULL;
        return err;
    }

    mark_interrupted_recordings();
    s_initialized = true;
    ESP_LOGI(TAG, "SD mounted (1-bit, %u kHz); directories are ready",
             (unsigned)STORAGE_SD_FREQ_KHZ);
    return ESP_OK;
}

esp_err_t node_storage_begin_recording(node_recording_t *recording,
                                       node_media_t media,
                                       time_t recorded_at_epoch,
                                       size_t expected_bytes)
{
    if (!recording || recorded_at_epoch <= 0 || expected_bytes == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(recording, 0, sizeof(*recording));
    recording->fd = -1;
    recording->media = media;
    recording->recorded_at_epoch = recorded_at_epoch;
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    char stem[STORAGE_STEM_MAX] = {0};
    if (!format_stem(recorded_at_epoch, stem, sizeof(stem))) {
        return ESP_FAIL;
    }
    const char *extension = media_extension(media);
    snprintf(recording->undone_path, sizeof(recording->undone_path),
             STORAGE_UNDONE_DIR "/%s%s", stem, extension);
    snprintf(recording->done_path, sizeof(recording->done_path),
             STORAGE_DONE_DIR "/%s%s", stem, extension);
    snprintf(recording->broken_path, sizeof(recording->broken_path),
             STORAGE_BROKEN_DIR "/%s" STORAGE_BROKEN_SUFFIX "%s", stem, extension);

    if (xSemaphoreTake(s_storage_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (path_exists(recording->undone_path) || path_exists(recording->done_path) ||
        path_exists(recording->broken_path)) {
        ESP_LOGE(TAG, "Recording already exists: %s%s", stem, extension);
        xSemaphoreGive(s_storage_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    int fd = open(recording->undone_path, O_CREAT | O_EXCL | O_WRONLY, 0644);
    xSemaphoreGive(s_storage_mutex);
    if (fd < 0) {
        ESP_LOGE(TAG, "Cannot create %s (errno=%d)", recording->undone_path, errno);
        return ESP_FAIL;
    }

    /* Preallocation of the 92 MB RAW takes seconds; keep the mutex free. */
    esp_err_t err = preallocate_file(fd, expected_bytes);
    if (err != ESP_OK) {
        close(fd);
        unlink(recording->undone_path);
        return err;
    }

    recording->fd = fd;
    recording->active = true;
    ESP_LOGI(TAG, "Prepared %s (%u bytes)", recording->undone_path,
             (unsigned)expected_bytes);
    return ESP_OK;
}

esp_err_t node_storage_finish_recording(node_recording_t *recording, bool upload)
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
        upload = false;
    }
    recording->fd = -1;

    const char *target = upload ? recording->done_path : recording->broken_path;
    if (path_exists(target)) {
        ESP_LOGE(TAG, "Target already exists: %s", target);
        err = ESP_ERR_INVALID_STATE;
    } else if (rename(recording->undone_path, target) != 0) {
        ESP_LOGE(TAG, "Cannot move %s -> %s (errno=%d)",
                 recording->undone_path, target, errno);
        err = ESP_FAIL;
    } else if (upload) {
        ESP_LOGI(TAG, "Recording ready for upload: %s", target);
    } else {
        ESP_LOGW(TAG, "Recording marked broken: %s", target);
    }

    recording->active = false;
    xSemaphoreGive(s_storage_mutex);
    return err;
}

esp_err_t node_storage_get_next_pending(node_pending_file_t *pending)
{
    if (!pending) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(pending, 0, sizeof(*pending));
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_storage_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    DIR *directory = opendir(STORAGE_DONE_DIR);
    if (!directory) {
        xSemaphoreGive(s_storage_mutex);
        return ESP_FAIL;
    }
    time_t best_epoch = (time_t)LLONG_MAX;
    node_media_t best_media = NODE_MEDIA_VIDEO;
    char best_name[NAME_MAX + 1] = {0};
    struct dirent *entry = NULL;
    while ((entry = readdir(directory)) != NULL) {
        time_t epoch = 0;
        node_media_t media = NODE_MEDIA_VIDEO;
        if (!parse_filename(entry->d_name, NULL, &epoch, &media)) {
            continue;
        }
        /* Same session: the small WAV goes first. */
        const bool better = epoch < best_epoch ||
                            (epoch == best_epoch && media == NODE_MEDIA_AUDIO);
        if (better) {
            best_epoch = epoch;
            best_media = media;
            strlcpy(best_name, entry->d_name, sizeof(best_name));
        }
    }
    closedir(directory);

    esp_err_t err = ESP_ERR_NOT_FOUND;
    if (best_name[0] != '\0') {
        int length = snprintf(pending->path, sizeof(pending->path),
                              STORAGE_DONE_DIR "/%s", best_name);
        if (length > 0 && (size_t)length < sizeof(pending->path)) {
            pending->media = best_media;
            pending->recorded_at_epoch = best_epoch;
            err = node_storage_get_file_size(pending->path, &pending->file_bytes);
        } else {
            err = ESP_ERR_INVALID_SIZE;
        }
    }
    xSemaphoreGive(s_storage_mutex);
    return err;
}

static size_t count_matching(const char *directory_path, const char *suffix)
{
    DIR *directory = opendir(directory_path);
    if (!directory) {
        return 0;
    }
    size_t count = 0;
    struct dirent *entry = NULL;
    while ((entry = readdir(directory)) != NULL) {
        if (parse_filename(entry->d_name, suffix, NULL, NULL)) {
            ++count;
        }
    }
    closedir(directory);
    return count;
}

esp_err_t node_storage_count(size_t *pending_out, size_t *broken_out)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_storage_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (pending_out) {
        *pending_out = count_matching(STORAGE_DONE_DIR, NULL);
    }
    if (broken_out) {
        *broken_out = count_matching(STORAGE_BROKEN_DIR, STORAGE_BROKEN_SUFFIX);
    }
    xSemaphoreGive(s_storage_mutex);
    return ESP_OK;
}

esp_err_t node_storage_delete_uploaded(const char *path)
{
    static const char prefix[] = STORAGE_DONE_DIR "/";
    if (!path || strncmp(path, prefix, sizeof(prefix) - 1U) != 0 ||
        strchr(path + sizeof(prefix) - 1U, '/') != NULL) {
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
        ESP_LOGE(TAG, "Cannot delete uploaded file %s (errno=%d)", path, errno);
        err = ESP_FAIL;
    }
    xSemaphoreGive(s_storage_mutex);
    return err;
}

esp_err_t node_storage_mark_rejected(const node_pending_file_t *pending)
{
    if (!pending) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    char stem[STORAGE_STEM_MAX] = {0};
    char target[NODE_STORAGE_PATH_MAX] = {0};
    if (!format_stem(pending->recorded_at_epoch, stem, sizeof(stem))) {
        return ESP_FAIL;
    }
    snprintf(target, sizeof(target), STORAGE_BROKEN_DIR "/%s" STORAGE_BROKEN_SUFFIX "%s",
             stem, media_extension(pending->media));

    if (xSemaphoreTake(s_storage_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = ESP_OK;
    if (path_exists(target) || rename(pending->path, target) != 0) {
        ESP_LOGE(TAG, "Cannot move rejected %s (errno=%d)", pending->path, errno);
        err = ESP_FAIL;
    } else {
        ESP_LOGW(TAG, "Server rejected upload; kept as %s", target);
    }
    xSemaphoreGive(s_storage_mutex);
    return err;
}

esp_err_t node_storage_get_file_size(const char *path, size_t *size_out)
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

esp_err_t node_storage_get_free_bytes(uint64_t *free_bytes_out)
{
    if (!free_bytes_out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    uint64_t total_bytes = 0;
    return esp_vfs_fat_info(STORAGE_MOUNT_POINT, &total_bytes, free_bytes_out);
}
