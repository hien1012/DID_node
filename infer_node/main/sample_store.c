#include "sample_store.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "infer_config.h"

#define SAMPLE_ROOT_DIR      "/sdcard/samples"
#define SAMPLE_AUDIO_DIR     SAMPLE_ROOT_DIR "/audio"
#define SAMPLE_VIDEO_DIR     SAMPLE_ROOT_DIR "/video"
#define SAMPLE_REJECTED_DIR  SAMPLE_ROOT_DIR "/rejected"
#define META_EXTENSION       ".json"
#define META_TMP_EXTENSION   ".jtmp"

static const char *TAG = "SAMPLE_STORE";

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static size_t s_pending[SAMPLE_MEDIA_COUNT];
static uint32_t s_sequence = 0;
static uint32_t s_full_warnings = 0;
static SemaphoreHandle_t s_changed = NULL;

const char *sample_store_media_name(sample_media_t media)
{
    return media == SAMPLE_MEDIA_AUDIO ? "audio" : "video";
}

int sample_store_find_background(const char *const *class_names, int class_count)
{
#if INFER_SAVE_NON_BACKGROUND
    for (int i = 0; i < class_count; ++i) {
        if (class_names[i] && strcmp(class_names[i], INFER_BACKGROUND_LABEL) == 0) {
            return i;
        }
    }
    ESP_LOGW(TAG, "No \"%s\" class; saving low-confidence samples only", INFER_BACKGROUND_LABEL);
#else
    (void)class_names;
    (void)class_count;
#endif
    return -1;
}

const char *sample_store_save_reason(bool low_conf, bool non_background)
{
    if (low_conf && non_background) {
        return "both";
    }
    return non_background ? "non_background" : "low_conf";
}

static const char *media_dir(sample_media_t media)
{
    return media == SAMPLE_MEDIA_AUDIO ? SAMPLE_AUDIO_DIR : SAMPLE_VIDEO_DIR;
}

static const char *data_extension(sample_media_t media)
{
    return media == SAMPLE_MEDIA_AUDIO ? ".wav" : ".raw";
}

static bool has_suffix(const char *name, const char *suffix)
{
    const size_t name_len = strlen(name);
    const size_t suffix_len = strlen(suffix);
    return name_len >= suffix_len && strcmp(name + name_len - suffix_len, suffix) == 0;
}

static esp_err_t ensure_directory(const char *path)
{
    if (mkdir(path, 0775) == 0 || errno == EEXIST) {
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Cannot create directory %s (errno=%d)", path, errno);
    return ESP_FAIL;
}

static bool path_exists(const char *path)
{
    struct stat info;
    return stat(path, &info) == 0;
}

/* 開機盤點：有 .json 的是完整樣本；其餘 (斷電時寫到一半) 刪除 */
static size_t scan_directory(sample_media_t media)
{
    const char *dir_path = media_dir(media);
    DIR *directory = opendir(dir_path);
    if (!directory) {
        return 0;
    }
    size_t complete = 0;
    char path[SAMPLE_PATH_MAX + NAME_MAX];
    char other[SAMPLE_PATH_MAX + NAME_MAX];
    struct dirent *entry = NULL;
    while ((entry = readdir(directory)) != NULL) {
        const char *name = entry->d_name;
        snprintf(path, sizeof(path), "%s/%s", dir_path, name);
        if (has_suffix(name, META_EXTENSION)) {
            const size_t stem_len = strlen(name) - strlen(META_EXTENSION);
            snprintf(other, sizeof(other), "%s/%.*s%s", dir_path, (int)stem_len, name,
                     data_extension(media));
            if (path_exists(other)) {
                ++complete;
            } else {
                ESP_LOGW(TAG, "Removing meta without data: %s", path);
                (void)unlink(path);
            }
        } else if (has_suffix(name, META_TMP_EXTENSION)) {
            (void)unlink(path);
        } else if (has_suffix(name, data_extension(media))) {
            const size_t stem_len = strlen(name) - strlen(data_extension(media));
            snprintf(other, sizeof(other), "%s/%.*s%s", dir_path, (int)stem_len, name,
                     META_EXTENSION);
            if (!path_exists(other)) {
                ESP_LOGW(TAG, "Removing interrupted sample: %s", path);
                (void)unlink(path);
            }
        }
    }
    closedir(directory);
    return complete;
}

esp_err_t sample_store_init(void)
{
    s_changed = xSemaphoreCreateBinary();
    if (!s_changed) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = ensure_directory(SAMPLE_ROOT_DIR);
    if (err == ESP_OK) {
        err = ensure_directory(SAMPLE_AUDIO_DIR);
    }
    if (err == ESP_OK) {
        err = ensure_directory(SAMPLE_VIDEO_DIR);
    }
    if (err == ESP_OK) {
        err = ensure_directory(SAMPLE_REJECTED_DIR);
    }
    if (err != ESP_OK) {
        return err;
    }
    for (int media = 0; media < SAMPLE_MEDIA_COUNT; ++media) {
        s_pending[media] = scan_directory((sample_media_t)media);
    }
    ESP_LOGI(TAG, "Pending samples: audio=%u video=%u",
             (unsigned)s_pending[SAMPLE_MEDIA_AUDIO], (unsigned)s_pending[SAMPLE_MEDIA_VIDEO]);
    return ESP_OK;
}

esp_err_t sample_store_begin(sample_file_t *file, sample_media_t media, int64_t epoch_ms)
{
    if (!file || media >= SAMPLE_MEDIA_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(file, 0, sizeof(*file));
    file->fd = -1;
    file->media = media;

    if (sample_store_pending(media) >= INFER_MAX_PENDING_SAMPLES) {
        if ((s_full_warnings++ % 50U) == 0U) {
            ESP_LOGW(TAG, "%s samples full (%u pending); not saving more",
                     sample_store_media_name(media), (unsigned)INFER_MAX_PENDING_SAMPLES);
        }
        return ESP_ERR_NO_MEM;
    }

    taskENTER_CRITICAL(&s_lock);
    const uint32_t sequence = s_sequence++;
    taskEXIT_CRITICAL(&s_lock);

    const time_t seconds = (time_t)(epoch_ms / 1000LL);
    struct tm local_time = {0};
    char stamp[16] = {0};
    if (!localtime_r(&seconds, &local_time) ||
        strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local_time) == 0) {
        return ESP_FAIL;
    }
    snprintf(file->stem, sizeof(file->stem), "%s_%03d_%05u", stamp,
             (int)(epoch_ms % 1000LL), (unsigned)(sequence % 100000U));
    snprintf(file->data_path, sizeof(file->data_path), "%s/%s%s", media_dir(media),
             file->stem, data_extension(media));

    file->fd = open(file->data_path, O_CREAT | O_TRUNC | O_WRONLY, 0644);
    if (file->fd < 0) {
        ESP_LOGE(TAG, "Cannot create %s (errno=%d)", file->data_path, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

void sample_store_abort(sample_file_t *file)
{
    if (!file) {
        return;
    }
    if (file->fd >= 0) {
        close(file->fd);
        file->fd = -1;
    }
    if (file->data_path[0] != '\0') {
        (void)unlink(file->data_path);
    }
}

esp_err_t sample_store_commit(sample_file_t *file, const char *meta_json)
{
    if (!file || !meta_json || file->fd < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (fsync(file->fd) != 0) {
        ESP_LOGE(TAG, "fsync failed: %s (errno=%d)", file->data_path, errno);
        sample_store_abort(file);
        return ESP_FAIL;
    }
    close(file->fd);
    file->fd = -1;

    char tmp_path[SAMPLE_PATH_MAX];
    char meta_path[SAMPLE_PATH_MAX];
    snprintf(tmp_path, sizeof(tmp_path), "%s/%s" META_TMP_EXTENSION,
             media_dir(file->media), file->stem);
    snprintf(meta_path, sizeof(meta_path), "%s/%s" META_EXTENSION,
             media_dir(file->media), file->stem);

    const int fd = open(tmp_path, O_CREAT | O_TRUNC | O_WRONLY, 0644);
    if (fd < 0) {
        ESP_LOGE(TAG, "Cannot create %s (errno=%d)", tmp_path, errno);
        sample_store_abort(file);
        return ESP_FAIL;
    }
    const size_t meta_bytes = strlen(meta_json);
    const bool written = write(fd, meta_json, meta_bytes) == (ssize_t)meta_bytes &&
                         fsync(fd) == 0;
    close(fd);
    if (!written || rename(tmp_path, meta_path) != 0) {
        ESP_LOGE(TAG, "Cannot finish %s (errno=%d)", meta_path, errno);
        (void)unlink(tmp_path);
        sample_store_abort(file);
        return ESP_FAIL;
    }

    taskENTER_CRITICAL(&s_lock);
    ++s_pending[file->media];
    taskEXIT_CRITICAL(&s_lock);
    if (s_changed) {
        xSemaphoreGive(s_changed);
    }
    return ESP_OK;
}

size_t sample_store_pending(sample_media_t media)
{
    taskENTER_CRITICAL(&s_lock);
    const size_t pending = s_pending[media];
    taskEXIT_CRITICAL(&s_lock);
    return pending;
}

bool sample_store_any_ready(size_t threshold)
{
    return sample_store_pending(SAMPLE_MEDIA_AUDIO) >= threshold ||
           sample_store_pending(SAMPLE_MEDIA_VIDEO) >= threshold;
}

void sample_store_wait_change(TickType_t timeout_ticks)
{
    if (s_changed) {
        (void)xSemaphoreTake(s_changed, timeout_ticks);
    } else {
        vTaskDelay(timeout_ticks);
    }
}

esp_err_t sample_store_next(sample_media_t media, sample_entry_t *entry)
{
    if (!entry || media >= SAMPLE_MEDIA_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    const char *dir_path = media_dir(media);
    DIR *directory = opendir(dir_path);
    if (!directory) {
        return ESP_FAIL;
    }
    /* 檔名以時間開頭，字典序最小的就是最舊的 */
    char oldest[SAMPLE_STEM_MAX] = {0};
    struct dirent *dir_entry = NULL;
    while ((dir_entry = readdir(directory)) != NULL) {
        const char *name = dir_entry->d_name;
        if (!has_suffix(name, META_EXTENSION)) {
            continue;
        }
        const size_t stem_len = strlen(name) - strlen(META_EXTENSION);
        if (stem_len >= sizeof(oldest)) {
            continue;
        }
        char stem[SAMPLE_STEM_MAX];
        memcpy(stem, name, stem_len);
        stem[stem_len] = '\0';
        if (oldest[0] == '\0' || strcmp(stem, oldest) < 0) {
            memcpy(oldest, stem, stem_len + 1U);
        }
    }
    closedir(directory);
    if (oldest[0] == '\0') {
        return ESP_ERR_NOT_FOUND;
    }

    memset(entry, 0, sizeof(*entry));
    entry->media = media;
    memcpy(entry->stem, oldest, sizeof(entry->stem));
    snprintf(entry->data_path, sizeof(entry->data_path), "%s/%s%s", dir_path, oldest,
             data_extension(media));
    snprintf(entry->meta_path, sizeof(entry->meta_path), "%s/%s" META_EXTENSION, dir_path, oldest);
    struct stat info;
    if (stat(entry->data_path, &info) != 0) {
        return ESP_FAIL;
    }
    entry->data_bytes = (size_t)info.st_size;
    return ESP_OK;
}

static void note_removed(sample_media_t media)
{
    taskENTER_CRITICAL(&s_lock);
    if (s_pending[media] > 0U) {
        --s_pending[media];
    }
    taskEXIT_CRITICAL(&s_lock);
}

esp_err_t sample_store_remove(const sample_entry_t *entry)
{
    if (!entry) {
        return ESP_ERR_INVALID_ARG;
    }
    /* 先刪 meta：中途斷電只留下資料檔，開機時會被清掉，不會重傳 */
    const bool meta_removed = unlink(entry->meta_path) == 0;
    const bool data_removed = unlink(entry->data_path) == 0;
    note_removed(entry->media);
    if (!meta_removed || !data_removed) {
        ESP_LOGE(TAG, "Cannot delete %s (errno=%d)", entry->stem, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t sample_store_mark_rejected(const sample_entry_t *entry)
{
    if (!entry) {
        return ESP_ERR_INVALID_ARG;
    }
    char data_target[SAMPLE_PATH_MAX];
    char meta_target[SAMPLE_PATH_MAX];
    snprintf(data_target, sizeof(data_target), SAMPLE_REJECTED_DIR "/%s%s", entry->stem,
             data_extension(entry->media));
    snprintf(meta_target, sizeof(meta_target), SAMPLE_REJECTED_DIR "/%s" META_EXTENSION,
             entry->stem);
    /* 先移資料：若中途斷電，meta 留著會被視為缺資料而清除，不會反覆上傳 */
    const bool moved = rename(entry->data_path, data_target) == 0 &&
                       rename(entry->meta_path, meta_target) == 0;
    note_removed(entry->media);
    if (!moved) {
        ESP_LOGE(TAG, "Cannot move %s to rejected/ (errno=%d)", entry->stem, errno);
        return ESP_FAIL;
    }
    ESP_LOGW(TAG, "Server rejected %s; moved to rejected/", entry->stem);
    return ESP_OK;
}
