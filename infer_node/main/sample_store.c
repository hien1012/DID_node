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

#include "esp_heap_caps.h"
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

typedef struct {
    char stem[SAMPLE_STEM_MAX];
    uint32_t data_bytes;
} queued_sample_t;

typedef struct {
    queued_sample_t *items;     /* 配置於 PSRAM，不佔內部 DRAM */
    size_t count;
    size_t capacity;
} sample_heap_t;

static const char *TAG = "SAMPLE_STORE";

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_heap_mutex_buf;
static SemaphoreHandle_t s_heap_mutex = NULL;   /* 保護 PSRAM 內的 s_heaps，不關中斷 */
static sample_heap_t s_heaps[SAMPLE_MEDIA_COUNT];
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

static bool media_is_active_role(sample_media_t media)
{
#if INFER_CAM_ENABLE
    return media == SAMPLE_MEDIA_VIDEO;
#else
    return media == SAMPLE_MEDIA_AUDIO;
#endif
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

/* ============================================================================
 * PSRAM 最小二元堆積 (Min-Heap，依 stem 字典序排列)：
 * 開機掃描一次目錄後，上傳直接 O(1) 取堆頂、新增/刪除 O(log N) 純記憶體調整，
 * 避免每傳一筆樣本就重掃整個 FAT32 目錄 (CMD17 單區段讀取與 0xE5 墓碑區開銷)。
 * ============================================================================ */

static bool heap_ensure_capacity(sample_heap_t *h, size_t min_capacity)
{
    if (h->capacity >= min_capacity) {
        return true;
    }
    queued_sample_t *grown = heap_caps_realloc(
        h->items, min_capacity * sizeof(queued_sample_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!grown) {
        return false;
    }
    h->items = grown;
    h->capacity = min_capacity;
    return true;
}

static void heap_sift_up(sample_heap_t *h, size_t idx)
{
    const queued_sample_t item = h->items[idx];
    while (idx > 0U) {
        const size_t parent = (idx - 1U) / 2U;
        if (strcmp(h->items[parent].stem, item.stem) <= 0) {
            break;
        }
        h->items[idx] = h->items[parent];
        idx = parent;
    }
    h->items[idx] = item;
}

static void heap_sift_down(sample_heap_t *h, size_t idx)
{
    const queued_sample_t item = h->items[idx];
    while (true) {
        const size_t left = idx * 2U + 1U;
        const size_t right = left + 1U;
        if (left >= h->count) {
            break;
        }
        size_t smallest = left;
        if (right < h->count && strcmp(h->items[right].stem, h->items[left].stem) < 0) {
            smallest = right;
        }
        if (strcmp(item.stem, h->items[smallest].stem) <= 0) {
            break;
        }
        h->items[idx] = h->items[smallest];
        idx = smallest;
    }
    h->items[idx] = item;
}

static void heap_push_keep_oldest(sample_heap_t *h, const char *stem, uint32_t data_bytes)
{
    if (!h->items || h->capacity == 0U) {
        return;
    }
    if (h->count < h->capacity) {
        const size_t idx = h->count++;
        strlcpy(h->items[idx].stem, stem, sizeof(h->items[idx].stem));
        h->items[idx].data_bytes = data_bytes;
        heap_sift_up(h, idx);
        return;
    }
    /* 超出容量 (例如外部預放超過 500 個檔案)：找葉節點中最大的，若新檔案更舊就取代它 */
    size_t max_leaf = h->count / 2U;
    for (size_t i = max_leaf + 1U; i < h->count; ++i) {
        if (strcmp(h->items[i].stem, h->items[max_leaf].stem) > 0) {
            max_leaf = i;
        }
    }
    if (strcmp(stem, h->items[max_leaf].stem) < 0) {
        strlcpy(h->items[max_leaf].stem, stem, sizeof(h->items[max_leaf].stem));
        h->items[max_leaf].data_bytes = data_bytes;
        heap_sift_up(h, max_leaf);
    }
}

static void heap_remove(sample_heap_t *h, const char *stem)
{
    if (!h->items || h->count == 0U) {
        return;
    }
    size_t idx = 0;
    while (idx < h->count && strcmp(h->items[idx].stem, stem) != 0) {
        ++idx;
    }
    if (idx >= h->count) {
        return;
    }
    const queued_sample_t last = h->items[--h->count];
    if (idx == h->count) {
        return;
    }
    h->items[idx] = last;
    if (idx > 0U && strcmp(h->items[idx].stem, h->items[(idx - 1U) / 2U].stem) < 0) {
        heap_sift_up(h, idx);
    } else {
        heap_sift_down(h, idx);
    }
}

/* 開機盤點 (clean=true 順便清殘檔) 或堆空補貨 (clean=false)：掃一次目錄填入最舊的一批 */
static size_t scan_into_heap_locked(sample_media_t media, bool clean)
{
    sample_heap_t *heap = &s_heaps[media];
    heap->count = 0;

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
            if (stem_len == 0U || stem_len >= SAMPLE_STEM_MAX) {
                continue;
            }
            snprintf(other, sizeof(other), "%s/%.*s%s", dir_path, (int)stem_len, name,
                     data_extension(media));
            struct stat info = {0};
            if (stat(other, &info) == 0 && info.st_size > 0 &&
                (uint64_t)info.st_size <= UINT32_MAX) {
                ++complete;
                if (heap->capacity == 0U) {
                    /* 非本板啟用的角色只有真的掃到舊檔才配置小塊 PSRAM */
                    (void)heap_ensure_capacity(heap, 64U);
                }
                char stem[SAMPLE_STEM_MAX];
                memcpy(stem, name, stem_len);
                stem[stem_len] = '\0';
                heap_push_keep_oldest(heap, stem, (uint32_t)info.st_size);
            } else if (clean) {
                ESP_LOGW(TAG, "Removing meta without data: %s", path);
                (void)unlink(path);
            }
        } else if (clean && has_suffix(name, META_TMP_EXTENSION)) {
            (void)unlink(path);
        } else if (clean && has_suffix(name, data_extension(media))) {
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
    s_heap_mutex = xSemaphoreCreateMutexStatic(&s_heap_mutex_buf);
    if (!s_changed || !s_heap_mutex) {
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
        const sample_media_t m = (sample_media_t)media;
        if (media_is_active_role(m) &&
            !heap_ensure_capacity(&s_heaps[m], INFER_MAX_PENDING_SAMPLES)) {
            return ESP_ERR_NO_MEM;
        }
        s_pending[m] = scan_into_heap_locked(m, true);
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
    /* 關檔前直接從記憶體中的 FatFs FIL 結構讀檔案大小，省去上傳時再做一次 stat() 掃目錄 */
    struct stat data_info = {0};
    if (fstat(file->fd, &data_info) != 0 || data_info.st_size <= 0 ||
        (uint64_t)data_info.st_size > UINT32_MAX) {
        ESP_LOGE(TAG, "fstat failed: %s (errno=%d)", file->data_path, errno);
        sample_store_abort(file);
        return ESP_FAIL;
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

    if (s_heap_mutex && xSemaphoreTake(s_heap_mutex, portMAX_DELAY) == pdTRUE) {
        (void)heap_ensure_capacity(&s_heaps[file->media], INFER_MAX_PENDING_SAMPLES);
        heap_push_keep_oldest(&s_heaps[file->media], file->stem, (uint32_t)data_info.st_size);
        xSemaphoreGive(s_heap_mutex);
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
    if (!entry || media >= SAMPLE_MEDIA_COUNT || !s_heap_mutex) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_heap_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    sample_heap_t *heap = &s_heaps[media];
    if (heap->count == 0U && sample_store_pending(media) > 0U) {
        /* 只有開機待傳數超過 heap 容量並把前一批傳完時，才補掃一次目錄 */
        const size_t remaining = scan_into_heap_locked(media, false);
        taskENTER_CRITICAL(&s_lock);
        s_pending[media] = remaining;
        taskEXIT_CRITICAL(&s_lock);
    }

    queued_sample_t top;
    const bool found = heap->count > 0U;
    if (found) {
        top = heap->items[0];
    }
    xSemaphoreGive(s_heap_mutex);
    if (!found) {
        return ESP_ERR_NOT_FOUND;
    }

    const char *dir_path = media_dir(media);
    memset(entry, 0, sizeof(*entry));
    entry->media = media;
    memcpy(entry->stem, top.stem, sizeof(entry->stem));
    snprintf(entry->data_path, sizeof(entry->data_path), "%s/%s%s", dir_path, entry->stem,
             data_extension(media));
    snprintf(entry->meta_path, sizeof(entry->meta_path), "%s/%s" META_EXTENSION, dir_path,
             entry->stem);
    entry->data_bytes = top.data_bytes;
    return ESP_OK;
}

static void note_removed(sample_media_t media, const char *stem)
{
    if (s_heap_mutex && stem && xSemaphoreTake(s_heap_mutex, portMAX_DELAY) == pdTRUE) {
        heap_remove(&s_heaps[media], stem);
        xSemaphoreGive(s_heap_mutex);
    }
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
    const bool meta_removed = unlink(entry->meta_path) == 0 || errno == ENOENT;
    const bool data_removed = unlink(entry->data_path) == 0 || errno == ENOENT;
    note_removed(entry->media, entry->stem);
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
    note_removed(entry->media, entry->stem);
    if (!moved) {
        ESP_LOGE(TAG, "Cannot move %s to rejected/ (errno=%d)", entry->stem, errno);
        return ESP_FAIL;
    }
    ESP_LOGW(TAG, "Server rejected %s; moved to rejected/", entry->stem);
    return ESP_OK;
}
