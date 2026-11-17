#include "task_usage.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "infer_time.h"
#include "result_report.h"

#if !CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
#error "Run time stats are off: delete infer_node/sdkconfig so sdkconfig.defaults is applied"
#endif

/* 關閉時 task_usage.h 提供空的 inline 版本，這個檔案不編任何東西 */
#if INFER_TASK_USAGE_ENABLE

/* 系統約有 20 個 task；放不下時 uxTaskGetSystemState() 回傳 0，只會少這段統計 */
#define MAX_TASKS 32U

typedef struct {
    TaskHandle_t handle;
    char name[configMAX_TASK_NAME_LEN];     /* 回報記錄直接指向這裡，所以 table 內容不移動 */
    int8_t core;                            /* 0、1；-1 = 不綁核心 */
    uint32_t start_counter;
    uint64_t total_us;
} task_entry_t;

/* 一組統計：區間開始時記下每個 task 的計數器，結束時累加差值 */
typedef struct {
    task_entry_t entries[MAX_TASKS];
    size_t entry_count;
    int64_t start_us;
    bool open;
    uint64_t wall_total_us;
    uint32_t intervals;                     /* 累積了幾個區間 */
} usage_set_t;

static const char *TAG = "TASK_USAGE";

/*
 * 兩組統計各自一份 snapshot 緩衝，可以在不同 task 同時進行 (相機：推論在 cam_infer、一輪在 cam_detect)。
 * static 放內部 RAM，因為 snapshot 是在關閉排程的狀態下寫入。
 *   s_infer：只有推論 task 存取，不用鎖。
 *   s_cycle：mark 的 task 寫、report 可以在另一個 task 讀，用 s_cycle_lock 保護；
 *            report 在鎖內複製到 s_cycle_copy 後才印，印 log 的時間不佔鎖。
 */
static TaskStatus_t s_infer_status[MAX_TASKS];
static TaskStatus_t s_cycle_status[MAX_TASKS];
static int8_t s_infer_cores[MAX_TASKS];
static int8_t s_cycle_cores[MAX_TASKS];
static usage_set_t s_infer;     /* 推論期間 */
static usage_set_t s_cycle;     /* 一輪 (mark 到 mark) */
static usage_set_t s_cycle_copy;
static portMUX_TYPE s_cycle_lock = portMUX_INITIALIZER_UNLOCKED;

/*
 * ulRunTimeCounter 只在 task 切換時更新，所以先 yield，呼叫者自己的計數才是最新的。
 * 另一顆核心正在跑的 task 不會更新，單次會有誤差，但開頭和結尾的誤差多次平均後會互相抵銷。
 * 核心綁定在這裡讀 (鎖外)：不綁核心的 task 第一次用 float 時會被自動綁定，所以每次重讀。
 */
static UBaseType_t take_snapshot(TaskStatus_t status[MAX_TASKS], int8_t cores[MAX_TASKS])
{
    taskYIELD();
    const UBaseType_t count = uxTaskGetSystemState(status, MAX_TASKS, NULL);
    for (UBaseType_t i = 0; i < count; ++i) {
        const BaseType_t affinity = xTaskGetAffinity(status[i].xHandle);
        cores[i] = affinity == tskNO_AFFINITY ? -1 : (int8_t)affinity;
    }
    return count;
}

static task_entry_t *find_entry(usage_set_t *set, TaskHandle_t handle)
{
    for (size_t i = 0; i < set->entry_count; ++i) {
        if (set->entries[i].handle == handle) {
            return &set->entries[i];
        }
    }
    return NULL;
}

static task_entry_t *add_entry(usage_set_t *set, const TaskStatus_t *status)
{
    if (set->entry_count >= MAX_TASKS) {
        return NULL;
    }
    task_entry_t *entry = &set->entries[set->entry_count++];
    memset(entry, 0, sizeof(*entry));
    entry->handle = status->xHandle;
    strlcpy(entry->name, status->pcTaskName, sizeof(entry->name));
    return entry;
}

/* 用剛抓的 snapshot 當每個 task 的起點 */
static void set_start(usage_set_t *set, const TaskStatus_t *status, const int8_t *cores,
                      UBaseType_t count, int64_t now_us)
{
    for (UBaseType_t i = 0; i < count; ++i) {
        task_entry_t *entry = find_entry(set, status[i].xHandle);
        if (entry == NULL) {
            entry = add_entry(set, &status[i]);
        }
        if (entry != NULL) {
            entry->start_counter = status[i].ulRunTimeCounter;
            entry->core = cores[i];
        }
    }
    set->open = true;
    set->start_us = now_us;
}

/* 用剛抓的 snapshot 結束區間：累加每個 task 從起點到現在的計數 */
static void set_accumulate(usage_set_t *set, const TaskStatus_t *status, UBaseType_t count,
                           int64_t now_us)
{
    set->wall_total_us += (uint64_t)(now_us - set->start_us);
    ++set->intervals;
    for (UBaseType_t i = 0; i < count; ++i) {
        task_entry_t *entry = find_entry(set, status[i].xHandle);
        if (entry != NULL) {
            /* 無號相減，計數器 (約 71 分鐘) 溢位時也正確 */
            entry->total_us += (uint32_t)(status[i].ulRunTimeCounter - entry->start_counter);
        }
    }
}

/* 由多到少排序 */
static void sort_entries(const usage_set_t *set, uint8_t order[MAX_TASKS])
{
    for (size_t i = 0; i < set->entry_count; ++i) {
        size_t at = i;
        while (at > 0U && set->entries[order[at - 1U]].total_us < set->entries[i].total_us) {
            order[at] = order[at - 1U];
            --at;
        }
        order[at] = (uint8_t)i;
    }
}

/*
 * 印出平均每個區間的表，最後一行是兩顆核心的忙碌度 = 100% − 該核心 IDLE 的占比。
 * 中斷沒有獨立計時，算在被打斷的 task 上 (也可能是 IDLE)。
 */
static void print_table(const usage_set_t *set, const uint8_t order[MAX_TASKS], const char *what,
                        const char *unit, uint32_t avg_wall_us)
{
    ESP_LOGI(TAG, "Task CPU per %s: %u %s, avg %.1f ms (2 cores = 200%%)", what,
             (unsigned)set->intervals, unit, (double)avg_wall_us / 1000.0);
    uint64_t sum_us = 0;
    uint64_t idle_us[2] = {0, 0};
    for (size_t i = 0; i < set->entry_count; ++i) {
        const task_entry_t *entry = &set->entries[order[i]];
        const uint32_t avg_us = (uint32_t)(entry->total_us / set->intervals);
        sum_us += avg_us;
        if (strncmp(entry->name, "IDLE", 4) == 0 && entry->core >= 0 && entry->core <= 1) {
            idle_us[entry->core] += avg_us;
        }
        if (avg_us > 0U) {
            ESP_LOGI(TAG, "  %-16s core %2d %9.1f ms %6.1f%%", entry->name, entry->core,
                     (double)avg_us / 1000.0,
                     (double)avg_us * 100.0 / (double)avg_wall_us);
        }
    }
    ESP_LOGI(TAG, "  sum %.1f%%", (double)sum_us * 100.0 / (double)avg_wall_us);
#if !CONFIG_FREERTOS_UNICORE
    ESP_LOGI(TAG, "  busy: core 0 %.1f%%, core 1 %.1f%%",
             100.0 - (double)idle_us[0] * 100.0 / (double)avg_wall_us,
             100.0 - (double)idle_us[1] * 100.0 / (double)avg_wall_us);
#else
    ESP_LOGI(TAG, "  busy: core 0 %.1f%%",
             100.0 - (double)idle_us[0] * 100.0 / (double)avg_wall_us);
#endif
}

static void set_clear_totals(usage_set_t *set)
{
    for (size_t i = 0; i < set->entry_count; ++i) {
        set->entries[i].total_us = 0;
    }
    set->wall_total_us = 0;
    set->intervals = 0;
}

void task_usage_begin(void)
{
    const UBaseType_t count = take_snapshot(s_infer_status, s_infer_cores);
    s_infer.open = false;
    if (count > 0U) {
        set_start(&s_infer, s_infer_status, s_infer_cores, count, esp_timer_get_time());
    }
}

void task_usage_end(void)
{
    if (!s_infer.open) {
        return;
    }
    s_infer.open = false;
    const UBaseType_t count = take_snapshot(s_infer_status, s_infer_cores);
    if (count == 0U) {
        return;
    }
    set_accumulate(&s_infer, s_infer_status, count, esp_timer_get_time());
}

void task_usage_report(const char *media)
{
    if (s_infer.intervals == 0U) {
        return;
    }

    /* 回報和 log 只留前面的 */
    uint8_t order[MAX_TASKS];
    sort_entries(&s_infer, order);

    const uint32_t avg_wall_us = (uint32_t)(s_infer.wall_total_us / s_infer.intervals);
    infer_record_t record = {.type = INFER_RECORD_TASK_USAGE};
    infer_task_usage_record_t *out = &record.data.task_usage;
    out->media = media;
    out->ts_epoch_ms = infer_epoch_ms_from_timer(esp_timer_get_time());
    out->inferences = s_infer.intervals;
    out->avg_wall_us = avg_wall_us;
    for (size_t i = 0; i < s_infer.entry_count && i < INFER_TASK_USAGE_MAX; ++i) {
        const task_entry_t *entry = &s_infer.entries[order[i]];
        out->tasks[i].name = entry->name;
        out->tasks[i].core = entry->core;
        out->tasks[i].avg_us = (uint32_t)(entry->total_us / s_infer.intervals);
        out->task_count = (uint8_t)(i + 1U);
    }

    print_table(&s_infer, order, "inference", "inferences", avg_wall_us);
    result_report_push(&record);
    set_clear_totals(&s_infer);
}

void task_usage_cycle_mark(void)
{
    const UBaseType_t count = take_snapshot(s_cycle_status, s_cycle_cores);
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_cycle_lock);
    if (count == 0U) {
        s_cycle.open = false;       /* 這一輪不列入，下一次 mark 重新開始 */
    } else {
        if (s_cycle.open) {
            set_accumulate(&s_cycle, s_cycle_status, count, now_us);
        }
        set_start(&s_cycle, s_cycle_status, s_cycle_cores, count, now_us);
    }
    portEXIT_CRITICAL(&s_cycle_lock);
}

void task_usage_cycle_report(const char *label)
{
    /* 鎖內只複製與歸零 (約 1.3 KB)，排序與印 log 在鎖外做 */
    portENTER_CRITICAL(&s_cycle_lock);
    const bool has_data = s_cycle.intervals > 0U;
    if (has_data) {
        s_cycle_copy = s_cycle;
        set_clear_totals(&s_cycle);
    }
    portEXIT_CRITICAL(&s_cycle_lock);
    if (!has_data) {
        return;
    }
    uint8_t order[MAX_TASKS];
    sort_entries(&s_cycle_copy, order);
    const uint32_t avg_wall_us = (uint32_t)(s_cycle_copy.wall_total_us / s_cycle_copy.intervals);
    print_table(&s_cycle_copy, order, label, "cycles", avg_wall_us);
}

#endif /* INFER_TASK_USAGE_ENABLE */
