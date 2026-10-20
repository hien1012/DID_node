#include "timing_trace.h"

#if APP_TIMING_TRACE

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"

#define TT_NO_INDEX     UINT32_MAX      /* 匯入的統計沒有「最慢那次」的序號 */
#define TT_NO_REF       TT_MARK_COUNT   /* 時間軸上沒有參考點 (只印絕對時間) */
#define TT_TIME_TEXT_BYTES 32U          /* 正負號 + 20 位數 + '.' + 3 位 + NUL = 26，留餘量 */

typedef struct {
    uint32_t count;
    int64_t sum_us;
    int64_t min_us;
    int64_t max_us;
    uint32_t max_index;     /* 最慢那次的序號，TT_NO_INDEX = 不知道 */
} tt_stat_t;

typedef struct {
    int64_t mark_us[TT_MARK_COUNT];     /* 0 = 本輪沒有走到這一點 */
    tt_stat_t stats[TT_STAT_COUNT];
    uint32_t gauges[TT_GAUGE_COUNT];
} tt_round_t;

/*
 * 時間軸每一點：名稱、參考點、從參考點到這一點的「階段」說明。
 * 參考點不一定是上一行：cam 與 mic 平行執行，各自接在自己的前一步後面。
 */
typedef struct {
    const char *name;
    tt_mark_t ref;
    const char *phase;
} tt_mark_info_t;

static const tt_mark_info_t s_mark_info[TT_MARK_COUNT] = {
    [TT_MARK_WAKE]             = {"wake",             TT_NO_REF,                "scheduler wakes up (prepare lead before trigger)"},
    [TT_MARK_XCLK_ON]          = {"xclk_on",          TT_MARK_WAKE,             "restore camera XCLK"},
    [TT_MARK_CAM_PREPARED]     = {"cam_prepared",     TT_MARK_XCLK_ON,          "cam prepare: I/O buffer + create/preallocate .raw"},
    [TT_MARK_MIC_PREPARED]     = {"mic_prepared",     TT_MARK_CAM_PREPARED,     "mic prepare: I2S clock on + create/preallocate .wav"},
    [TT_MARK_WAIT_DONE]        = {"wait_done",        TT_MARK_MIC_PREPARED,     "idle until trigger - start margin"},
    [TT_MARK_CAM_STARTED]      = {"cam_started",      TT_MARK_WAIT_DONE,        "cam start: VSYNC on + create 2 tasks"},
    [TT_MARK_MIC_STARTED]      = {"mic_started",      TT_MARK_CAM_STARTED,      "mic start: create 2 tasks"},
    [TT_MARK_TRIGGER]          = {"trigger",          TT_MARK_MIC_STARTED,      "margin left before trigger (~300 ms expected)"},
    [TT_MARK_MIC_FIRST_SAMPLE] = {"mic_first_sample", TT_MARK_TRIGGER,          "mic start latency after trigger"},
    [TT_MARK_CAM_FIRST_FRAME]  = {"cam_first_frame",  TT_MARK_TRIGGER,          "first in-window frame VSYNC after trigger"},
    [TT_MARK_MIC_CAPTURE_DONE] = {"mic_capture_done", TT_MARK_MIC_FIRST_SAMPLE, "mic capture length (= record duration)"},
    [TT_MARK_MIC_WRITER_DONE]  = {"mic_writer_done",  TT_MARK_MIC_CAPTURE_DONE, "mic tail write + header + ftruncate + fsync"},
    [TT_MARK_CAM_CAPTURE_DONE] = {"cam_capture_done", TT_MARK_TRIGGER,          "cam capture until first frame past deadline"},
    [TT_MARK_CAM_WRITER_DONE]  = {"cam_writer_done",  TT_MARK_CAM_CAPTURE_DONE, "cam drain ready queue + ftruncate + fsync"},
    [TT_MARK_CAM_FINISHED]     = {"cam_finished",     TT_MARK_CAM_WRITER_DONE,  "cam pause + stats log + rename to done/"},
    [TT_MARK_XCLK_OFF]         = {"xclk_off",         TT_MARK_CAM_FINISHED,     "stop camera XCLK"},
    [TT_MARK_MIC_FINISHED]     = {"mic_finished",     TT_MARK_XCLK_OFF,         "mic finish: rename to done/ + I2S clock off"},
    [TT_MARK_UPLOAD_START]     = {"upload_start",     TT_MARK_MIC_FINISHED,     "session bookkeeping (health note)"},
    [TT_MARK_UPLOAD_END]       = {"upload_end",       TT_MARK_UPLOAD_START,     "upload pass (all pending files)"},
    [TT_MARK_RESYNC_DONE]      = {"resync_done",      TT_MARK_UPLOAD_END,       "time resync with server"},
};

typedef struct {
    const char *section;    /* 非 NULL 時先印分段標題 */
    const char *name;
    const char *meaning;
} tt_stat_info_t;

static const tt_stat_info_t s_stat_info[TT_STAT_COUNT] = {
    [TT_STAT_CAM_DMA_EOF_INTERVAL]  = {"[C] camera DMA (driver cam_task; whole recording incl. pre-trigger frames)",
                                       "eof_interval",  "EOF irq to next EOF irq in one frame (fill one half buffer)"},
    [TT_STAT_CAM_DMA_EOF_TO_TASK]   = {NULL, "eof_to_task",   "EOF irq -> cam_task receives event"},
    [TT_STAT_CAM_DMA_COPY]          = {NULL, "eof_copy",      "cam_task receives EOF -> Y copied to PSRAM fb"},
    [TT_STAT_CAM_DMA_VSYNC_TO_TASK] = {NULL, "vsync_to_task", "VSYNC irq -> cam_task receives event"},
    [TT_STAT_CAM_READOUT]           = {"[B] camera per frame (index = in-window frame)",
                                       "readout",       "frame start VSYNC -> driver has whole frame"},
    [TT_STAT_CAM_DELIVER]           = {NULL, "deliver",       "driver has frame -> camera_capture fb_get returns"},
    [TT_STAT_CAM_ENQUEUE]           = {NULL, "enqueue",       "fb_get returns -> pushed to ready queue"},
    [TT_STAT_CAM_QUEUE_WAIT]        = {NULL, "queue_wait",    "in ready queue -> sd_writer takes it"},
    [TT_STAT_CAM_SD_WRITE]          = {NULL, "sd_write",      "sd_writer takes it -> 300 KB on SD"},
    [TT_STAT_CAM_FB_HELD]           = {NULL, "fb_held",       "frame start VSYNC -> fb returned to driver"},
    [TT_STAT_MIC_DMA_INTERVAL]      = {"[D] microphone (index = DMA descriptor / read / block)",
                                       "dma_interval",  "I2S DMA descriptor done -> next one done (32 ms)"},
    [TT_STAT_MIC_DMA_TO_READ]       = {NULL, "dma_to_read",   "descriptor done irq -> i2s_channel_read hands it over"},
    [TT_STAT_MIC_READ_BLOCKED]      = {NULL, "read_blocked",  "time blocked inside i2s_channel_read"},
    [TT_STAT_MIC_CONVERT]           = {NULL, "convert",       "read returns -> 32->16 bit into block (excl. free_wait)"},
    [TT_STAT_MIC_FREE_WAIT]         = {NULL, "free_wait",     "mic_capture waits for a free block (pool exhausted)"},
    [TT_STAT_MIC_QUEUE_WAIT]        = {NULL, "queue_wait",    "full block queued -> mic_writer takes it"},
    [TT_STAT_MIC_GAIN]              = {NULL, "gain",          "mic_writer gain + copy of one block (excl. SD)"},
    [TT_STAT_MIC_SD_WRITE]          = {NULL, "sd_write",      "one write buffer to SD"},
    [TT_STAT_UP_SCAN]               = {"[E] upload per file (index = file in this pass)",
                                       "scan",          "find oldest file in done/"},
    [TT_STAT_UP_OPEN]               = {NULL, "http_open",     "TCP connect + send headers"},
    [TT_STAT_UP_BODY]               = {NULL, "body",          "send body (SD read overlaps Wi-Fi send)"},
    [TT_STAT_UP_ACK]                = {NULL, "ack",           "body sent -> ACK received and verified"},
    [TT_STAT_UP_DELETE]             = {NULL, "delete",        "delete local file after ACK"},
};

typedef struct {
    const char *name;
    const char *meaning;
} tt_gauge_info_t;

static const tt_gauge_info_t s_gauge_info[TT_GAUGE_COUNT] = {
    [TT_GAUGE_CAM_DMA_EVENTQ_PEAK]  = {"cam_dma_eventq_peak",  "max events waiting for cam_task (incl. the one taken)"},
    [TT_GAUGE_CAM_DMA_EVENTQ_SIZE]  = {"cam_dma_eventq_size",  "cam driver event queue capacity"},
    [TT_GAUGE_CAM_DMA_HALF_BYTES]   = {"cam_dma_half_bytes",   "bytes per EOF (YUV422, before Y extraction)"},
    [TT_GAUGE_CAM_READY_PEAK]       = {"cam_ready_peak",       "max frames waiting in ready queue"},
    [TT_GAUGE_MIC_DMA_BACKLOG_PEAK] = {"mic_dma_backlog_peak", "max finished I2S descriptors not yet read (overflow at desc-1)"},
    [TT_GAUGE_MIC_FULLQ_PEAK]       = {"mic_fullq_peak",       "max full blocks waiting for mic_writer"},
    [TT_GAUGE_MIC_OVF_RECORDING]    = {"mic_ovf_recording",    "I2S overflows trigger..capture done (real data loss)"},
    [TT_GAUGE_MIC_OVF_TAIL]         = {"mic_ovf_tail",         "I2S overflows capture done..clock off (not in file)"},
};

static tt_round_t s_round;

void timing_trace_begin(void)
{
    memset(&s_round, 0, sizeof(s_round));
}

void timing_trace_mark(tt_mark_t mark)
{
    timing_trace_mark_at(mark, esp_timer_get_time());
}

void timing_trace_mark_at(tt_mark_t mark, int64_t time_us)
{
    if (mark < TT_MARK_COUNT) {
        s_round.mark_us[mark] = time_us;
    }
}

void timing_trace_stat(tt_stat_id_t id, int64_t duration_us, uint32_t index)
{
    if (id >= TT_STAT_COUNT) {
        return;
    }
    tt_stat_t *stat = &s_round.stats[id];
    if (stat->count == 0U || duration_us < stat->min_us) {
        stat->min_us = duration_us;
    }
    if (stat->count == 0U || duration_us > stat->max_us) {
        stat->max_us = duration_us;
        stat->max_index = index;
    }
    stat->sum_us += duration_us;
    stat->count++;
}

void timing_trace_stat_import(tt_stat_id_t id, uint32_t count, int64_t sum_us,
                              int64_t min_us, int64_t max_us)
{
    if (id >= TT_STAT_COUNT) {
        return;
    }
    s_round.stats[id] = (tt_stat_t){
        .count = count,
        .sum_us = sum_us,
        .min_us = min_us,
        .max_us = max_us,
        .max_index = TT_NO_INDEX,
    };
}

void timing_trace_gauge_max(tt_gauge_id_t id, uint32_t value)
{
    if (id < TT_GAUGE_COUNT && value > s_round.gauges[id]) {
        s_round.gauges[id] = value;
    }
}

void timing_trace_gauge_set(tt_gauge_id_t id, uint32_t value)
{
    if (id < TT_GAUGE_COUNT) {
        s_round.gauges[id] = value;
    }
}

/* 微秒 → "+1234.567" (ms，三位小數)；不用浮點，避免 newlib nano 不支援 %f */
static void format_ms(char *buf, size_t size, int64_t us)
{
    const char sign = us < 0 ? '-' : '+';
    const uint64_t magnitude = (uint64_t)(us < 0 ? -us : us);
    snprintf(buf, size, "%c%" PRIu64 ".%03u", sign,
             magnitude / 1000U, (unsigned)(magnitude % 1000U));
}

static void print_timeline(void)
{
    const int64_t zero_us = s_round.mark_us[TT_MARK_TRIGGER];
    printf("[A] session timeline: time = ms from trigger, delta = ms from ref\n");
    if (zero_us == 0) {
        printf("    (no trigger this round; times are absolute esp_timer ms)\n");
    }
    for (int i = 0; i < (int)TT_MARK_COUNT; ++i) {
        const tt_mark_info_t *info = &s_mark_info[i];
        const int64_t at_us = s_round.mark_us[i];
        if (at_us == 0) {
            printf("  A%02d %-17s (not reached)\n", i, info->name);
            continue;
        }
        char at_text[TT_TIME_TEXT_BYTES];
        format_ms(at_text, sizeof(at_text), at_us - zero_us);
        if (info->ref == TT_NO_REF || s_round.mark_us[info->ref] == 0) {
            printf("  A%02d %-17s %12s ms                                | %s\n",
                   i, info->name, at_text, info->phase);
            continue;
        }
        char delta_text[TT_TIME_TEXT_BYTES];
        format_ms(delta_text, sizeof(delta_text), at_us - s_round.mark_us[info->ref]);
        printf("  A%02d %-17s %12s ms  delta %11s ms from %-16s | %s\n",
               i, info->name, at_text, delta_text, s_mark_info[info->ref].name, info->phase);
    }
}

static void print_stats(void)
{
    for (int i = 0; i < (int)TT_STAT_COUNT; ++i) {
        const tt_stat_info_t *info = &s_stat_info[i];
        const tt_stat_t *stat = &s_round.stats[i];
        if (info->section != NULL) {
            printf("%s\n", info->section);
            printf("  %-14s %6s %10s %10s %10s %8s | %s\n",
                   "name", "count", "min_us", "avg_us", "max_us", "max_at", "meaning");
        }
        if (stat->count == 0U) {
            printf("  %-14s %6s %10s %10s %10s %8s | %s\n",
                   info->name, "0", "-", "-", "-", "-", info->meaning);
            continue;
        }
        char max_at[16];
        if (stat->max_index == TT_NO_INDEX) {
            snprintf(max_at, sizeof(max_at), "-");
        } else {
            snprintf(max_at, sizeof(max_at), "#%" PRIu32, stat->max_index);
        }
        printf("  %-14s %6" PRIu32 " %10" PRId64 " %10" PRId64 " %10" PRId64 " %8s | %s\n",
               info->name, stat->count, stat->min_us,
               stat->sum_us / (int64_t)stat->count, stat->max_us, max_at, info->meaning);
    }
}

static void print_gauges(void)
{
    printf("[G] peaks and counters\n");
    for (int i = 0; i < (int)TT_GAUGE_COUNT; ++i) {
        printf("  %-22s %8" PRIu32 " | %s\n",
               s_gauge_info[i].name, s_round.gauges[i], s_gauge_info[i].meaning);
    }
}

void timing_trace_print(void)
{
    /* printf 而非 ESP_LOG：整段沒有 tag 前綴，方便直接複製出來比對 */
    printf("====start=====\n");
    print_timeline();
    print_stats();
    print_gauges();
    printf("====end=====\n");
}

#endif /* APP_TIMING_TRACE */
