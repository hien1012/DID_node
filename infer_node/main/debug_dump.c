/*
 * 除錯 dump 的共用輸出 (說明見 debug_dump.h)。
 *
 * 輸出格式 (每行一個 printf，與其他 task 的 log 交錯也不影響)：
 *   @@DUMP BEGIN <stem>
 *   @@FILE <name> <bytes> <crc32 hex>          crc32 = zlib crc32
 *   @@D <seq> <base64>                          每行 96 bytes 原始資料
 *   @@DUMP END <stem> <files> <errors>
 *
 * nn_index.csv：每個張量一行
 *   index,type,shape,quantized,scale,zero_point,bytes,file,name
 * file = nn_tNNN.bin (非常數張量) / const (權重) / 空 (沒有資料)；name 加引號放最後 (可能含逗號)。
 */
#include "debug_dump.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DUMP_BYTES_PER_LINE   96U     /* -> 128 base64 字元 */
#define DUMP_YIELD_EVERY      16U     /* 每 16 行讓出一次 CPU，IDLE task 才餵得到 task watchdog */
/* 相機模型 44 個張量、名稱最長約 190 字元，約 4.6 KB；放 PSRAM，第一次用時配置 */
#define DUMP_INDEX_BYTES      (16U * 1024U)

static const char *TAG = "DEBUG_DUMP";

/* 只在 dump 的 task 內使用；放 static 不佔呼叫端的 stack */
static char s_line[DUMP_BYTES_PER_LINE / 3U * 4U + 32U];
static char *s_index = NULL;
static size_t s_index_len = 0;
static bool s_index_truncated = false;

/* zlib 相容的 CRC32 (反射多項式 0xEDB88320)，PC 端以 zlib.crc32 驗證 */
static uint32_t crc32_zlib(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static size_t base64_encode(const uint8_t *src, size_t length, char *dst)
{
    static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t out = 0;
    for (size_t i = 0; i < length; i += 3) {
        const uint32_t b0 = src[i];
        const uint32_t b1 = i + 1 < length ? src[i + 1] : 0;
        const uint32_t b2 = i + 2 < length ? src[i + 2] : 0;
        const uint32_t triple = (b0 << 16) | (b1 << 8) | b2;
        dst[out++] = table[(triple >> 18) & 0x3F];
        dst[out++] = table[(triple >> 12) & 0x3F];
        dst[out++] = i + 1 < length ? table[(triple >> 6) & 0x3F] : '=';
        dst[out++] = i + 2 < length ? table[triple & 0x3F] : '=';
    }
    dst[out] = '\0';
    return out;
}

void debug_dump_begin(debug_dump_stats_t *stats, const char *stem)
{
    memset(stats, 0, sizeof(*stats));
    printf("@@DUMP BEGIN %s\n", stem);
}

void debug_dump_file(debug_dump_stats_t *stats, const char *name, const void *data, size_t bytes)
{
    if (data == NULL) {
        ++stats->errors;
        printf("@@ERROR %s missing\n", name);
        return;
    }
    const uint8_t *p = (const uint8_t *)data;
    printf("@@FILE %s %u %08lx\n", name, (unsigned)bytes, (unsigned long)crc32_zlib(p, bytes));
    uint32_t seq = 0;
    for (size_t offset = 0; offset < bytes; offset += DUMP_BYTES_PER_LINE, ++seq) {
        const size_t chunk = bytes - offset < DUMP_BYTES_PER_LINE ? bytes - offset : DUMP_BYTES_PER_LINE;
        base64_encode(p + offset, chunk, s_line);
        printf("@@D %lu %s\n", (unsigned long)seq, s_line);
        if ((++stats->lines % DUMP_YIELD_EVERY) == 0U) {
            vTaskDelay(1);
        }
    }
    ++stats->files;
}

void debug_dump_end(const debug_dump_stats_t *stats, const char *stem)
{
    printf("@@DUMP END %s %d %d\n", stem, stats->files, stats->errors);
}

static void index_append(const char *format, ...) __attribute__((format(printf, 1, 2)));

static void index_append(const char *format, ...)
{
    if (s_index == NULL || s_index_len >= DUMP_INDEX_BYTES) {
        s_index_truncated = true;
        return;
    }
    va_list args;
    va_start(args, format);
    const int written = vsnprintf(s_index + s_index_len, DUMP_INDEX_BYTES - s_index_len, format, args);
    va_end(args);
    if (written < 0) {
        s_index_truncated = true;
        return;
    }
    s_index_len += (size_t)written;
    if (s_index_len >= DUMP_INDEX_BYTES) {
        s_index_len = DUMP_INDEX_BYTES - 1U;
        s_index_truncated = true;
    }
}

void debug_dump_index_reset(void)
{
    if (s_index == NULL) {
        s_index = heap_caps_malloc(DUMP_INDEX_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_index == NULL) {
            ESP_LOGE(TAG, "nn_index.csv buffer allocation failed (%u bytes)", (unsigned)DUMP_INDEX_BYTES);
        }
    }
    s_index_len = 0;
    s_index_truncated = false;
    index_append("index,type,shape,quantized,scale,zero_point,bytes,file,name\n");
}

void debug_dump_tensor(const debug_tensor_info_t *tensor, void *ctx)
{
    debug_dump_stats_t *stats = (debug_dump_stats_t *)ctx;
    char file_name[24] = "";
    if (!tensor->is_const && tensor->data != NULL && tensor->bytes > 0) {
        if (snprintf(file_name, sizeof(file_name), "nn_t%03d.bin", tensor->index) < (int)sizeof(file_name)) {
            debug_dump_file(stats, file_name, tensor->data, tensor->bytes);
        } else {
            ++stats->errors;
            file_name[0] = '\0';
        }
    }
    index_append("%d,%s,", tensor->index, tensor->type_name);
    for (int d = 0; d < tensor->dims_count && d < DEBUG_TENSOR_MAX_DIMS; ++d) {
        index_append(d ? "x%d" : "%d", tensor->dims[d]);
    }
    /* 名稱可能含逗號：放最後一欄並加引號 */
    index_append(",%d,%.9g,%d,%u,%s,\"%s\"\n", tensor->quantized ? 1 : 0, (double)tensor->scale,
                 tensor->zero_point, (unsigned)tensor->bytes, tensor->is_const ? "const" : file_name,
                 tensor->name);
}

void debug_dump_index_emit(debug_dump_stats_t *stats)
{
    if (s_index_truncated) {
        /* 少了幾行的索引會讓 PC 端誤判成「張量沒輸出」，所以算錯誤 */
        ++stats->errors;
        printf("@@ERROR nn_index.csv truncated at %u bytes\n", (unsigned)s_index_len);
    }
    debug_dump_file(stats, "nn_index.csv", s_index, s_index_len);
}
