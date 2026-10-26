#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

/*
 * SD layout:
 *   /sdcard/undone/  file being recorded (left over after power loss -> broken)
 *   /sdcard/done/    complete recordings waiting for upload, deleted after ACK
 *   /sdcard/broken/  damaged or rejected recordings, kept on the card
 * Names: YYYY_MM_DD_HHMMSS.raw / .wav, broken ones YYYY_MM_DD_HHMMSS_broken.raw
 */

#define NODE_STORAGE_PATH_MAX 96U

typedef enum {
    NODE_MEDIA_VIDEO = 0,   /* GRAY8 RAW, .raw */
    NODE_MEDIA_AUDIO,       /* 16-bit PCM WAV, .wav */
} node_media_t;

typedef struct {
    node_media_t media;
    time_t recorded_at_epoch;
    char undone_path[NODE_STORAGE_PATH_MAX];
    char done_path[NODE_STORAGE_PATH_MAX];
    char broken_path[NODE_STORAGE_PATH_MAX];
    int fd;
    bool active;
} node_recording_t;

typedef struct {
    char path[NODE_STORAGE_PATH_MAX];
    node_media_t media;
    time_t recorded_at_epoch;
    size_t file_bytes;
} node_pending_file_t;

/** Mount the card, create the directories and mark interrupted files broken. */
esp_err_t node_storage_init(void);

/** Create the file in undone/ and preallocate expected_bytes. */
esp_err_t node_storage_begin_recording(node_recording_t *recording,
                                       node_media_t media,
                                       time_t recorded_at_epoch,
                                       size_t expected_bytes);

/** Close the file and move it to done/ (upload) or broken/. */
esp_err_t node_storage_finish_recording(node_recording_t *recording,
                                        bool upload);

/** Oldest file in done/; audio before video for the same session. */
esp_err_t node_storage_get_next_pending(node_pending_file_t *pending);
esp_err_t node_storage_delete_uploaded(const char *path);
/** Move a done/ file the server permanently rejected to broken/. */
esp_err_t node_storage_mark_rejected(const node_pending_file_t *pending);
esp_err_t node_storage_count(size_t *pending_out, size_t *broken_out);
esp_err_t node_storage_get_file_size(const char *path, size_t *size_out);
esp_err_t node_storage_get_free_bytes(uint64_t *free_bytes_out);
const char *node_storage_media_name(node_media_t media);

/*
 * 錄影 (cam_writer 暫存區) 與上傳 (讀檔雙緩衝) 共用的 32 KB 內部 DMA 緩衝。
 * 兩者不會同時進行，所以共用一塊省下約 32 KB 內部 RAM (debug_sdcard #24)。
 * 4-byte 對齊、位於內部 RAM，SDMMC 可直接 DMA。同一時間只能有一個持有者；
 * 已被持有時 acquire 回傳 NULL。
 */
#define NODE_STORAGE_IO_BUFFER_BYTES (32U * 1024U)
uint8_t *node_storage_io_buffer_acquire(const char *owner);
void node_storage_io_buffer_release(const uint8_t *buffer);
