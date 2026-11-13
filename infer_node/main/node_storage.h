#pragma once

/*
 * 推論韌體用的 SD 掛載與共用 I/O 緩衝。介面與 DID_NODE/main/node_storage.h 的同名函式相同，
 * 所以從 infer_node (S3) 照搬的 infer_app.c / infer_health.c 不必修改；錄影用的
 * undone/ done/ broken/ 這裡沒有 (樣本放在 /sdcard/samples/，由 sample_store.c 管理)。
 */
#include <stdint.h>

#include "esp_err.h"

#define NODE_STORAGE_MOUNT_POINT "/sdcard"

/** 掛載 SD (SDMMC slot 1，1-bit，/sdcard)。 */
esp_err_t node_storage_init(void);

esp_err_t node_storage_get_free_bytes(uint64_t *free_bytes_out);

/*
 * cam_writer 暫存區 (前半) 與上傳讀檔緩衝 (後半) 共用的 32 KB 內部 DMA 緩衝。
 * 4-byte 對齊、位於內部 RAM，SDMMC 可直接 DMA。同一時間只能有一個持有者；
 * 已被持有時 acquire 回傳 NULL。
 */
#define NODE_STORAGE_IO_BUFFER_BYTES (32U * 1024U)
uint8_t *node_storage_io_buffer_acquire(const char *owner);
void node_storage_io_buffer_release(const uint8_t *buffer);
