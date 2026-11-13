/*
 * 推論韌體的 SD 掛載與共用 I/O 緩衝 (DID_NODE/main/node_storage.c 只留推論用到的部分)。
 * 掛載參數與錄影韌體的 main/cam_storage.c 相同：TTGO T8 板上 microSD 走 SDMMC slot 1
 * (CLK 14 / CMD 15 / D0 2，IO MUX 固定腳位)，1-bit、40 MHz。
 */
#include "node_storage.h"

#include <stdbool.h>
#include <stdio.h>

#include "driver/sdmmc_host.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

/* 樣本寫入 (資料 + meta) 與上傳讀取 (資料 + meta) 可能同時開著 */
#define STORAGE_MAX_OPEN_FILES   8
#define STORAGE_SD_FREQ_KHZ      SDMMC_FREQ_HIGHSPEED   /* 40 MHz，同 cam_storage.c */

static const char *TAG = "NODE_STORAGE";
static sdmmc_card_t *s_sd_card = NULL;
static bool s_mounted = false;

/* static 放內部 DRAM (ESP32 上可 DMA)；不從 heap 要，避免開機後內部 RAM 碎裂時配置失敗 */
static WORD_ALIGNED_ATTR uint8_t s_io_buffer[NODE_STORAGE_IO_BUFFER_BYTES];
static const char *s_io_buffer_owner = NULL;
static portMUX_TYPE s_io_buffer_lock = portMUX_INITIALIZER_UNLOCKED;

_Static_assert((NODE_STORAGE_IO_BUFFER_BYTES % 512U) == 0U,
               "The I/O buffer must hold whole SD sectors");

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

esp_err_t node_storage_init(void)
{
    if (s_mounted) {
        return ESP_OK;
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
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    const esp_err_t err = esp_vfs_fat_sdmmc_mount(NODE_STORAGE_MOUNT_POINT, &host, &slot,
                                                  &mount_config, &s_sd_card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(err));
        return err;
    }
    sdmmc_card_print_info(stdout, s_sd_card);
    s_mounted = true;
    ESP_LOGI(TAG, "SD mounted at %s (1-bit, %u kHz)", NODE_STORAGE_MOUNT_POINT,
             (unsigned)STORAGE_SD_FREQ_KHZ);
    return ESP_OK;
}

esp_err_t node_storage_get_free_bytes(uint64_t *free_bytes_out)
{
    if (!free_bytes_out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_mounted) {
        return ESP_ERR_INVALID_STATE;
    }
    uint64_t total_bytes = 0;
    return esp_vfs_fat_info(NODE_STORAGE_MOUNT_POINT, &total_bytes, free_bytes_out);
}
