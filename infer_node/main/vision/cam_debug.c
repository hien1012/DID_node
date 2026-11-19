/*
 * 相機除錯 dump：正式推論路徑的一個 ROI，經 console (UART log) 輸出 CNN 每一層。
 * 說明見 cam_debug.h；輸出格式 (@@DUMP / @@FILE / @@D) 與 nn_index.csv 見 debug_dump.c。
 *
 * 檔案 (全部 little-endian 原始二進位)：
 *   roi.gray        64 x 64 uint8   正式路徑實際送進 CNN 的 ROI (row-major)
 *   nn_index.csv                    每個張量的索引、型別、shape、量化參數、檔名、名稱
 *   nn_tNNN.bin                     每個非常數張量 (各層輸出) 的原始資料，合計約 156 KB
 *   meta.json                       正式 / 重跑 / 除錯 interpreter 的機率、時間、模型長度、ESP-NN 設定
 */
#include "cam_debug.h"

#include <stdbool.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "debug_dump.h"

/* main/CMakeLists.txt 由 ESP_NN_KERNEL_SET cache 變數帶進來 */
#ifndef INFER_ESP_NN_KERNEL_SET
#define INFER_ESP_NN_KERNEL_SET "unknown"
#endif

static const char *TAG = "CAM_DEBUG";

/* 只在 cam_infer task 內使用；放 static 不佔它的 stack */
static char s_meta[1536];

static const char *label_of(int cls)
{
    return cls >= 0 ? cam_model_class_name(cls) : "";
}

esp_err_t cam_debug_dump_roi(const cam_debug_roi_t *r)
{
    if (r == NULL || r->roi == NULL || r->probs == NULL || r->bbox == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const int64_t started_us = esp_timer_get_time();
    const bool replay = r->replay_name != NULL;
    char stem[96];
    if (replay) {
        snprintf(stem, sizeof(stem), "%.63s_%s", r->replay_name, INFER_ESP_NN_KERNEL_SET);
    } else {
        snprintf(stem, sizeof(stem), "r%05u_%lld", (unsigned)r->index, (long long)r->ts_epoch_ms);
    }

    /*
     * 正式 interpreter 對同一個 ROI 再跑一次：音訊模型的 ESP-NN avg pool 曾經同一輸入兩次結果不同
     * (docs/debug_2026_11_17.md 第 4 項)，這裡直接檢查相機模型有沒有這種情形。
     */
    float repeat_probs[CAM_MODEL_CLASS_COUNT] = {0};
    int repeat_pred = -1;
    const esp_err_t repeat_err = cam_model_predict(r->roi, repeat_probs, &repeat_pred);
    const uint32_t repeat_invoke_us = cam_model_last_invoke_us();

    ESP_LOGW(TAG, "Dumping ROI %u (frame %u track %d) over the console as %s (~20 s at 115200 baud)...",
             (unsigned)r->index, (unsigned)r->frame, (int)r->track_id, stem);
    debug_dump_stats_t stats;
    debug_dump_begin(&stats, stem);
    debug_dump_file(&stats, "roi.gray", r->roi, CAM_MODEL_ROI_BYTES);

    /* 保留所有張量的 interpreter，輸入 = 正式路徑實際送進 CNN 的 ROI；每一層在 callback 裡直接輸出 */
    debug_dump_index_reset();
    float dump_probs[CAM_MODEL_CLASS_COUNT] = {0};
    int dump_pred = -1;
    size_t arena_used = 0;
    const esp_err_t dump_err = cam_model_debug_run(r->roi, dump_probs, &dump_pred, &arena_used,
                                                   debug_dump_tensor, &stats);
    if (dump_err != ESP_OK || repeat_err != ESP_OK) {
        ++stats.errors;
    }
    debug_dump_index_emit(&stats);

    const int length = snprintf(
        s_meta, sizeof(s_meta),
        "{\n"
        " \"source\": \"%s\",\n"
        " \"kind\": \"camera_roi\",\n"
        " \"input_file\": \"%.63s\",\n"
        " \"roi_index\": %u,\n"
        " \"frame\": %u,\n"
        " \"track_id\": %d,\n"
        " \"bbox\": [%d, %d, %d, %d],\n"
        " \"ts_epoch_ms\": %lld,\n"
        " \"roi_size\": %d,\n"
        " \"model_bytes\": %u,\n"
        " \"esp_nn_kernel_set\": \"%s\",\n"
        " \"live_invoke_us\": %u,\n"
        " \"prod_pred\": \"%s\",\n"
        " \"prod_probs\": [%.6f, %.6f, %.6f],\n"
        " \"prod_repeat_ok\": %s,\n"
        " \"prod_repeat_invoke_us\": %u,\n"
        " \"prod_repeat_pred\": \"%s\",\n"
        " \"prod_repeat_probs\": [%.6f, %.6f, %.6f],\n"
        " \"dump_model_ok\": %s,\n"
        " \"dump_pred\": \"%s\",\n"
        " \"dump_probs\": [%.6f, %.6f, %.6f],\n"
        " \"dump_arena_used\": %u,\n"
        " \"labels\": [\"%s\", \"%s\", \"%s\"]\n"
        "}\n",
        replay ? "sd_replay" : "live_capture", replay ? r->replay_name : "",
        (unsigned)r->index, (unsigned)r->frame, (int)r->track_id,
        (int)r->bbox[0], (int)r->bbox[1], (int)r->bbox[2], (int)r->bbox[3],
        (long long)r->ts_epoch_ms, CAM_MODEL_ROI_SIZE, (unsigned)cam_model_bytes(),
        INFER_ESP_NN_KERNEL_SET, (unsigned)r->invoke_us,
        label_of(r->pred), (double)r->probs[0], (double)r->probs[1], (double)r->probs[2],
        repeat_err == ESP_OK ? "true" : "false", (unsigned)repeat_invoke_us, label_of(repeat_pred),
        (double)repeat_probs[0], (double)repeat_probs[1], (double)repeat_probs[2],
        dump_err == ESP_OK ? "true" : "false", label_of(dump_pred),
        (double)dump_probs[0], (double)dump_probs[1], (double)dump_probs[2], (unsigned)arena_used,
        cam_model_class_name(0), cam_model_class_name(1), cam_model_class_name(2));
    if (length > 0 && length < (int)sizeof(s_meta)) {
        debug_dump_file(&stats, "meta.json", s_meta, (size_t)length);
    } else {
        ++stats.errors;
    }
    debug_dump_end(&stats, stem);

    ESP_LOGW(TAG, "ROI %u: live=%s [%.3f %.3f %.3f] repeat=%s [%.3f %.3f %.3f] debug_interp=%s "
             "[%.3f %.3f %.3f] ESP-NN=%s files=%d errors=%d (%.1f s)",
             (unsigned)r->index, r->pred >= 0 ? label_of(r->pred) : "?",
             (double)r->probs[0], (double)r->probs[1], (double)r->probs[2],
             repeat_pred >= 0 ? label_of(repeat_pred) : "?",
             (double)repeat_probs[0], (double)repeat_probs[1], (double)repeat_probs[2],
             dump_pred >= 0 ? label_of(dump_pred) : "?",
             (double)dump_probs[0], (double)dump_probs[1], (double)dump_probs[2],
             INFER_ESP_NN_KERNEL_SET, stats.files, stats.errors,
             (double)(esp_timer_get_time() - started_us) / 1e6);
    return stats.errors == 0 ? ESP_OK : ESP_FAIL;
}
