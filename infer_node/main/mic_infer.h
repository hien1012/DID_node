#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint32_t windows;           /* 推論完成的視窗數 */
    uint32_t dropped_windows;   /* 三塊緩衝都在忙 (推論或寫 SD 來不及) 而丟掉資料的次數 */
    uint32_t low_conf;
    uint32_t saved;
    uint32_t overflows;         /* I2S DMA overflow 累計 */
    uint32_t last_feature_us;
    uint32_t last_invoke_us;
} mic_infer_stats_t;

/* I2S 與模型初始化；時脈保持停止直到 mic_infer_start()。 */
esp_err_t mic_infer_init(void);

/*
 * 啟動時脈與四個常駐 task，之後一直連續錄音：
 *   mic_capture   (Core 0)：填滿 2 s 視窗就換另一塊緩衝繼續收，
 *   audio_feature (Core 0)：對剛收滿的上一個視窗算 logmel，
 *   audio_cnn     (Core 1)：CNN，結果立刻回傳，
 *   audio_writer  (Core 1)：低信賴度的視窗寫成 WAV 存 SD。
 * 要 SD 與 result_report 都準備好才呼叫。
 */
esp_err_t mic_infer_start(void);

void mic_infer_get_stats(mic_infer_stats_t *stats);

/*
 * 等錄音管線的空檔：收滿的視窗都已推論、寫完 SD，且離下一個視窗收滿還有 300 ms 以上。
 * 回傳 true = 空檔；等超過 max_wait_ms 回傳 false，呼叫端照樣繼續
 * (推論一直沒有空檔時，上傳變慢但不會停擺)。給上傳在每段讀 SD 前呼叫，mic_infer_start() 之後才有效。
 */
bool mic_infer_wait_gap(uint32_t max_wait_ms);
