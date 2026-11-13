#pragma once

/*
 * 每個 task 用掉多少 CPU，兩種統計各自累積、互不影響：
 *   推論期間：task_usage_begin() / task_usage_end() 之間，只含推論那一段 (回傳 server)。
 *   一輪：task_usage_cycle_mark() 到下一次 mark，連續不間斷 (麥克風一個 2 s 視窗、相機每 10 幀 = 2 s)，
 *        包含推論以外的 FD、寫 SD、上傳、回報與閒置時間，看整體忙碌程度 (只印 log)。
 *
 * 用 FreeRTOS 內建的 run time stats：區間前後各抓一次 uxTaskGetSystemState()，
 * 每個 task 的 ulRunTimeCounter 相減，累積多次後取平均 (vTaskGetRunTimeStats 只有開機以來的累計)。
 * 呼叫規則：begin / end / report 內部沒有鎖，不可同時執行：通常由同一個 task 呼叫；
 * 分在不同 task 時 (麥克風：begin 在 audio_feature、end / report 在 audio_cnn) 由呼叫端保證先後。
 * cycle_mark 只能由同一個 task 呼叫，可以和推論是不同 task；cycle_report 可以在任何 task 呼叫 (內部有鎖)。
 */

/** 推論開始前呼叫。 */
void task_usage_begin(void);

/** 推論成功結束後呼叫；推論失敗就不要呼叫，那一次不列入。 */
void task_usage_end(void);

/**
 * @brief 回傳 server 並印出上次回報以來的平均，然後歸零；還沒有資料時不做事。
 * @param media "video" 或 "audio"，常數字串
 */
void task_usage_report(const char *media);

/** 結束上一輪並開始下一輪 (第一次呼叫只開始)；每輪的起點呼叫一次，例如收到新視窗時。不印 log，可以放在有時限的 task。 */
void task_usage_cycle_mark(void);

/**
 * @brief 印出上次回報以來平均每輪各 task 的 CPU 時間、占比與兩顆核心的忙碌度，然後歸零。
 *        只印 log，不回傳 server；還沒有完整的一輪時不做事。
 * @param label log 上的名稱，例如 "audio cycle"
 */
void task_usage_cycle_report(const char *label);
