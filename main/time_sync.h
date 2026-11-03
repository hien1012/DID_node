#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

esp_err_t time_sync_from_server(void);
bool time_sync_is_valid(void);
void time_sync_configure_timezone(void);
/* 距離上次校時超過 APP_TIME_RESYNC_SEC 才重新向 server 校時；在錄製之間呼叫 */
void time_sync_refresh_if_due(void);
/* 阻塞到下一個 epoch 為 period_sec 倍數的時間點（至少 min_lead_sec 之後），回傳該時間點。
 * 已經開始的那一秒不算，錯過的時間點不補錄。 */
time_t time_sync_wait_next_slot(uint32_t period_sec, uint32_t min_lead_sec);
