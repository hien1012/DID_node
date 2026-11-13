#pragma once

#include <stdint.h>
#include <sys/time.h>

#include "esp_timer.h"

/* esp_timer 時間點 (微秒) 換算成 epoch 毫秒；時鐘同步後才有意義。 */
static inline int64_t infer_epoch_ms_from_timer(int64_t timer_us)
{
    struct timeval now = {0};
    gettimeofday(&now, NULL);
    const int64_t wall_us = (int64_t)now.tv_sec * 1000000LL + now.tv_usec;
    return (wall_us - (esp_timer_get_time() - timer_us)) / 1000LL;
}
