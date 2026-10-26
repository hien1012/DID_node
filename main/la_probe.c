#include "la_probe.h"

#if APP_LA_PROBE_ENABLE

/* 必須在 FreeRTOS 標頭之前：它定義 traceTASK_SWITCHED_IN/OUT，FreeRTOS.h 才不會
 * 先定義成空巨集而造成重複定義。這裡只用到 la_probe_task_tcb[]。 */
#include "la_task_trace.h"

#include <stddef.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"

#include "board_pins.h"

static const char *TAG = "LA_PROBE";

/* 換表時關中斷，避免本核心在一半時切換 task */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static const int s_task_pins[LA_TASK_SLOT_COUNT] = {
    LA_PIN_TASK0, LA_PIN_TASK1, LA_PIN_TASK2, LA_PIN_TASK3,
};

/* 索引對應 s_task_pins；名稱就是 xTaskCreate 時給的名稱 */
static const char *const s_record_tasks[LA_TASK_SLOT_COUNT] = {
    "cam_task", "cam_writer", "mic_capture", "mic_writer",
};
static const char *const s_upload_tasks[LA_TASK_SLOT_COUNT] = {
    "wifi", "main", "upload_reader", "tiT",
};

esp_err_t la_probe_init(void)
{
    uint64_t mask = (1ULL << LA_PIN_IDLE_CPU0) | (1ULL << LA_PIN_IDLE_CPU1);
    for (size_t i = 0; i < LA_TASK_SLOT_COUNT; ++i) {
        mask |= 1ULL << s_task_pins[i];
    }
    const gpio_config_t config = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "gpio_config failed");
    for (size_t i = 0; i < LA_TASK_SLOT_COUNT; ++i) {
        la_probe_task_tcb[i] = NULL;
        gpio_ll_set_level(&GPIO, (uint32_t)s_task_pins[i], 0);
    }
    la_probe_core_pin[0] = 0;
    la_probe_core_pin[1] = 0;

    ESP_LOGW(TAG, "LA probes on: idle CPU0=GPIO%d CPU1=GPIO%d, task pins GPIO%d/%d/%d/%d",
             LA_PIN_IDLE_CPU0, LA_PIN_IDLE_CPU1,
             LA_PIN_TASK0, LA_PIN_TASK1, LA_PIN_TASK2, LA_PIN_TASK3);
    return ESP_OK;
}

void la_probe_set_phase(la_phase_t phase)
{
    const char *const *names = NULL;
    if (phase == LA_PHASE_RECORD) {
        names = s_record_tasks;
    } else if (phase == LA_PHASE_UPLOAD) {
        names = s_upload_tasks;
    }

    /* xTaskGetHandle() 會暫停排程器並走訪所有 task，不能放在 critical section 裡，先查好 */
    TaskHandle_t tasks[LA_TASK_SLOT_COUNT] = {NULL};
    for (size_t i = 0; names != NULL && i < LA_TASK_SLOT_COUNT; ++i) {
        tasks[i] = xTaskGetHandle(names[i]);
        if (tasks[i] == NULL) {
            ESP_LOGW(TAG, "Task \"%s\" not found; GPIO%d stays low", names[i], s_task_pins[i]);
        }
    }

    /*
     * 關中斷 (本核心不會切換 task) 下換表：全部解除、拉低、清掉各核心記錄的腳。
     * 另一核心若正好在跑某個綁定 task，它的腳要等下次切入才會拉高 (少記一小段)。
     * 呼叫者自己正在跑，切入 hook 要等下次，這裡直接補上並記到本核心。
     */
    const TaskHandle_t self = xTaskGetCurrentTaskHandle();
    taskENTER_CRITICAL(&s_lock);
    for (size_t i = 0; i < LA_TASK_SLOT_COUNT; ++i) {
        la_probe_task_tcb[i] = NULL;
        gpio_ll_set_level(&GPIO, (uint32_t)s_task_pins[i], 0);
    }
    la_probe_core_pin[0] = 0;
    la_probe_core_pin[1] = 0;
    for (size_t i = 0; i < LA_TASK_SLOT_COUNT; ++i) {
        la_probe_task_tcb[i] = tasks[i];
        if (tasks[i] != NULL && tasks[i] == self) { //若還是目前的task就不拉低gpio
            gpio_ll_set_level(&GPIO, (uint32_t)s_task_pins[i], 1);
            la_probe_core_pin[xPortGetCoreID()] = (uint8_t)s_task_pins[i];
        }
    }
    taskEXIT_CRITICAL(&s_lock);
}

#else

esp_err_t la_probe_init(void)
{
    return ESP_OK;
}

void la_probe_set_phase(la_phase_t phase)
{
    (void)phase;
}

#endif
