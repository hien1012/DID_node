#pragma once

/*
 * Debug：邏輯分析儀 task 量測腳 (APP_LA_PROBE_ENABLE)，在 FreeRTOS 切換 task 時更新：
 *  - LA_PIN_IDLE_CPU0/1：該核心正在執行 IDLE task 時為高 → 高電位比例 = 空閒比例
 *  - LA_PIN_TASK0..3：綁定的 task 正在 CPU 上執行時為高 (綁定哪些 task 由
 *    la_probe_set_phase() 依錄製 / 上傳階段切換)
 *
 * 由頂層 CMakeLists.txt 以 -include 強制加進 freertos 元件的 C 檔 (不改 IDF)；
 * traceTASK_SWITCHED_IN 只在 tasks.c 的 vTaskSwitchContext() / vTaskStartScheduler()
 * 展開，那裡看得到 pxCurrentTCB[] 與 static 的 xIdleTaskHandle[]。
 * traceTASK_SWITCHED_OUT 不用 (見 la_probe_core_pin 的說明)。
 *
 * 限制：沒有 ISR hook，task (含 IDLE) 執行期間被中斷的時間仍算在該 task。
 * 只寫 GPIO W1TS/W1TC 暫存器，不呼叫函式 (排程器內、可能在 cache 關閉時執行)。
 */
#include "app_config.h"
#include "board_pins.h"

#define LA_TASK_SLOT_COUNT 4

#if APP_LA_PROBE_ENABLE && !CONFIG_APPTRACE_SV_ENABLE

#include <stddef.h>
#include <stdint.h>
#include "soc/soc.h"
#include "soc/gpio_reg.h"

/*
 * 綁定的 task (TCB 位址)，索引對應 LA_PIN_TASK0..3；NULL = 該腳不用。
 * weak：每個 include 此檔的 .c 都有一份定義，連結時合併成同一個；
 * la_probe.c 寫入、tasks.c 的切換 hook 讀取。放在 DRAM，cache 關閉時也可讀。
 */
__attribute__((weak)) void *volatile la_probe_task_tcb[LA_TASK_SLOT_COUNT];

/*
 * 每顆核心目前拉高的 task 腳 (0 = 沒有；GPIO0 不會是 task 腳)。
 * vTaskSwitchContext() 每次都會走一遍選擇，即使最後選回同一個 task (例如
 * time slicing 時同優先權 ready list 裡有另一核心的 task)。只在 IN 比對新舊，
 * 腳位真的換人才動，才不會在「選回同一個 task」時產生假的低電位缺口。
 * 只有該核心的切換 hook 與 la_probe_set_phase() (critical section 內) 會寫入。
 */
__attribute__((weak)) volatile uint8_t la_probe_core_pin[2];

/* pin < 32 用 GPIO_OUT_*，32..48 用 GPIO_OUT1_* */
#define LA_PIN_SET(pin, level)                                                             \
    REG_WRITE(((pin) < 32) ? ((level) ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG)             \
                           : ((level) ? GPIO_OUT1_W1TS_REG : GPIO_OUT1_W1TC_REG),          \
              1UL << ((pin) & 31))

/* tcb 綁定的 task 腳；沒有綁定回傳 0 */
static inline __attribute__((always_inline)) uint32_t la_task_pin_of(const void *tcb)
{
    if (tcb == NULL) {
        return 0;
    }
    if (tcb == la_probe_task_tcb[0]) {
        return LA_PIN_TASK0;
    }
    if (tcb == la_probe_task_tcb[1]) {
        return LA_PIN_TASK1;
    }
    if (tcb == la_probe_task_tcb[2]) {
        return LA_PIN_TASK2;
    }
    if (tcb == la_probe_task_tcb[3]) {
        return LA_PIN_TASK3;
    }
    return 0;
}

/* 在 taskSELECT_HIGHEST_PRIORITY_TASK() 之後執行：pxCurrentTCB 已是新 task */
#define traceTASK_SWITCHED_IN()                                                            \
    do {                                                                                   \
        const BaseType_t la_core = xPortGetCoreID();                                       \
        const uint32_t la_idle_pin = la_core ? LA_PIN_IDLE_CPU1 : LA_PIN_IDLE_CPU0;        \
        if (pxCurrentTCB[la_core] == xIdleTaskHandle[la_core]) {                           \
            LA_PIN_SET(la_idle_pin, 1);                                                    \
        } else {                                                                           \
            LA_PIN_SET(la_idle_pin, 0);                                                    \
        }                                                                                  \
        const uint32_t la_new = la_task_pin_of(pxCurrentTCB[la_core]);                     \
        const uint32_t la_old = la_probe_core_pin[la_core];                                \
        if (la_new != la_old) {                                                            \
            if (la_old != 0) {                                                             \
                LA_PIN_SET(la_old, 0);                                                     \
            }                                                                              \
            if (la_new != 0) {                                                             \
                LA_PIN_SET(la_new, 1);                                                     \
            }                                                                              \
            la_probe_core_pin[la_core] = (uint8_t)la_new;                                  \
        }                                                                                  \
    } while (0)

#endif
