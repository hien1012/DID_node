#pragma once

/*
 * Debug：C ISR 分派器攔截與各別中斷 CPU 負擔統計 (APP_ISR_PROFILE_ENABLE)。
 *
 * 原理 (不改 ESP-IDF)：
 *  1. 以 -Wl,--wrap=esp_intr_alloc_intrstatus 攔截所有經由 IDF 註冊的中斷
 *     (含 Shared Interrupt 的個別周邊，直接取得 ETS_*_INTR_SOURCE 編號)。
 *  2. 以 -Wl,--wrap=xt_set_interrupt_handler 攔截不經 esp_intr_alloc 直接寫
 *     _xt_interrupt_table 的中斷 (例如 Wi-Fi MAC Int 0、FreeRTOS SysTick Int 6、
 *     跨核心 Software Int 7/29 等)。
 *  3. 每個 C ISR 被替換為 IRAM 中的通用 trampoline，以 rsr.ccount (1 cycle)
 *     量測每次執行耗時並依 Core 0 / Core 1 與中斷源累計。
 *
 * 限制：Level 4..7 高優先權組語中斷與 NMI 不走 C handler，不在統計內。
 */

#include "app_config.h"

#if APP_ISR_PROFILE_ENABLE

/* 歸零所有 ISR 統計計數器，並記下當前時間作為統計視窗起點 */
void isr_profile_reset(void);

/* 印出從上次 reset() 到現在，Core 0 與 Core 1 各個 ISR 的次數、耗時與 CPU 負擔 % */
void isr_profile_print(const char *phase_name);

#else

static inline void isr_profile_reset(void) {}
static inline void isr_profile_print(const char *phase_name)
{
    (void)phase_name;
}

#endif

