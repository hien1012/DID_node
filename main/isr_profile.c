#include "isr_profile.h"

#if (APP_ISR_PROFILE_ENABLE) != defined(ISR_PROFILE_WRAP)
#error "ISR_PROFILE_WRAP does not match APP_ISR_PROFILE_ENABLE: check main/CMakeLists.txt"
#endif

#if APP_ISR_PROFILE_ENABLE

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "soc/periph_defs.h"
#include "soc/soc.h"
#include "xtensa/xtensa_api.h"

#define MAX_PROFILED_ISRS 48U
#define CPU_CORES         portNUM_PROCESSORS
#define SRC_DIRECT_BASE   (-200) /* 直接呼叫 xt_set_interrupt_handler 的中斷源標記 */

typedef struct {
    intr_handler_t real_handler;
    void          *real_arg;
    int16_t        source;       /* ETS_*_INTR_SOURCE，或 SRC_DIRECT_BASE - intno */
    uint8_t        cpu;          /* 註冊所在的 Core (0 或 1) */
    uint8_t        intno;        /* CPU 中斷號 (0..31)，255 = shared 子項目尚未填入 */
    uint8_t        is_shared;    /* 1 = 掛在 shared_intr_isr 下的個別周邊 handler */
    uint32_t       count;
    uint64_t       total_cycles;
    uint32_t       max_cycles;
} isr_slot_t;

static DRAM_ATTR isr_slot_t s_slots[MAX_PROFILED_ISRS];
static DRAM_ATTR uint32_t   s_slot_count = 0;
static int64_t              s_window_start_us = 0;

/* --------------------------------------------------------------------------
 * 通用 Trampoline (必須在 IRAM)：所有被攔截的 C ISR 都會經過這裡計時
 * -------------------------------------------------------------------------- */
static void IRAM_ATTR isr_trampoline(void *arg)
{
    isr_slot_t *slot = (isr_slot_t *)arg;

    const uint32_t start_cc = esp_cpu_get_cycle_count();
    slot->real_handler(slot->real_arg);
    const uint32_t elapsed_cc = esp_cpu_get_cycle_count() - start_cc;

    slot->count++;
    slot->total_cycles += elapsed_cc;
    if (elapsed_cc > slot->max_cycles) {
        slot->max_cycles = elapsed_cc;
    }
}

/* 尋找可重用的槽位 (同一 Core、同一 handler 與 source 重複註冊時重用) */
static isr_slot_t * IRAM_ATTR alloc_or_find_slot(uint8_t cpu, int16_t source,
                                                  uint8_t intno, uint8_t is_shared,
                                                  intr_handler_t handler, void *arg)
{
    for (uint32_t i = 0; i < s_slot_count; ++i) {
        isr_slot_t *s = &s_slots[i];
        if (s->cpu == cpu && s->source == source &&
            s->real_handler == handler && s->real_arg == arg) {
            if (intno != 255U) {
                s->intno = intno;
            }
            return s;
        }
    }
    const uint32_t idx = __atomic_fetch_add(&s_slot_count, 1U, __ATOMIC_RELAXED);
    if (idx >= MAX_PROFILED_ISRS) {
        __atomic_fetch_sub(&s_slot_count, 1U, __ATOMIC_RELAXED);
        return NULL;
    }
    isr_slot_t *slot = &s_slots[idx];
    slot->real_handler = handler;
    slot->real_arg     = arg;
    slot->source       = source;
    slot->cpu          = cpu;
    slot->intno        = intno;
    slot->is_shared    = is_shared;
    return slot;
}

/* --------------------------------------------------------------------------
 * 1. 攔截 xt_set_interrupt_handler (C 分派表 _xt_interrupt_table 的唯一寫入入口)
 *    - 涵蓋 Wi-Fi 閉源驅動 (Int 0 WMAC)、FreeRTOS SysTick 等直接寫表的中斷
 *    - 若傳入的 handler 已經是 isr_trampoline (由下面 esp_intr_alloc 包過)，或
 *      是 intr_alloc.c 內部的 shared_intr_isr，則不重複包裹。
 * -------------------------------------------------------------------------- */
extern xt_handler __real_xt_set_interrupt_handler(int n, xt_handler f, void *arg);

/* 由 __wrap_esp_intr_alloc_intrstatus 在呼叫 __real 期間設為 true，讓底層對
 * xt_set_interrupt_handler 的呼叫直接放行，避免把 shared_intr_isr 外層又包一層 */
static DRAM_ATTR volatile bool s_in_intr_alloc[CPU_CORES] = {false, false};

xt_handler IRAM_ATTR __wrap_xt_set_interrupt_handler(int n, xt_handler f, void *arg)
{
    const uint8_t cpu = (uint8_t)xPortGetCoreID();
    if (f == NULL || f == (xt_handler)&isr_trampoline || s_in_intr_alloc[cpu] ||
        n < 0 || n >= XCHAL_NUM_INTERRUPTS || Xthal_intlevel[n] > XCHAL_EXCM_LEVEL) {
        return __real_xt_set_interrupt_handler(n, f, arg);
    }

    isr_slot_t *slot = alloc_or_find_slot(
        cpu, (int16_t)(SRC_DIRECT_BASE - n), (uint8_t)n, 0U, (intr_handler_t)f, arg);
    if (slot == NULL) {
        return __real_xt_set_interrupt_handler(n, f, arg);
    }
    return __real_xt_set_interrupt_handler(n, isr_trampoline, slot);
}

/* --------------------------------------------------------------------------
 * 2. 攔截 esp_intr_alloc_intrstatus (涵蓋所有 IDF 驅動與 Shared Interrupt 子項)
 * -------------------------------------------------------------------------- */
extern esp_err_t __real_esp_intr_alloc_intrstatus(
    int source, int flags, uint32_t intrstatusreg, uint32_t intrstatusmask,
    intr_handler_t handler, void *arg, intr_handle_t *ret_handle);

esp_err_t __wrap_esp_intr_alloc_intrstatus(
    int source, int flags, uint32_t intrstatusreg, uint32_t intrstatusmask,
    intr_handler_t handler, void *arg, intr_handle_t *ret_handle)
{
    if (handler == NULL || (flags & ESP_INTR_FLAG_HIGH) != 0) {
        return __real_esp_intr_alloc_intrstatus(source, flags, intrstatusreg,
                                                intrstatusmask, handler, arg, ret_handle);
    }

    const uint8_t cpu = (uint8_t)xPortGetCoreID();
    const uint8_t is_shared = (flags & ESP_INTR_FLAG_SHARED) ? 1U : 0U;
    isr_slot_t *slot = alloc_or_find_slot(cpu, (int16_t)source, 255U, is_shared,
                                          handler, arg);
    if (slot == NULL) {
        return __real_esp_intr_alloc_intrstatus(source, flags, intrstatusreg,
                                                intrstatusmask, handler, arg, ret_handle);
    }

    intr_handle_t local_handle = NULL;
    s_in_intr_alloc[cpu] = true;
    esp_err_t err = __real_esp_intr_alloc_intrstatus(
        source, flags, intrstatusreg, intrstatusmask,
        isr_trampoline, slot, &local_handle);
    s_in_intr_alloc[cpu] = false;

    if (err == ESP_OK && local_handle != NULL) {
        slot->intno = (uint8_t)esp_intr_get_intno(local_handle);
        slot->cpu   = (uint8_t)esp_intr_get_cpu(local_handle);
        if (ret_handle != NULL) {
            *ret_handle = local_handle;
        }
    }
    return err;
}

esp_err_t __wrap_esp_intr_alloc(int source, int flags, intr_handler_t handler,
                                void *arg, intr_handle_t *ret_handle)
{
    return __wrap_esp_intr_alloc_intrstatus(source, flags, 0, 0,
                                            handler, arg, ret_handle);
}

/* --------------------------------------------------------------------------
 * 3. 中斷源名稱對照表 (ESP32-S3 periph_interrput_t + 內部中斷)
 * -------------------------------------------------------------------------- */
static const char *source_name(int16_t src, uint8_t intno)
{
    if (src <= SRC_DIRECT_BASE) {
        switch (intno) {
        case ETS_WMAC_INUM: return "WIFI_MAC (direct Int0)";
        case ETS_WBB_INUM:  return "WIFI_BB (direct Int4)";
        case 6:             return "FREERTOS_SYSTICK (Timer0)";
        case 7:             return "SW0 (Yield/Crosscore)";
        case 15:            return "TIMER1 (L3)";
        case 29:            return "SW1 (IPC)";
        default:            return "DIRECT_XT_ISR";
        }
    }
    switch (src) {
    case ETS_INTERNAL_TIMER0_INTR_SOURCE:       return "FREERTOS_SYSTICK (Timer0)";
    case ETS_INTERNAL_TIMER1_INTR_SOURCE:       return "INTERNAL_TIMER1";
    case ETS_INTERNAL_SW0_INTR_SOURCE:          return "INTERNAL_SW0";
    case ETS_INTERNAL_SW1_INTR_SOURCE:          return "INTERNAL_SW1";
    case ETS_WIFI_MAC_INTR_SOURCE:              return "WIFI_MAC";
    case ETS_WIFI_MAC_NMI_SOURCE:               return "WIFI_MAC_NMI";
    case ETS_WIFI_PWR_INTR_SOURCE:              return "WIFI_PWR";
    case ETS_WIFI_BB_INTR_SOURCE:               return "WIFI_BB";
    case ETS_GPIO_INTR_SOURCE:                  return "GPIO (Cam VSYNC)";
    case ETS_SPI2_INTR_SOURCE:                  return "SPI2";
    case ETS_SPI3_INTR_SOURCE:                  return "SPI3";
    case ETS_LCD_CAM_INTR_SOURCE:               return "LCD_CAM (Cam VSYNC)";
    case ETS_I2S0_INTR_SOURCE:                  return "I2S0 (Mic)";
    case ETS_I2S1_INTR_SOURCE:                  return "I2S1";
    case ETS_UART0_INTR_SOURCE:                 return "UART0";
    case ETS_SDIO_HOST_INTR_SOURCE:             return "SDMMC_HOST";
    case ETS_LEDC_INTR_SOURCE:                  return "LEDC";
    case ETS_USB_INTR_SOURCE:                   return "USB_OTG";
    case ETS_RTC_CORE_INTR_SOURCE:              return "RTC_CORE";
    case ETS_RMT_INTR_SOURCE:                   return "RMT";
    case ETS_I2C_EXT0_INTR_SOURCE:              return "I2C0 (Cam SCCB)";
    case ETS_I2C_EXT1_INTR_SOURCE:              return "I2C1";
    case ETS_TG0_T0_LEVEL_INTR_SOURCE:          return "TG0_T0";
    case ETS_TG0_WDT_LEVEL_INTR_SOURCE:         return "TG0_WDT";
    case ETS_TG1_T0_LEVEL_INTR_SOURCE:          return "TG1_T0";
    case ETS_TG1_WDT_LEVEL_INTR_SOURCE:         return "TG1_WDT (Int WDT)";
    case ETS_SYSTIMER_TARGET0_EDGE_INTR_SOURCE: return "SYSTIMER0 (esp_timer)";
    case ETS_SYSTIMER_TARGET1_EDGE_INTR_SOURCE: return "SYSTIMER1 (RTOS Tick0)";
    case ETS_SYSTIMER_TARGET2_EDGE_INTR_SOURCE: return "SYSTIMER2 (RTOS Tick1)";
    case ETS_DMA_IN_CH0_INTR_SOURCE:            return "GDMA_IN_CH0";
    case ETS_DMA_IN_CH1_INTR_SOURCE:            return "GDMA_IN_CH1 (Mic I2S)";
    case ETS_DMA_IN_CH2_INTR_SOURCE:            return "GDMA_IN_CH2 (Cam EOF)";
    case ETS_DMA_OUT_CH0_INTR_SOURCE:           return "GDMA_OUT_CH0";
    case ETS_FROM_CPU_INTR0_SOURCE:             return "CROSSCORE_YIELD_0";
    case ETS_FROM_CPU_INTR1_SOURCE:             return "CROSSCORE_YIELD_1";
    case ETS_FROM_CPU_INTR2_SOURCE:             return "CROSSCORE_IPC_2";
    case ETS_FROM_CPU_INTR3_SOURCE:             return "CROSSCORE_IPC_3";
    case ETS_USB_SERIAL_JTAG_INTR_SOURCE:       return "USB_SERIAL_JTAG";
    default:                                    return "OTHER_PERIPH";
    }
}

/* --------------------------------------------------------------------------
 * 4. 重置與印出統計報表
 * -------------------------------------------------------------------------- */
void isr_profile_reset(void)
{
    for (uint32_t i = 0; i < s_slot_count; ++i) {
        s_slots[i].count        = 0;
        s_slots[i].total_cycles = 0;
        s_slots[i].max_cycles   = 0;
    }
    s_window_start_us = esp_timer_get_time();
}

void isr_profile_print(const char *phase_name)
{
    const int64_t window_us = esp_timer_get_time() - s_window_start_us;
    if (window_us <= 0) {
        return;
    }
    const uint32_t cpu_mhz = (uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    const uint64_t window_cycles = (uint64_t)window_us * cpu_mhz;

    printf("\n==== ISR CPU Load [%s] (window = %" PRId64 " ms @ %u MHz, slots = %" PRIu32 ") ====\n",
           phase_name ? phase_name : "", window_us / 1000, (unsigned)cpu_mhz, s_slot_count);
    printf("  %-4s %-5s %-4s %-6s %-26s %8s %10s %8s %8s %9s\n",
           "Core", "IntNo", "Type", "SrcID", "Name", "Count", "Total_us", "Avg_us", "Max_us", "CPU_Load%");

    for (int cpu = 0; cpu < CPU_CORES; ++cpu) {
        uint64_t core_total_cycles = 0;
        uint32_t core_total_count  = 0;
        for (uint32_t i = 0; i < s_slot_count; ++i) {
            const isr_slot_t *s = &s_slots[i];
            if (s->cpu != cpu || s->count == 0) {
                continue;
            }
            core_total_cycles += s->total_cycles;
            core_total_count  += s->count;

            const uint64_t total_us = s->total_cycles / cpu_mhz;
            const uint32_t avg_us   = (uint32_t)(total_us / s->count);
            const uint32_t max_us   = s->max_cycles / cpu_mhz;
            const uint32_t load_bp  = (uint32_t)((s->total_cycles * 10000ULL) / window_cycles);

            printf("  CPU%d %5u %-4s %6d %-26s %8" PRIu32 " %10" PRIu64 " %8u %8u %5u.%02u%%\n",
                   cpu, (unsigned)s->intno, s->is_shared ? "SHRD" : "EXCL",
                   (int)s->source, source_name(s->source, s->intno),
                   s->count, total_us, (unsigned)avg_us, (unsigned)max_us,
                   (unsigned)(load_bp / 100U), (unsigned)(load_bp % 100U));
        }
        const uint64_t core_us = core_total_cycles / cpu_mhz;
        const uint32_t core_bp = (uint32_t)((core_total_cycles * 10000ULL) / window_cycles);
        printf("  ---- CPU%d TOTAL ISR ------------------------------ %8" PRIu32 " %10" PRIu64 "          %5u.%02u%%\n",
               cpu, core_total_count, core_us,
               (unsigned)(core_bp / 100U), (unsigned)(core_bp % 100U));
    }
    printf("=======================================================================================\n");
}

#endif /* APP_ISR_PROFILE_ENABLE */

