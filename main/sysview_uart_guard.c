/*
 * SystemView 走 UART（sdkconfig.sysview_uart）時的開機 panic 修正，只在該 trace build 生效。
 *
 * IDF 5.1.2 的問題：SystemView 每送一個封包（包括還沒 Start、被丟掉的封包）都會呼叫
 * SEGGER_RTT_ReadNoLock() 檢查 host 命令（SEGGER_RTT_HASDATA 固定為 1）。UART 版的實作
 * 會呼叫 uart_get_buffered_data_len() / uart_read_bytes()，但 UART 驅動放在 flash。
 * 另一顆核心寫 flash 時 cache 被關掉，這顆核心上的 IRAM 中斷一進 ISR wrapper 就記錄
 * ISR enter 事件 → 執行到 flash 裡的 UART 驅動 →「Cache disabled but cached memory region
 * accessed」panic（2026-10-23，backtrace：ipc_task → ISR → _SendPacket → uart_read_bytes）。
 *
 * 做法：用 -Wl,--wrap=SEGGER_RTT_ReadNoLock 攔截（呼叫端 SEGGER_SYSVIEW.c 與定義端
 * SEGGER_RTT_esp.c 是不同 object，wrap 才有效），在 cache 關閉或 ISR 內直接回 0
 * （當作 host 沒有資料），只在一般 task 且 cache 開著時才讀 UART。host 的 Start 命令
 * 晚一點、在下一個 task context 的封包才被讀到，不影響功能。
 */
#include "sdkconfig.h"

#if CONFIG_APPTRACE_SV_DEST_UART

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "esp_private/cache_utils.h"

unsigned __real_SEGGER_RTT_ReadNoLock(unsigned BufferIndex, void *pData, unsigned BufferSize);

/* 從 cache 關閉中的 ISR 也會被呼叫，所以本身與先呼叫的檢查都必須在 IRAM。
 * spi_flash_cache_enabled() 是 IRAM_ATTR；確認 cache 開著之後才呼叫 xPortInIsrContext()。 */
unsigned IRAM_ATTR __wrap_SEGGER_RTT_ReadNoLock(unsigned BufferIndex, void *pData, unsigned BufferSize)
{
    if (!spi_flash_cache_enabled()) {
        return 0;
    }
    /* ISR 內不能呼叫 uart_read_bytes()（會拿 mutex） */
    if (xPortInIsrContext()) {
        return 0;
    }
    return __real_SEGGER_RTT_ReadNoLock(BufferIndex, pData, BufferSize);
}

#endif /* CONFIG_APPTRACE_SV_DEST_UART */
