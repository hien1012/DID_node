#pragma once

/*
 * Debug：邏輯分析儀量測腳 (APP_LA_PROBE_ENABLE)，腳位在 board_pins.h 的 LA_PIN_*。
 * 腳位的高低由 FreeRTOS 切換 task 時更新 (la_task_trace.h)；這裡只負責初始化，
 * 以及依階段決定 LA_PIN_TASK0..3 對應哪些 task。關閉時兩個函式都不做事。
 */
#include "esp_err.h"
#include "app_config.h"

#if APP_LA_PROBE_ENABLE && CONFIG_APPTRACE_DEST_UART1
/* sdkconfig.sysview_uart 把 GPIO41/42 給 SystemView 的 UART1 */
#error "APP_LA_PROBE_ENABLE uses GPIO41/42, which the UART SystemView build also uses"
#endif

typedef enum {
    LA_PHASE_NONE = 0,  /* task 腳不綁定 (只剩 idle 腳) */
    LA_PHASE_RECORD,    /* cam_task, cam_writer, mic_capture, mic_writer */
    LA_PHASE_UPLOAD,    /* wifi, main (HTTP), upload_reader, tiT */
} la_phase_t;

/* 把 LA_PIN_* 設成輸出並拉低。app_main 開頭呼叫，idle 腳從此開始有波形。 */
esp_err_t la_probe_init(void);

/* 依階段重新綁定 task 腳；task 用名稱查 (xTaskGetHandle)，必須已經建立。
 * 會短暫暫停排程器查表，只在階段切換時呼叫。 */
void la_probe_set_phase(la_phase_t phase);
