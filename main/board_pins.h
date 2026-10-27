#pragma once

/*
 * ESP32-S3-DevKitC-1 v1.0 (WROOM-1 N16R8) wired with jumper cables.
 *
 * Do not use: GPIO26-32 (flash), GPIO33-37 (octal PSRAM),
 * GPIO0/3/45/46 (strapping; 3 and 46 serve only as post-boot LA outputs below),
 * GPIO19/20 (native USB/JTAG port),
 * GPIO43/44 (UART0 = the board's COM port: flashing and the main console).
 *
 * Every peripheral is routed through the GPIO matrix, so pins can be moved
 * here without touching the drivers. DevKitC-1 v1.1 moved the RGB LED to
 * GPIO38: on that board swap BOARD_PIN_RGB_LED and SD_PIN_CMD.
 */

/* --- Camera (8-bit DVP, ESP32-S3-EYE layout) --- */
/* 3.3 V module only. OV2640 with its own 12 MHz oscillator leaves XCLK
 * unconnected (see USE_OV2640_ONBOARD_XVCLK in cam_record.c). */
#define CAM_PIN_PWDN   -1   /* not wired on this build (GPIO21 is the LA probe below) */
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   15
#define CAM_PIN_SIOD   4
#define CAM_PIN_SIOC   5

/*
 * CAM_PIN_D0..D7 are bus bits from least to most significant:
 *   OV7725 breakout labelled D2..D9 (Y2..Y9): D2->D0, ..., D9->D7
 *   OV7670/OV2640 breakout labelled D0..D7:    D0->D0, ..., D7->D7
 */
#define CAM_PIN_D0     11
#define CAM_PIN_D1     9
#define CAM_PIN_D2     8
#define CAM_PIN_D3     10
#define CAM_PIN_D4     12
#define CAM_PIN_D5     18
#define CAM_PIN_D6     17
#define CAM_PIN_D7     16

#define CAM_PIN_VSYNC  6
#define CAM_PIN_HREF   7
#define CAM_PIN_PCLK   13

/* --- Microphone (I2S0; PDM RX is only available on I2S0) --- */
/* 1: I2S standard mic (BCLK/WS/DIN), 0: PDM mic (CLK/DIN, WS unused). */
#define MIC_INTERFACE_I2S 1

/*
 * Keep the mic clock away from the camera bus. BCLK used to be GPIO14, next
 * to PCLK (GPIO13) on the header: its ~1 MHz square wave coupled into the
 * PCLK jumper and corrupted every recording (byte-phase slips and bit errors),
 * even with the mic wires unplugged (docs/debug_sdcard.md #32-#33).
 */
#define MIC_PIN_CLK    47   /* I2S BCLK or PDM CLK */
#define MIC_PIN_WS     1    /* I2S WS/LRCLK, unused by PDM */
#define MIC_PIN_DIN    2

/* --- microSD (SDMMC, 1-bit). CMD and D0 need 10k pull-ups. --- */
#define SD_PIN_CLK     40
#define SD_PIN_CMD     38
#define SD_PIN_D0      39

/* --- On-board WS2812 RGB LED --- */
#define BOARD_PIN_RGB_LED 48

/*
 * --- Logic analyzer probes (debug, APP_LA_PROBE_ENABLE in app_config.h) ---
 * Driven from the FreeRTOS task-switch hook (la_task_trace.h) and the interrupt
 * entry/exit wrappers (la_isr_trace.S: low while that core is in an ISR). Spare pins only;
 * not GPIO14 (next to PCLK, see the mic clock note above). Passive LA taps need
 * no pin: VSYNC = 6, SD CMD = 38.
 *
 * Task pins: high while the bound task runs. Binding follows the phase
 * (la_probe_set_phase):      recording        upload
 */
#define LA_PIN_TASK0      41   /* cam_task         wifi          */
#define LA_PIN_TASK1      42   /* cam_writer       main (HTTP)   */
#define LA_PIN_TASK2      21   /* mic_capture      upload_reader */
#define LA_PIN_TASK3      48   /* mic_writer       tiT (lwIP)    */
/* GPIO48 is also the on-board RGB LED data line: status_led_off() only blanks
 * it at boot, after that it is a plain output (the LED may light up randomly
 * while probing). */

/* High while that core runs its IDLE task (la_task_trace.h). Strapping pins,
 * used only as outputs after boot: GPIO3 (JTAG source select, ignored unless the
 * STRAP_JTAG_SEL eFuse is burned) and GPIO46 (boot mode / ROM log, pulled down).
 * The LA inputs must not drive them during reset. */
#define LA_PIN_IDLE_CPU0  3
#define LA_PIN_IDLE_CPU1  46
