#pragma once

/*
 * ESP32-S3-DevKitC-1 v1.0 (WROOM-1 N16R8) wired with jumper cables.
 *
 * Do not use: GPIO26-32 (flash), GPIO33-37 (octal PSRAM),
 * GPIO0/3/45/46 (strapping), GPIO19/20 (native USB/JTAG port),
 * GPIO43/44 (UART0 = the board's COM port: flashing and the main console).
 *
 * Every peripheral is routed through the GPIO matrix, so pins can be moved
 * here without touching the drivers. DevKitC-1 v1.1 moved the RGB LED to
 * GPIO38: on that board swap BOARD_PIN_RGB_LED and SD_PIN_CMD.
 */

/* --- Camera (8-bit DVP, ESP32-S3-EYE layout) --- */
/* 3.3 V module only. OV2640 with its own 12 MHz oscillator leaves XCLK
 * unconnected (see USE_OV2640_ONBOARD_XVCLK in cam_record.c). */
#define CAM_PIN_PWDN   21
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

#define MIC_PIN_CLK    14   /* I2S BCLK or PDM CLK */
#define MIC_PIN_WS     1    /* I2S WS/LRCLK, unused by PDM */
#define MIC_PIN_DIN    2

/* --- microSD (SDMMC, 1-bit). CMD and D0 need 10k pull-ups. --- */
#define SD_PIN_CLK     40
#define SD_PIN_CMD     38
#define SD_PIN_D0      39

/* --- On-board WS2812 RGB LED --- */
#define BOARD_PIN_RGB_LED 48
