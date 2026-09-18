#pragma once

/*
 * TTGO T8 V1.7 + external 8-bit DVP camera module.
 *
 * Supported by esp32-camera with this same wiring:
 *   OV2640, OV7670, OV7725
 *
 * The camera module must be powered from 3.3 V. Do not apply 5 V logic to any
 * ESP32 GPIO. PWDN and RESET are not connected by default; tie RESET high and
 * PWDN low on the camera module if its breakout board does not already do so.
 */
#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   5
#define CAM_PIN_SIOD   26
#define CAM_PIN_SIOC   27

/*
 * CAM_PIN_D0..D7 mean bus bits from least to most significant:
 *   OV7725 breakout labelled D2..D9 (Y2..Y9): D2->D0, ..., D9->D7
 *   OV7670/OV2640 breakout labelled D0..D7:    D0->D0, ..., D7->D7
 */
#define CAM_PIN_D0     32
#define CAM_PIN_D1     33
#define CAM_PIN_D2     34
#define CAM_PIN_D3     35
#define CAM_PIN_D4     36
#define CAM_PIN_D5     39
#define CAM_PIN_D6     18
#define CAM_PIN_D7     19

#define CAM_PIN_VSYNC  25
#define CAM_PIN_HREF   23
#define CAM_PIN_PCLK   22

/*
 * TTGO T8 V1.7 onboard microSD wiring (native ESP32 SDMMC slot 1):
 *   CLK=GPIO14, CMD=GPIO15, D0=GPIO2, D1=GPIO4, D2=GPIO12, D3=GPIO13
 * These pins are intentionally not used by the camera mapping above.
 */
