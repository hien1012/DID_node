#pragma once

#include "esp_err.h"

/*
 * Debug: copy a captured frame to PSRAM and stream it to the PC
 * over a plain TCP socket (tools/cam_stream_view.py in the repository root).
 * No SD card and no HTTP, so a corrupted frame here was already corrupted at
 * capture. One frame at a time: resume, capture, return the frame buffer,
 * pause, then send. The recorder retains its original XCLK management;
 * an onboard sensor oscillator keeps running while capture is paused.
 * Call with the camera recorder initialized and paused. Streams forever,
 * reconnecting whenever the viewer is not listening. Returns only when the
 * camera cannot be paused or resumed, allocation fails, or ESP_ERR_NOT_SUPPORTED when
 * APP_CAM_STREAM_ENABLE is 0.
 */
esp_err_t cam_stream_run(void);
