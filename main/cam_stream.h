#pragma once

#include "esp_err.h"

/*
 * Debug: stream every captured frame straight from the frame buffer to the PC
 * over a plain TCP socket (tools/cam_stream_view.py). No SD card and no HTTP,
 * so a corrupted frame here was already corrupted at capture.
 * One frame at a time: resume camera + XCLK, capture, pause + stop XCLK,
 * then send, so the camera bus does not jam Wi-Fi while sending.
 * Takes over the boot flow and streams forever, reconnecting whenever the
 * viewer is not listening. Returns only when the camera cannot be paused or
 * resumed, or ESP_ERR_NOT_SUPPORTED when APP_CAM_STREAM_ENABLE is 0.
 */
esp_err_t cam_stream_run(void);
