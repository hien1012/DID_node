#pragma once

#include "esp_err.h"

#define DEVICE_ID_AUDIO_PREFIX  "mic"
#define DEVICE_ID_CAMERA_PREFIX "cam"

esp_err_t device_identity_init(const char *role_prefix);
const char *device_identity_get(void);
