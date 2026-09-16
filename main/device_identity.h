#pragma once

#include "esp_err.h"

esp_err_t device_identity_init(void);
const char *device_identity_get(void);