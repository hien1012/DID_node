#pragma once

#include "esp_err.h"

esp_err_t device_identity_init(const char *role_prefix);
const char *device_identity_get(void);
