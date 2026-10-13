#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t time_sync_from_server(void);
bool time_sync_is_valid(void);
void time_sync_configure_timezone(void);
