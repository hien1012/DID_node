#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "sensor.h"

/** Read selected bits from one camera register through the active SCCB bus. */
esp_err_t camera_reg_read(sensor_t *sensor, uint16_t reg, uint8_t mask,
                          uint8_t *value_out);

/** Read-modify-write selected bits in one camera register. */
esp_err_t camera_reg_write(sensor_t *sensor, uint16_t reg, uint8_t mask,
                           uint8_t value);

