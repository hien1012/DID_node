#include "camera_registers.h"

#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

static const char *TAG = "camera_reg";

#if CONFIG_SCCB_HARDWARE_I2C_PORT1
#define CAMERA_SCCB_PORT I2C_NUM_1
#else
#define CAMERA_SCCB_PORT I2C_NUM_0
#endif

#define SCCB_TIMEOUT_MS 100

/*
 * OV7670 does not expose sensor->get_reg/set_reg in esp32-camera 2.1.5.
 * These helpers reuse the I2C driver already installed by esp_camera_init();
 * they deliberately do not call i2c_driver_install/delete.
 */
static esp_err_t ov7670_sccb_read(sensor_t *sensor, uint8_t reg,
                                  uint8_t *value_out)
{
    i2c_cmd_handle_t command = i2c_cmd_link_create();
    if (command == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(command);
    i2c_master_write_byte(command,
                          (sensor->slv_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(command, reg, true);
    i2c_master_stop(command);

    esp_err_t err = i2c_master_cmd_begin(
        CAMERA_SCCB_PORT, command, pdMS_TO_TICKS(SCCB_TIMEOUT_MS));
    i2c_cmd_link_delete(command);
    if (err != ESP_OK) {
        return err;
    }

    /* SCCB uses a separate read transaction instead of an I2C repeated-start. */
    command = i2c_cmd_link_create();
    if (command == NULL) {
        return ESP_ERR_NO_MEM;
    }
    i2c_master_start(command);
    i2c_master_write_byte(command,
                          (sensor->slv_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(command, value_out, I2C_MASTER_NACK);
    i2c_master_stop(command);
    err = i2c_master_cmd_begin(
        CAMERA_SCCB_PORT, command, pdMS_TO_TICKS(SCCB_TIMEOUT_MS));
    i2c_cmd_link_delete(command);
    return err;
}

static esp_err_t ov7670_sccb_write(sensor_t *sensor, uint8_t reg,
                                   uint8_t value)
{
    i2c_cmd_handle_t command = i2c_cmd_link_create();
    if (command == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(command);
    i2c_master_write_byte(command,
                          (sensor->slv_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(command, reg, true);
    i2c_master_write_byte(command, value, true);
    i2c_master_stop(command);

    const esp_err_t err = i2c_master_cmd_begin(
        CAMERA_SCCB_PORT, command, pdMS_TO_TICKS(SCCB_TIMEOUT_MS));
    i2c_cmd_link_delete(command);
    return err;
}

esp_err_t camera_reg_read(sensor_t *sensor, uint16_t reg, uint8_t mask,
                          uint8_t *value_out)
{
    if (sensor == NULL || value_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensor->get_reg == NULL) {
        if (sensor->id.PID != OV7670_PID) {
            return ESP_ERR_NOT_SUPPORTED;
        }
        uint8_t value = 0;
        const esp_err_t err = ov7670_sccb_read(sensor, reg & 0xFF, &value);
        if (err == ESP_OK) {
            *value_out = value & mask;
        }
        return err;
    }

    const int value = sensor->get_reg(sensor, reg, mask);
    if (value < 0) {
        ESP_LOGE(TAG, "SCCB read failed: reg=0x%04x mask=0x%02x",
                 reg, mask);
        return ESP_FAIL;
    }

    *value_out = (uint8_t)value;
    return ESP_OK;
}

esp_err_t camera_reg_write(sensor_t *sensor, uint16_t reg, uint8_t mask,
                           uint8_t value)
{
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensor->set_reg == NULL) {
        if (sensor->id.PID != OV7670_PID) {
            return ESP_ERR_NOT_SUPPORTED;
        }

        uint8_t current = 0;
        ESP_RETURN_ON_ERROR(ov7670_sccb_read(sensor, reg & 0xFF, &current),
                            TAG, "OV7670 SCCB read-before-write failed");
        const uint8_t updated = (current & ~mask) | (value & mask);
        return ov7670_sccb_write(sensor, reg & 0xFF, updated);
    }

    if (sensor->set_reg(sensor, reg, mask, value) != 0) {
        ESP_LOGE(TAG, "SCCB write failed: reg=0x%04x mask=0x%02x value=0x%02x",
                 reg, mask, value);
        return ESP_FAIL;
    }
    return ESP_OK;
}
