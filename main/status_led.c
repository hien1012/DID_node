#include "status_led.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "board_pins.h"

/* WS2812 timing at 10 MHz (0.1 us per tick): 0 = 0.3/0.9 us, 1 = 0.9/0.3 us. */
#define LED_RMT_RESOLUTION_HZ 10000000U
#define LED_T0H_TICKS         3U
#define LED_T0L_TICKS         9U
#define LED_T1H_TICKS         9U
#define LED_T1L_TICKS         3U
#define LED_TX_TIMEOUT_MS     100

static const char *TAG = "STATUS_LED";

esp_err_t status_led_off(void)
{
    rmt_channel_handle_t channel = NULL;
    rmt_encoder_handle_t encoder = NULL;
    const rmt_tx_channel_config_t channel_config = {
        .gpio_num = BOARD_PIN_RGB_LED,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 48,
        .trans_queue_depth = 1,
    };
    const rmt_bytes_encoder_config_t encoder_config = {
        .bit0 = {
            .level0 = 1, .duration0 = LED_T0H_TICKS,
            .level1 = 0, .duration1 = LED_T0L_TICKS,
        },
        .bit1 = {
            .level0 = 1, .duration0 = LED_T1H_TICKS,
            .level1 = 0, .duration1 = LED_T1L_TICKS,
        },
        .flags.msb_first = 1,
    };

    esp_err_t err = rmt_new_tx_channel(&channel_config, &channel);
    if (err == ESP_OK) {
        err = rmt_new_bytes_encoder(&encoder_config, &encoder);
    }
    if (err == ESP_OK) {
        err = rmt_enable(channel);
    }
    if (err == ESP_OK) {
        static const uint8_t black_grb[3] = {0, 0, 0};
        const rmt_transmit_config_t transmit_config = {.loop_count = 0};
        err = rmt_transmit(channel, encoder, black_grb, sizeof(black_grb),
                           &transmit_config);
        if (err == ESP_OK) {
            err = rmt_tx_wait_all_done(channel, LED_TX_TIMEOUT_MS);
        }
        (void)rmt_disable(channel);
    }
    if (encoder) {
        (void)rmt_del_encoder(encoder);
    }
    if (channel) {
        (void)rmt_del_channel(channel);
    }

    /* Hold the data line low so the LED stays latched at black. */
    gpio_reset_pin(BOARD_PIN_RGB_LED);
    gpio_set_direction(BOARD_PIN_RGB_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(BOARD_PIN_RGB_LED, 0);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Cannot switch RGB LED off: %s", esp_err_to_name(err));
    }
    return err;
}
