#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "audio_record.h"

#define BTN_PIN                 GPIO_NUM_0
#define BTN_DEBOUNCE_MS         50
#define BTN_POST_RELEASE_MS     300

static const char *TAG = "APP_MAIN";

static SemaphoreHandle_t s_btn_pressed_sem = NULL;
static SemaphoreHandle_t s_record_start_sem = NULL;

static void IRAM_ATTR btn_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_btn_pressed_sem, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
}

static esp_err_t btn_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    return gpio_isr_handler_add(BTN_PIN, btn_isr_handler, NULL);
}

static void button_task(void *arg)
{
    (void)arg;

    while (true) {
        xSemaphoreTake(s_btn_pressed_sem, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));

        if (gpio_get_level(BTN_PIN) != 0) {
            continue;
        }

        while (gpio_get_level(BTN_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(BTN_POST_RELEASE_MS));
        xSemaphoreTake(s_btn_pressed_sem, 0);
        xSemaphoreGive(s_record_start_sem);
    }
}

void app_main(void)
{
    s_btn_pressed_sem = xSemaphoreCreateBinary();
    s_record_start_sem = xSemaphoreCreateBinary();
    if (!s_btn_pressed_sem || !s_record_start_sem) {
        ESP_LOGE(TAG, "Button semaphore creation failed");
        return;
    }

    esp_err_t err = btn_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Button initialization failed: %s", esp_err_to_name(err));
        return;
    }

    err = audio_record_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Audio recorder initialization failed: %s", esp_err_to_name(err));
        return;
    }

    if (xTaskCreatePinnedToCore(button_task, "btn_task", 2048, NULL, 5, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Button task creation failed");
        audio_record_deinit();
        return;
    }

    ESP_LOGI(TAG, "Ready. Press button to start recording");

    while (true) {
        xSemaphoreTake(s_record_start_sem, portMAX_DELAY);

        audio_record_result_t result = {0};
        err = audio_record_once(&result);
        if (err == ESP_OK && result.success) {
            ESP_LOGI(TAG,
                     "Recording saved: %s | samples=%u bytes=%u overflow=%u elapsed=%u ms",
                     result.file_path,
                     (unsigned)result.total_samples,
                     (unsigned)result.file_bytes,
                     (unsigned)result.overflow_count,
                     (unsigned)result.elapsed_ms);
        } else {
            ESP_LOGE(TAG,
                     "Recording failed: %s | capture=%d writer=%d write_error=%d",
                     esp_err_to_name(result.error),
                     (int)result.capture_done,
                     (int)result.writer_done,
                     (int)result.write_error);
        }

        ESP_LOGI(TAG, "Ready for next recording");
    }
}
