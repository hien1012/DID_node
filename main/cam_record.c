#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_camera.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "sensor.h"

#include "camera_pinout.h"
#include "camera_registers.h"

#define IMAGE_WIDTH             640U
#define IMAGE_HEIGHT            480U
#define IMAGE_PIXELS            (IMAGE_WIDTH * IMAGE_HEIGHT)
#define SENSOR_YUV422_BYTES     (IMAGE_PIXELS * 2U)

/*
 * Clock-source selection for the interchangeable camera modules.
 *
 * The OV2640 module used here has its own 12 MHz oscillator connected to
 * XVCLK.  Keep the ESP32 XCLK output disabled for that module; two clock
 * outputs must never drive the same XVCLK net.  Set this to 0 before using
 * OV7670/OV7725 modules that require the ESP32-generated 16 MHz XCLK.
 */
#define USE_OV2640_ONBOARD_XVCLK 0
#define ESP32_CAMERA_XCLK_HZ     16000000U
#define OV2640_ONBOARD_XVCLK_HZ  12000000U
#define OV2640_HAL_XCLK_HINT_HZ  20000000U

#if USE_OV2640_ONBOARD_XVCLK
#define CAMERA_CONFIG_XCLK_PIN   (-1)
/*
 * pin_xclk=-1 means this value does not drive the sensor oscillator.  On
 * classic ESP32, esp32-camera also uses it as the I2S sampling-mode selector:
 * values above 10 MHz select its high-speed unpacker.  The measured DVP PCLK
 * is only 8 MHz, so select the low-speed unpacker while retaining the actual
 * 12 MHz onboard XVCLK in OV2640_XVCLK_HZ and the sensor clock calculation.
 */
#define CAMERA_CONFIG_XCLK_HZ    OV2640_HAL_XCLK_HINT_HZ
#else
#define CAMERA_CONFIG_XCLK_PIN   CAM_PIN_XCLK
#define CAMERA_CONFIG_XCLK_HZ    ESP32_CAMERA_XCLK_HZ
#endif

#define CAMERA_WARMUP_FRAMES    10U
#define CAMERA_FRAME_BUFFERS    8U //更改PSRAM_frame_buffer數量看看是否會發生VSYNC-OVF ==> 仍然失敗
#define TEST_DURATION_SECONDS   10U
#define TARGET_FPS               5U
#define EXPECTED_FRAME_COUNT     (TEST_DURATION_SECONDS * TARGET_FPS)
#define EXPECTED_FRAME_PERIOD_US (1000000LL / TARGET_FPS)
#define GAP_THRESHOLD_US         (EXPECTED_FRAME_PERIOD_US * 3LL / 2LL)
#define FPS_TOLERANCE_MILLI      250U

#define SD_MOUNT_POINT                "/sdcard"
#define SD_MAX_OPEN_FILES             2
#define SD_FORMAT_ALLOCATION_BYTES    (16U * 1024U)
#define SD_DMA_BUFFER_PREFERRED_BYTES (32U * 1024U)
#define SD_DMA_BUFFER_MIN_BYTES       (8U * 1024U)
/*
 * Writer holds one framebuffer, camera HAL needs one active buffer and one
 * spare buffer at the next VSYNC.  Queue at most the remaining five pointers.
 */
#define FRAME_QUEUE_CAPACITY          (CAMERA_FRAME_BUFFERS - 3U)
#define RECORD_DURATION_SECONDS       10U
#define RECORD_EXPECTED_FRAMES        (RECORD_DURATION_SECONDS * TARGET_FPS)
#define RECORD_FILE_BYTES             ((uint64_t)RECORD_EXPECTED_FRAMES * IMAGE_PIXELS)
#define RECORD_MAX_SEQUENCE           10000000U
#define RECORD_PATH_BYTES             32U
#define RECORD_QUEUE_WAIT_MS          50U

#define CAPTURE_TASK_STACK_BYTES      4096U
#define WRITER_TASK_STACK_BYTES       4096U
#define CAPTURE_TASK_PRIORITY         6U
#define WRITER_TASK_PRIORITY          5U

#if CONFIG_FREERTOS_UNICORE
#define CAPTURE_TASK_CORE             0
#define WRITER_TASK_CORE              0
#else
#define CAPTURE_TASK_CORE             1
#define WRITER_TASK_CORE              0
#endif

#define CAPTURE_DONE_BIT              BIT0
#define WRITER_DONE_BIT               BIT1

#define TEST_BUTTON_GPIO         GPIO_NUM_0
#define BUTTON_POLL_MS          10U
#define BUTTON_DEBOUNCE_MS      30U

#define OV7725_REG_COM3             0x000CU
#define OV7725_REG_COM4             0x000DU
#define OV7725_REG_COM5             0x000EU
#define OV7725_REG_CLKRC            0x0011U
#define OMNIVISION_REG_COM8         0x0013U
#define OV7725_REG_DSP_CTRL3        0x0066U
#define OV7725_COM3_SWAP_YUV        0x10U
#define OV7725_COM4_PLL_MASK        0xC0U
#define OV7725_COM5_AUTO_FPS        0x80U
#define OV7725_CLKRC_DIV_MASK       0x3FU
#define OV7725_CLKRC_DIVIDER        3U // Divide by 4 instead of 2: halve prior rate.
#define OMNIVISION_COM8_AUTO_MASK   0x07U

/* OV2640 get_reg/set_reg encode the bank in register address bit 8. */
#define OV2640_DSP_R_BYPASS         0x0005U
#define OV2640_DSP_ZMOW             0x005AU
#define OV2640_DSP_ZMOH             0x005BU
#define OV2640_DSP_ZMHH             0x005CU
#define OV2640_SENSOR_CLKRC         0x0111U
#define OV2640_SENSOR_COM8          0x0113U
#define OV2640_SENSOR_REG2A         0x012AU
#define OV2640_SENSOR_FRARL         0x012BU
#define OV2640_SENSOR_ADDVSL        0x012DU
#define OV2640_SENSOR_ADDVSH        0x012EU
#define OV2640_SENSOR_FLL           0x0146U
#define OV2640_SENSOR_FLH           0x0147U
#define OV2640_DSP_CTRL0            0x00C2U
#define OV2640_DSP_CTRL1            0x00C3U
#define OV2640_DSP_R_DVP_SP         0x00D3U
#define OV2640_DSP_IMAGE_MODE       0x00DAU
#define OV2640_R_BYPASS_DSP_MASK    0x01U
#define OV2640_CTRL0_YUV422_MASK    0x0CU
#define OV2640_IMAGE_MODE_FMT_MASK  0x5CU
#define OV2640_VGA_ZMOW_VALUE       0xA0U
#define OV2640_VGA_ZMOH_VALUE       0x78U
#define OV2640_VGA_ZMHH_VALUE       0x00U
#define OV2640_COM8_AEC_AGC_MASK    0x05U
#define OV2640_CTRL1_AWB_MASK       0x08U
#define OV2640_CLKRC_CLOCK_MASK     0xBFU
#if USE_OV2640_ONBOARD_XVCLK
/* Match the SCCB-only 5 fps test: 12 MHz x 2 / (5 + 1) = 4 MHz. */
#define OV2640_XVCLK_HZ              OV2640_ONBOARD_XVCLK_HZ
#define OV2640_CLK_MULTIPLIER        2ULL
#define OV2640_CLKRC_DOUBLER         0x80U
#define OV2640_CLKRC_DIVIDER         5U
#else
/* ESP32 clock path: 16 MHz XVCLK / (3 + 1) = 4 MHz. */
#define OV2640_XVCLK_HZ              ESP32_CAMERA_XCLK_HZ
#define OV2640_CLK_MULTIPLIER        1ULL
#define OV2640_CLKRC_DOUBLER         0x00U
#define OV2640_CLKRC_DIVIDER         3U
#endif
#define OV2640_CLKRC_VALUE \
    (OV2640_CLKRC_DOUBLER | OV2640_CLKRC_DIVIDER)
#define OV2640_REG2A_FRARH_MASK     0xF0U
#define OV2640_DVP_SP_AUTO_MODE     0x80U
/* Automatic DVP PCLK control plus divisor 6 gives R_DVP_SP=0x86.
 * The 48 MHz sysclk model below is nominal; measure actual DVP PCLK. */
#define OV2640_DVP_PCLK_DIVIDER     6U
#define OV2640_DVP_SP_VALUE \
    (OV2640_DVP_SP_AUTO_MODE | OV2640_DVP_PCLK_DIVIDER)

#define OV7670_REG_COM3             0x000CU
#define OV7670_REG_COM4             0x000DU
#define OV7670_REG_CLKRC            0x0011U
#define OV7670_REG_ADVFL            0x002DU
#define OV7670_REG_ADVFH            0x002EU
#define OV7670_REG_TSLB             0x003AU
#define OV7670_REG_COM11            0x003BU
#define OV7670_REG_DBLV             0x006BU
#define OV7670_REG_DM_LNL           0x0092U
#define OV7670_REG_DM_LNH           0x0093U
#define OV7670_CLKRC_CLOCK_MASK     0x7FU
#define OV7670_CLKRC_DIVIDER        7U // Divide by 8 instead of 4: halve prior rate.
#define OV7670_DBLV_PLL_MASK        0xC0U
#define OV7670_DBLV_PLL_4X          0x40U
#define OV7670_COM11_NIGHT_MODE     0x80U

#define OV7725_PLL_MULTIPLIER       1U
#define OV7725_INTERNAL_CLOCK_HZ \
    (ESP32_CAMERA_XCLK_HZ * OV7725_PLL_MULTIPLIER / \
     ((OV7725_CLKRC_DIVIDER + 1U) * 2U))

/* Datasheet VGA YUV timing: 510 lines, 784 tP/line, tP = 2 PCLK. */
#define OV7725_VGA_LINES_PER_FRAME       510U
#define OV7725_VGA_TP_PER_LINE           784U
#define OV7725_YUV_PCLK_PER_TP           2U
#define OV7725_PCLK_PER_FRAME \
    (OV7725_VGA_LINES_PER_FRAME * OV7725_VGA_TP_PER_LINE )
#define OV7725_CALCULATED_FPS \
    (OV7725_INTERNAL_CLOCK_HZ / OV7725_PCLK_PER_FRAME)
#define OV7725_CALCULATED_MILLI_FPS \
    ((uint32_t)(((uint64_t)OV7725_INTERNAL_CLOCK_HZ * 1000ULL) / \
                OV7725_PCLK_PER_FRAME))

/*
 * OV7670 clock tree:
 * fINT = XCLK * PLL / (2 * (CLKRC + 1)).  The datasheet's VGA/YUV
 * reference is 30 fps at fINT=24 MHz, so fINT=4 MHz targets 5 fps.
 */
#define OV7670_PLL_MULTIPLIER             4ULL
#define OV7670_REFERENCE_INTERNAL_HZ       24000000ULL
#define OV7670_REFERENCE_MILLI_FPS         30000ULL
#define OV7670_INTERNAL_CLOCK_HZ \
    ((ESP32_CAMERA_XCLK_HZ * OV7670_PLL_MULTIPLIER) / \
     (2ULL * (OV7670_CLKRC_DIVIDER + 1ULL)))
#define OV7670_CALCULATED_MILLI_FPS \
    ((uint32_t)((OV7670_INTERNAL_CLOCK_HZ * \
                 OV7670_REFERENCE_MILLI_FPS) / \
                OV7670_REFERENCE_INTERNAL_HZ))

/* VGA is scaled by the OV2640 DSP from the sensor's 800x600 SVGA timing. */
#define OV2640_SVGA_CLOCKS_PER_LINE 1190ULL
#define OV2640_SVGA_LINES_PER_FRAME 672ULL
#define OV2640_INTERNAL_CLOCK_HZ \
    ((OV2640_XVCLK_HZ * OV2640_CLK_MULTIPLIER) / \
     (OV2640_CLKRC_DIVIDER + 1ULL))
#define OV2640_CALCULATED_MILLI_FPS \
    ((uint32_t)((OV2640_INTERNAL_CLOCK_HZ * 1000ULL) / \
                (OV2640_SVGA_CLOCKS_PER_LINE * \
                 OV2640_SVGA_LINES_PER_FRAME)))
#define OV2640_DVP_SYSCLK_HZ       48000000ULL
#define OV2640_DVP_PCLK_HZ \
    (OV2640_DVP_SYSCLK_HZ / OV2640_DVP_PCLK_DIVIDER)
#define OV2640_DVP_TRANSFER_US \
    ((SENSOR_YUV422_BYTES * 1000000ULL) / OV2640_DVP_PCLK_HZ)

_Static_assert(CAMERA_WARMUP_FRAMES >= 3U,
               "fps verification requires at least three warm-up frames");
_Static_assert(OV7670_CALCULATED_MILLI_FPS == TARGET_FPS * 1000U,
               "OV7670 clock model must match TARGET_FPS");
_Static_assert(OV2640_CALCULATED_MILLI_FPS >= TARGET_FPS * 1000U &&
               OV2640_CALCULATED_MILLI_FPS <= TARGET_FPS * 1000U + 10U,
               "OV2640 clock model must approximately match TARGET_FPS");
_Static_assert(OV2640_DVP_PCLK_HZ * 100ULL >=
                   SENSOR_YUV422_BYTES * TARGET_FPS * 120ULL,
               "OV2640 DVP PCLK needs at least 20 percent transfer headroom");

typedef struct {
    uint32_t get_calls;
    uint32_t complete_frames;
    uint32_t invalid_frames;
    uint32_t stale_frames;
    uint32_t after_deadline_frames;
    uint32_t timeout_count;
    uint32_t timestamp_errors;
    uint32_t gap_events;
    uint32_t estimated_missing_frames;
    int64_t first_frame_us;
    int64_t last_frame_us;
    int64_t min_interval_us;
    int64_t max_interval_us;
} dma_receive_test_stats_t;

typedef struct {
    camera_fb_t *frame;
    uint32_t frame_id;
    int64_t timestamp_us;
} ready_frame_t;

typedef struct {
    uint32_t get_calls;
    uint32_t in_window_frames;
    uint32_t enqueued_frames;
    uint32_t saved_frames;
    uint32_t stale_frames;
    uint32_t invalid_frames;
    uint32_t timestamp_errors;
    uint32_t capture_timeouts;
    uint32_t pool_overflows;
    uint32_t queue_errors;
    uint32_t frame_gaps;
    uint32_t peak_ready_slots;
    int64_t first_frame_us;
    int64_t last_frame_us;
    int64_t min_interval_us;
    int64_t max_interval_us;
    int64_t max_sd_chunk_us;
    int64_t max_sd_frame_us;
    int64_t capture_end_us;
    int64_t writer_end_us;
    esp_err_t capture_result;
    esp_err_t writer_result;
} recording_stats_t;

typedef struct {
    int fd;
    int64_t trigger_us;
    int64_t deadline_us;
    recording_stats_t stats;
} recording_context_t;

_Static_assert(IMAGE_PIXELS == 307200U,
               "VGA GRAY8 must contain 307200 bytes");
_Static_assert(EXPECTED_FRAME_COUNT == 50U,
               "10 seconds at 5 fps must contain about 50 frames");
_Static_assert(OV7725_CALCULATED_FPS == TARGET_FPS,
               "OV7725 clock model must match TARGET_FPS");
_Static_assert((SD_DMA_BUFFER_PREFERRED_BYTES % 512U) == 0U,
               "SD DMA staging buffer must contain complete SD sectors");
_Static_assert((IMAGE_PIXELS % 512U) == 0U,
               "Each GRAY8 frame must end on an SD sector boundary");
_Static_assert(CAMERA_FRAME_BUFFERS >= 4U,
               "Zero-copy recording needs active, spare and queued buffers");
_Static_assert(RECORD_FILE_BYTES <= INT32_MAX,
               "The preallocated RAW file must fit in off_t on ESP32");

static const char *TAG = "cam_record";
static sdmmc_card_t *s_sd_card = NULL;
static uint8_t *s_sd_dma_buffer = NULL;
static size_t s_sd_dma_buffer_bytes = 0;

static StaticQueue_t s_ready_queue_control;
static uint8_t s_ready_queue_storage[
    FRAME_QUEUE_CAPACITY * sizeof(ready_frame_t)];
static QueueHandle_t s_ready_queue = NULL;

static StaticEventGroup_t s_record_events_control;
static EventGroupHandle_t s_record_events = NULL;
static recording_context_t s_recording;

static camera_config_t s_camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAMERA_CONFIG_XCLK_PIN,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = CAMERA_CONFIG_XCLK_HZ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    /* Sensors transmit YUV422; classic ESP32 DMA retains Y into GRAY8. */
    .pixel_format = PIXFORMAT_GRAYSCALE,
    .frame_size = FRAMESIZE_VGA,
    .jpeg_quality = 12, /* Unused in grayscale mode. */
    /*
     * Eight driver-owned PSRAM buffers replace the application PSRAM copy.
     * Writer returns each camera_fb_t only after its RAW payload is committed.
     * The ready queue limit leaves active and spare buffers for the next VSYNC.
     */
    .fb_count = CAMERA_FRAME_BUFFERS,
    .fb_location = CAMERA_FB_IN_PSRAM,
    /* While waiting for GPIO0, retain only the newest completed frame. */
    .grab_mode = CAMERA_GRAB_LATEST,
};

static const char *sensor_name(uint16_t pid)
{
    switch (pid) {
    case OV2640_PID: return "OV2640";
    case OV7670_PID: return "OV7670";
    case OV7725_PID: return "OV7725";
    default: return "unsupported";
    }
}
/* Configure the measured OV7725 grayscale byte order and fixed hardware fps. */
static esp_err_t configure_ov7725(sensor_t *sensor)
{
    uint8_t com3 = 0;
    uint8_t dsp_ctrl3 = 0;
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_COM3, 0xFF, &com3),
                        TAG, "Cannot read OV7725 COM3");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_DSP_CTRL3,
                                        0xFF, &dsp_ctrl3),
                        TAG, "Cannot read OV7725 DSP_CTRL3");

    /* Previous raw-frame tests proved COM3[4]=1 gives Y-first on this wiring. */
    if ((com3 & OV7725_COM3_SWAP_YUV) == 0) {
        ESP_LOGE(TAG,
                 "OV7725 COM3=0x%02X: COM3[4] must remain 1 for GRAY8 capture",
                 com3);
        return ESP_ERR_INVALID_STATE;
    }

    /* Halve the prior sensor rate through CLKRC; verify VSYNC on hardware.
     * A calculated rate or delivered-frame rate alone does not prove VSYNC. */
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7725_REG_COM4,
                                         OV7725_COM4_PLL_MASK, 0x00),
                        TAG, "Cannot select OV7725 PLL bypass");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7725_REG_CLKRC,
                                         OV7725_CLKRC_DIV_MASK,
                                         OV7725_CLKRC_DIVIDER),
                        TAG, "Cannot set OV7725 CLKRC divider");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7725_REG_COM5,
                                         OV7725_COM5_AUTO_FPS, 0x00),
                        TAG, "Cannot disable OV7725 automatic frame-rate reduction");

    uint8_t com4 = 0;
    uint8_t com5 = 0;
    uint8_t clkrc = 0;
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_COM4, 0xFF, &com4),
                        TAG, "Cannot verify OV7725 COM4");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_COM5, 0xFF, &com5),
                        TAG, "Cannot verify OV7725 COM5");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_CLKRC, 0xFF, &clkrc),
                        TAG, "Cannot verify OV7725 CLKRC");

    if ((com4 & OV7725_COM4_PLL_MASK) != 0 ||
        (clkrc & OV7725_CLKRC_DIV_MASK) != OV7725_CLKRC_DIVIDER ||
        (com5 & OV7725_COM5_AUTO_FPS) != 0) {
        ESP_LOGE(TAG, "OV7725 clock verification failed: COM4=0x%02X "
                 "CLKRC=0x%02X COM5=0x%02X", com4, clkrc, com5);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG,
             "OV7725 GRAY8: COM3=0x%02X DSP_CTRL3=0x%02X; "
             "XCLK=%u Hz, calculated=%u.%03u fps",
             com3, dsp_ctrl3, (unsigned)ESP32_CAMERA_XCLK_HZ,
             (unsigned)(OV7725_CALCULATED_MILLI_FPS / 1000U),
             (unsigned)(OV7725_CALCULATED_MILLI_FPS % 1000U));
    return ESP_OK;
}

static esp_err_t configure_ov7670(sensor_t *sensor)
{
    /*
     * Keep PLL x4, then divide the multiplied 16 MHz XCLK by 2*(7+1).
     * This targets a sensor/VSYNC rate of 5 fps; it is not merely
     * a software-side frame selection interval.
     */
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7670_REG_DBLV,
                                         OV7670_DBLV_PLL_MASK,
                                         OV7670_DBLV_PLL_4X),
                        TAG, "Cannot set OV7670 PLL x4");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7670_REG_CLKRC,
                                         OV7670_CLKRC_CLOCK_MASK,
                                         OV7670_CLKRC_DIVIDER),
                        TAG, "Cannot set OV7670 CLKRC divider");

    /* Do not let low-light/night mode or stale dummy rows lower the fps. */
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7670_REG_COM11,
                                         OV7670_COM11_NIGHT_MODE, 0x00),
                        TAG, "Cannot disable OV7670 night-mode fps reduction");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7670_REG_ADVFL, 0xFF, 0x00),
                        TAG, "Cannot clear OV7670 ADVFL");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7670_REG_ADVFH, 0xFF, 0x00),
                        TAG, "Cannot clear OV7670 ADVFH");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7670_REG_DM_LNL, 0xFF, 0x00),
                        TAG, "Cannot clear OV7670 DM_LNL");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7670_REG_DM_LNH, 0xFF, 0x00),
                        TAG, "Cannot clear OV7670 DM_LNH");

    uint8_t tslb = 0, com3 = 0, com4 = 0, clkrc = 0, dblv = 0;
    uint8_t com11 = 0, advfl = 0, advfh = 0, dm_lnl = 0, dm_lnh = 0;
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7670_REG_TSLB, 0xFF, &tslb),
                        TAG, "Cannot verify OV7670 TSLB");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7670_REG_COM3, 0xFF, &com3),
                        TAG, "Cannot verify OV7670 COM3");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7670_REG_COM4, 0xFF, &com4),
                        TAG, "Cannot verify OV7670 COM4");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7670_REG_CLKRC, 0xFF, &clkrc),
                        TAG, "Cannot verify OV7670 CLKRC");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7670_REG_DBLV, 0xFF, &dblv),
                        TAG, "Cannot verify OV7670 DBLV");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7670_REG_COM11, 0xFF, &com11),
                        TAG, "Cannot verify OV7670 COM11");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7670_REG_ADVFL, 0xFF, &advfl),
                        TAG, "Cannot verify OV7670 ADVFL");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7670_REG_ADVFH, 0xFF, &advfh),
                        TAG, "Cannot verify OV7670 ADVFH");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7670_REG_DM_LNL, 0xFF, &dm_lnl),
                        TAG, "Cannot verify OV7670 DM_LNL");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7670_REG_DM_LNH, 0xFF, &dm_lnh),
                        TAG, "Cannot verify OV7670 DM_LNH");

    if ((dblv & OV7670_DBLV_PLL_MASK) != OV7670_DBLV_PLL_4X ||
        (clkrc & OV7670_CLKRC_CLOCK_MASK) != OV7670_CLKRC_DIVIDER ||
        (com11 & OV7670_COM11_NIGHT_MODE) != 0 ||
        advfl != 0 || advfh != 0 || dm_lnl != 0 || dm_lnh != 0) {
        ESP_LOGE(TAG, "OV7670 clock verification failed: CLKRC=0x%02X "
                 "DBLV=0x%02X COM11=0x%02X ADVF=%02X%02X DM_LN=%02X%02X",
                 clkrc, dblv, com11, advfh, advfl, dm_lnh, dm_lnl);
        return ESP_ERR_INVALID_STATE;
    }

    /* esp32-camera's VGA/YUV table is YUYV, so the DMA keeps byte zero. */
    ESP_LOGI(TAG,
             "OV7670 GRAY8: TSLB=0x%02X COM3=0x%02X COM4=0x%02X "
             "CLKRC=0x%02X DBLV=0x%02X; hardware=%u.%03u fps",
             tslb, com3, com4, clkrc, dblv,
             (unsigned)(OV7670_CALCULATED_MILLI_FPS / 1000U),
             (unsigned)(OV7670_CALCULATED_MILLI_FPS % 1000U));
    return ESP_OK;
}

static esp_err_t configure_ov2640(sensor_t *sensor)
{
    //camera_reg_write(sensor, 0x0112U, 0xFF, 0x80);
    /*
     * VGA is produced by scaling the 800x600 SVGA sensor timing.  With
     * the selected XVCLK/multiplier/divider constants, the sensor timing
     * clock is 4 MHz.  The SVGA timing model of 1190*672 clocks/frame gives
     * 5.002 fps.  Match the standalone test's R_DVP_SP=0x86; actual PCLK,
     * 480 HREF/frame and 1280 PCLK/HREF still require hardware measurement.
     */
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_DSP_R_DVP_SP,
                                         0xFF, OV2640_DVP_SP_VALUE),
                        TAG, "Cannot set OV2640 DVP PCLK divider");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_SENSOR_CLKRC,
                                         OV2640_CLKRC_CLOCK_MASK,
                                         OV2640_CLKRC_VALUE),
                        TAG, "Cannot set OV2640 CLKRC doubler/divider");

    /*
     * OV2640 implements PIXFORMAT_GRAYSCALE by transmitting YUV422 and
     * letting the ESP32 DMA retain Y.  A VGA line must therefore contain
     * 640 pixels * 2 bytes = 1280 active PCLK edges at the sensor pins.
     */
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_DSP_CTRL0,
                                         OV2640_CTRL0_YUV422_MASK,
                                         OV2640_CTRL0_YUV422_MASK),
                        TAG, "Cannot enable OV2640 YUV422 pipeline");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_DSP_IMAGE_MODE,
                                         OV2640_IMAGE_MODE_FMT_MASK, 0x00),
                        TAG, "Cannot select OV2640 YUV422 DVP output");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_DSP_R_BYPASS,
                                         OV2640_R_BYPASS_DSP_MASK, 0x00),
                        TAG, "Cannot enable OV2640 DSP");

    /* Remove any line/frame extensions left by an earlier sensor setup. */
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_SENSOR_REG2A,
                                         OV2640_REG2A_FRARH_MASK, 0x00),
                        TAG, "Cannot clear OV2640 FRARH");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_SENSOR_FRARL,
                                         0xFF, 0x00),
                        TAG, "Cannot clear OV2640 FRARL");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_SENSOR_ADDVSL,
                                         0xFF, 0x00),
                        TAG, "Cannot clear OV2640 ADDVSL");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_SENSOR_ADDVSH,
                                         0xFF, 0x00),
                        TAG, "Cannot clear OV2640 ADDVSH");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_SENSOR_FLL,
                                         0xFF, 0x00),
                        TAG, "Cannot clear OV2640 FLL");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_SENSOR_FLH,
                                         0xFF, 0x00),
                        TAG, "Cannot clear OV2640 FLH");

    uint8_t clkrc = 0, dvp_sp = 0, reg2a = 0, frarl = 0;
    uint8_t addvsl = 0, addvsh = 0, fll = 0, flh = 0;
    uint8_t r_bypass = 0, ctrl0 = 0, image_mode = 0;
    uint8_t zmow = 0, zmoh = 0, zmhh = 0;
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_SENSOR_CLKRC,
                                        0xFF, &clkrc),
                        TAG, "Cannot verify OV2640 sensor CLKRC");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_DSP_R_DVP_SP,
                                         0xFF, &dvp_sp),
                        TAG, "Cannot verify OV2640 DSP R_DVP_SP");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_SENSOR_REG2A,
                                        0xFF, &reg2a),
                        TAG, "Cannot verify OV2640 REG2A");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_SENSOR_FRARL,
                                        0xFF, &frarl),
                        TAG, "Cannot verify OV2640 FRARL");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_SENSOR_ADDVSL,
                                        0xFF, &addvsl),
                        TAG, "Cannot verify OV2640 ADDVSL");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_SENSOR_ADDVSH,
                                        0xFF, &addvsh),
                        TAG, "Cannot verify OV2640 ADDVSH");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_SENSOR_FLL,
                                        0xFF, &fll),
                        TAG, "Cannot verify OV2640 FLL");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_SENSOR_FLH,
                                         0xFF, &flh),
                        TAG, "Cannot verify OV2640 FLH");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_DSP_R_BYPASS,
                                        0xFF, &r_bypass),
                        TAG, "Cannot verify OV2640 R_BYPASS");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_DSP_CTRL0,
                                        0xFF, &ctrl0),
                        TAG, "Cannot verify OV2640 CTRL0");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_DSP_IMAGE_MODE,
                                        0xFF, &image_mode),
                        TAG, "Cannot verify OV2640 IMAGE_MODE");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_DSP_ZMOW,
                                        0xFF, &zmow),
                        TAG, "Cannot verify OV2640 ZMOW");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_DSP_ZMOH,
                                        0xFF, &zmoh),
                        TAG, "Cannot verify OV2640 ZMOH");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_DSP_ZMHH,
                                        0xFF, &zmhh),
                        TAG, "Cannot verify OV2640 ZMHH");

    if (dvp_sp != OV2640_DVP_SP_VALUE ||
        (clkrc & OV2640_CLKRC_CLOCK_MASK) != OV2640_CLKRC_VALUE ||
        (reg2a & OV2640_REG2A_FRARH_MASK) != 0 || frarl != 0 ||
        addvsl != 0 || addvsh != 0 || fll != 0 || flh != 0) {
        ESP_LOGE(TAG, "OV2640 clock verification failed: CLKRC=0x%02X "
                 "R_DVP_SP=0x%02X "
                 "FRAR=%01X%02X ADDVS=%02X%02X FL=%02X%02X",
                 clkrc, dvp_sp, (reg2a >> 4), frarl,
                 addvsh, addvsl, flh, fll);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG,
             "OV2640 DVP format: R_BYPASS=0x%02X CTRL0=0x%02X "
             "IMAGE_MODE=0x%02X ZMOW/H/HH=%02X/%02X/%02X",
             r_bypass, ctrl0, image_mode, zmow, zmoh, zmhh);
    if ((r_bypass & OV2640_R_BYPASS_DSP_MASK) != 0 ||
        (ctrl0 & OV2640_CTRL0_YUV422_MASK) != OV2640_CTRL0_YUV422_MASK ||
        (image_mode & OV2640_IMAGE_MODE_FMT_MASK) != 0 ||
        zmow != OV2640_VGA_ZMOW_VALUE ||
        zmoh != OV2640_VGA_ZMOH_VALUE ||
        zmhh != OV2640_VGA_ZMHH_VALUE) {
        ESP_LOGE(TAG,
                 "OV2640 is not VGA YUV422; expected 1280 active PCLK edges/HREF");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG,
             "OV2640 GRAY8: XVCLK=%u Hz, sensor CLKRC=0x%02X, "
             "sensor_clock=%u Hz, DSP R_DVP_SP=0x%02X; "
             "calculated=%u.%03u fps, nominal_DVP_PCLK=%u Hz, nominal_active_transfer=%u us",
             (unsigned)OV2640_XVCLK_HZ, clkrc,
             (unsigned)OV2640_INTERNAL_CLOCK_HZ, dvp_sp,
             (unsigned)(OV2640_CALCULATED_MILLI_FPS / 1000U),
             (unsigned)(OV2640_CALCULATED_MILLI_FPS % 1000U),
             (unsigned)OV2640_DVP_PCLK_HZ,
             (unsigned)OV2640_DVP_TRANSFER_US);
    return ESP_OK;
}

static esp_err_t configure_sensor(sensor_t *sensor)
{
    switch (sensor->id.PID) {
    case OV7725_PID: return configure_ov7725(sensor);
    case OV7670_PID: return configure_ov7670(sensor);
    case OV2640_PID: return configure_ov2640(sensor);
    default: return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t write_all(int fd, const uint8_t *data, size_t bytes)
{
    size_t offset = 0;
    while (offset < bytes) {
        const ssize_t written = write(fd, data + offset, bytes - offset);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            return ESP_FAIL;
        }
        if (written == 0) {
            return ESP_FAIL;
        }
        offset += (size_t)written;
    }
    return ESP_OK;
}

static esp_err_t init_sdcard(void)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();

    /*
     * TTGO T8 V1.7 onboard slot 1: CLK=GPIO14, CMD=GPIO15, D0=GPIO2.
     * Use the fastest standard SD mode supported by this ESP-IDF version.
     * VGA GRAY8 at 5 fps requires 1.536 MB/s before filesystem overhead.
     */
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    slot.width = 1;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    const esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,
        .max_files = SD_MAX_OPEN_FILES,
        .allocation_unit_size = SD_FORMAT_ALLOCATION_BYTES,
    };

    ESP_RETURN_ON_ERROR(
        esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot, &mount,
                                &s_sd_card),
        TAG, "SDMMC 1-bit 40 MHz mount failed");

    sdmmc_card_print_info(stdout, s_sd_card);
    return ESP_OK;
}

static esp_err_t allocate_sd_dma_buffer(void)
{
    static const size_t candidate_sizes[] = {
        SD_DMA_BUFFER_PREFERRED_BYTES,
        16U * 1024U,
        SD_DMA_BUFFER_MIN_BYTES,
    };

    for (size_t i = 0; i < sizeof(candidate_sizes) / sizeof(candidate_sizes[0]);
         ++i) {
        const size_t bytes = candidate_sizes[i];
        uint8_t *buffer = heap_caps_aligned_alloc(
            4, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (buffer != NULL) {
            s_sd_dma_buffer = buffer;
            s_sd_dma_buffer_bytes = bytes;
            ESP_LOGI(TAG, "SD DMA staging buffer: %u bytes internal RAM",
                     (unsigned)bytes);
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG,
             "Cannot allocate an internal DMA staging buffer; largest DMA block=%u",
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    return ESP_ERR_NO_MEM;
}

static void release_record_pipeline(void)
{
    if (s_sd_dma_buffer != NULL) {
        heap_caps_free(s_sd_dma_buffer);
        s_sd_dma_buffer = NULL;
        s_sd_dma_buffer_bytes = 0;
    }
    s_ready_queue = NULL;
    s_record_events = NULL;
}

static esp_err_t init_record_pipeline(void)
{
    ESP_RETURN_ON_ERROR(allocate_sd_dma_buffer(), TAG,
                        "SD DMA staging allocation failed");

    s_ready_queue = xQueueCreateStatic(
        FRAME_QUEUE_CAPACITY, sizeof(ready_frame_t), s_ready_queue_storage,
        &s_ready_queue_control);
    s_record_events = xEventGroupCreateStatic(&s_record_events_control);
    if (s_ready_queue == NULL || s_record_events == NULL) {
        release_record_pipeline();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "Zero-copy recording ready: camera_fb=%u, ready_queue=%u, "
             "SD staging=%u bytes internal DMA RAM",
             (unsigned)CAMERA_FRAME_BUFFERS,
             (unsigned)FRAME_QUEUE_CAPACITY,
             (unsigned)s_sd_dma_buffer_bytes);
    return ESP_OK;
}

static esp_err_t reset_record_queues(void)
{
    /* A prior failed recording must never leave a driver framebuffer held. */
    ready_frame_t pending = {0};
    while (xQueueReceive(s_ready_queue, &pending, 0) == pdTRUE) {
        if (pending.frame != NULL) {
            esp_camera_fb_return(pending.frame);
        }
    }
    xQueueReset(s_ready_queue);
    xEventGroupClearBits(s_record_events,
                         CAPTURE_DONE_BIT | WRITER_DONE_BIT);
    return ESP_OK;
}

static esp_err_t preallocate_raw_file(int fd, int64_t *elapsed_us)
{
    const int64_t start_us = esp_timer_get_time();
    const off_t final_byte_offset = (off_t)(RECORD_FILE_BYTES - 1U);
    const uint8_t marker = 0;

    if (lseek(fd, final_byte_offset, SEEK_SET) != final_byte_offset ||
        write_all(fd, &marker, sizeof(marker)) != ESP_OK ||
        fsync(fd) != 0 || lseek(fd, 0, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "RAW preallocation failed: errno=%d (%s)",
                 errno, strerror(errno));
        return ESP_FAIL;
    }

    *elapsed_us = esp_timer_get_time() - start_us;
    return ESP_OK;
}

static esp_err_t open_next_raw_file(char *path, size_t path_bytes,
                                    int *fd_out)
{
    for (uint32_t id = 0; id < RECORD_MAX_SEQUENCE; ++id) {
        const int path_length = snprintf(path, path_bytes,
                                         SD_MOUNT_POINT "/R%07u.RAW",
                                         (unsigned)id);
        if (path_length < 0 || (size_t)path_length >= path_bytes) {
            return ESP_ERR_INVALID_SIZE;
        }

        const int fd = open(path, O_CREAT | O_EXCL | O_WRONLY, 0644);
        if (fd >= 0) {
            int64_t preallocate_us = 0;
            const esp_err_t err = preallocate_raw_file(fd, &preallocate_us);
            if (err != ESP_OK) {
                close(fd);
                unlink(path);
                return err;
            }
            ESP_LOGI(TAG,
                     "Prepared %s: preallocated %" PRIu64
                     " bytes in %" PRId64 " ms",
                     path, RECORD_FILE_BYTES, preallocate_us / 1000);
            *fd_out = fd;
            return ESP_OK;
        }
        if (errno != EEXIST) {
            ESP_LOGE(TAG, "Cannot create %s: errno=%d (%s)",
                     path, errno, strerror(errno));
            return ESP_FAIL;
        }
    }

    return ESP_ERR_NO_MEM;
}

static esp_err_t write_frame_to_sd(int fd, const uint8_t *pixels,
                                   int64_t *max_chunk_us)
{
    size_t offset = 0;
    while (offset < IMAGE_PIXELS) {
        const size_t remaining = IMAGE_PIXELS - offset;
        const size_t chunk = remaining < s_sd_dma_buffer_bytes
                                 ? remaining
                                 : s_sd_dma_buffer_bytes;

        /* SDMMC DMA cannot directly read classic ESP32 PSRAM efficiently. */
        memcpy(s_sd_dma_buffer, pixels + offset, chunk);
        const int64_t start_us = esp_timer_get_time();
        const esp_err_t err = write_all(fd, s_sd_dma_buffer, chunk);
        const int64_t chunk_us = esp_timer_get_time() - start_us;
        if (chunk_us > *max_chunk_us) {
            *max_chunk_us = chunk_us;
        }
        if (err != ESP_OK) {
            return err;
        }
        offset += chunk;
    }
    return ESP_OK;
}

static esp_err_t init_camera(sensor_t **sensor_out)
{
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "PSRAM is required for the VGA camera framebuffers");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "PSRAM: %u bytes total, %u bytes free, largest block=%u",
             (unsigned)esp_psram_get_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));

#if USE_OV2640_ONBOARD_XVCLK
    ESP_LOGI(TAG,
             "Camera clock source: OV2640 onboard 12 MHz XVCLK; "
             "ESP32 GPIO XCLK output disabled; HAL sampling hint=%u Hz",
             (unsigned)CAMERA_CONFIG_XCLK_HZ);
#else
    ESP_LOGI(TAG, "Camera clock source: ESP32 GPIO%d at %u Hz",
             CAM_PIN_XCLK, (unsigned)ESP32_CAMERA_XCLK_HZ);
#endif

    const esp_err_t err = esp_camera_init(&s_camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(err));
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL || strcmp(sensor_name(sensor->id.PID), "unsupported") == 0) {
        ESP_LOGE(TAG, "Only OV2640, OV7670 and OV7725 are supported");
        return ESP_ERR_NOT_SUPPORTED;
    }

#if USE_OV2640_ONBOARD_XVCLK
    if (sensor->id.PID != OV2640_PID) {
        ESP_LOGE(TAG,
                 "This build disables ESP32 XCLK for the self-clocked OV2640; "
                 "set USE_OV2640_ONBOARD_XVCLK=0 for OV7670/OV7725");
        return ESP_ERR_INVALID_STATE;
    }
#endif

    ESP_LOGI(TAG,
             "Detected %s PID=0x%04x; sensor YUV422=%u bytes, stored GRAY8=%u bytes",
             sensor_name(sensor->id.PID), sensor->id.PID,
             (unsigned)SENSOR_YUV422_BYTES, (unsigned)IMAGE_PIXELS);
    ESP_RETURN_ON_ERROR(configure_sensor(sensor), TAG,
                        "Sensor timing/output setup failed");
    *sensor_out = sensor;
    return ESP_OK;
}

static esp_err_t set_auto_controls(sensor_t *sensor, bool enable)
{
    if (sensor->set_gain_ctrl == NULL || sensor->set_exposure_ctrl == NULL ||
        sensor->set_whitebal == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* OV7725 returns its resulting state (1 when enabled), not only 0/-1. */
    if (sensor->set_gain_ctrl(sensor, enable) < 0 ||
        sensor->set_exposure_ctrl(sensor, enable) < 0 ||
        sensor->set_whitebal(sensor, enable) < 0) {
        return ESP_FAIL;
    }

    if (sensor->id.PID == OV2640_PID) {
        uint8_t com8 = 0, ctrl1 = 0;
        ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_SENSOR_COM8,
                                            0xFF, &com8),
                            TAG, "Cannot verify OV2640 COM8");
        ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_DSP_CTRL1,
                                            0xFF, &ctrl1),
                            TAG, "Cannot verify OV2640 DSP CTRL1");
        const uint8_t expected_com8 = enable ? OV2640_COM8_AEC_AGC_MASK : 0;
        const uint8_t expected_ctrl1 = enable ? OV2640_CTRL1_AWB_MASK : 0;
        if ((com8 & OV2640_COM8_AEC_AGC_MASK) != expected_com8 ||
            (ctrl1 & OV2640_CTRL1_AWB_MASK) != expected_ctrl1) {
            ESP_LOGE(TAG, "OV2640 auto-control verification failed: "
                     "COM8=0x%02X CTRL1=0x%02X", com8, ctrl1);
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGI(TAG, "OV2640 AE/AGC/AWB %s: COM8=0x%02X CTRL1=0x%02X",
                 enable ? "enabled" : "locked", com8, ctrl1);
    } else {
        uint8_t com8 = 0;
        ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OMNIVISION_REG_COM8,
                                            0xFF, &com8),
                            TAG, "Cannot verify COM8");
        const uint8_t expected = enable ? OMNIVISION_COM8_AUTO_MASK : 0;
        if ((com8 & OMNIVISION_COM8_AUTO_MASK) != expected) {
            ESP_LOGE(TAG, "%s auto-control verification failed: COM8=0x%02X",
                     sensor_name(sensor->id.PID), com8);
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGI(TAG, "%s AE/AGC/AWB %s: COM8=0x%02X",
                 sensor_name(sensor->id.PID),
                 enable ? "enabled" : "locked", com8);
    }
    return ESP_OK;
}

static bool frame_is_valid(const camera_fb_t *frame)
{
    return frame != NULL && frame->format == PIXFORMAT_GRAYSCALE &&
           frame->width == IMAGE_WIDTH && frame->height == IMAGE_HEIGHT &&
           frame->len == IMAGE_PIXELS;
}

static int64_t frame_timestamp_us(const camera_fb_t *frame)
{
    return (int64_t)frame->timestamp.tv_sec * 1000000LL +
           (int64_t)frame->timestamp.tv_usec;
}

static esp_err_t discard_warmup_frames(sensor_t *sensor)
{
    ESP_LOGI(TAG,
             "Warm-up: discard %u complete frames with AE/AGC/AWB enabled",
             (unsigned)CAMERA_WARMUP_FRAMES);

    int64_t first_us = 0;
    int64_t last_us = 0;

    for (uint32_t complete = 0; complete < CAMERA_WARMUP_FRAMES; ++complete) {
        /*
         * esp_camera_fb_get() returns only frames accepted by cam_hal.
         * A frame rejected by cam_hal for FB-SIZE mismatch is never exposed
         * here; repeated rejected frames eventually appear as a NULL timeout.
         */
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == NULL) {
            ESP_LOGE(TAG, "Warm-up timed out after %u/%u complete frames",
                     (unsigned)complete, (unsigned)CAMERA_WARMUP_FRAMES);
            return ESP_ERR_TIMEOUT;
        }

        const bool valid = frame_is_valid(frame);
        const int64_t timestamp_us = frame_timestamp_us(frame);
        if (!valid) {
            ESP_LOGE(TAG,
                     "Invalid warm-up frame: format=%d width=%u height=%u len=%u",
                     (int)frame->format, (unsigned)frame->width,
                     (unsigned)frame->height, (unsigned)frame->len);
            esp_camera_fb_return(frame);
            return ESP_ERR_INVALID_SIZE;
        }

        if (complete == 1U) {
            first_us = timestamp_us;
        }
        if (complete == CAMERA_WARMUP_FRAMES - 1U) {
            last_us = timestamp_us;
        }
        esp_camera_fb_return(frame);
    }

    if (last_us <= first_us) {
        ESP_LOGE(TAG, "Invalid warm-up timestamps: first=%" PRId64
                 " us last=%" PRId64 " us", first_us, last_us);
        return ESP_ERR_INVALID_STATE;
    }

    const uint32_t intervals = CAMERA_WARMUP_FRAMES - 2U;
    const uint64_t elapsed_us = (uint64_t)(last_us - first_us);
    const uint32_t milli_fps =
        (uint32_t)(((uint64_t)intervals * 1000000000ULL) / elapsed_us);

    /* Diagnostic only: the 10-second test below reports delivery loss. */
    ESP_LOGI(TAG,
             "%s warm-up delivery: %u intervals in %u ms, measured=%u.%03u fps",
             sensor_name(sensor->id.PID), (unsigned)intervals,
             (unsigned)(elapsed_us / 1000ULL),
             (unsigned)(milli_fps / 1000U),
             (unsigned)(milli_fps % 1000U));
    return ESP_OK;
}

static void init_record_button(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << TEST_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
}

static int64_t wait_for_record_trigger(void)
{
    ESP_LOGI(TAG,
             "Press and release GPIO0 to record %u seconds "
             "(expected about %u GRAY8 frames)",
             (unsigned)RECORD_DURATION_SECONDS,
             (unsigned)RECORD_EXPECTED_FRAMES);

    for (;;) {
        while (gpio_get_level(TEST_BUTTON_GPIO) != 0) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
        if (gpio_get_level(TEST_BUTTON_GPIO) != 0) {
            continue;
        }

        ESP_LOGI(TAG, "GPIO0 pressed; release it to start recording");
        while (gpio_get_level(TEST_BUTTON_GPIO) == 0) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
        if (gpio_get_level(TEST_BUTTON_GPIO) != 0) {
            return esp_timer_get_time();
        }
    }
}

static esp_err_t run_startup_fps_test(void)
{
    dma_receive_test_stats_t stats = {0};
    const int64_t trigger_us = esp_timer_get_time();
    const int64_t deadline_us =
        trigger_us + (int64_t)TEST_DURATION_SECONDS * 1000000LL;
    int64_t previous_seen_us = 0;

    ESP_LOGI(TAG,
             "Startup FPS test: timestamp window=%u s, "
             "expected~%u frames at %u fps; SD is not mounted",
             (unsigned)TEST_DURATION_SECONDS,
             (unsigned)EXPECTED_FRAME_COUNT,
             (unsigned)TARGET_FPS);

    for (;;) {
        stats.get_calls++;
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == NULL) {
            stats.timeout_count++;
            ESP_LOGE(TAG,
                     "esp_camera_fb_get() timed out after %u complete "
                     "in-window frames",
                     (unsigned)stats.complete_frames);
            break;
        }

        const int64_t timestamp_us = frame_timestamp_us(frame);
        const int64_t received_us = esp_timer_get_time();

        if (timestamp_us <= 0 || timestamp_us > received_us ||
            (previous_seen_us != 0 && timestamp_us <= previous_seen_us)) {
            stats.timestamp_errors++;
            ESP_LOGE(TAG,
                     "Invalid frame timestamp: current=%" PRId64
                     " previous=%" PRId64 " received=%" PRId64,
                     timestamp_us, previous_seen_us, received_us);
            esp_camera_fb_return(frame);
            continue;
        }
        previous_seen_us = timestamp_us;

        if (!frame_is_valid(frame)) {
            stats.invalid_frames++;
            ESP_LOGE(TAG,
                     "Invalid returned frame: format=%d width=%u height=%u len=%u",
                     (int)frame->format, (unsigned)frame->width,
                     (unsigned)frame->height, (unsigned)frame->len);
            esp_camera_fb_return(frame);
            continue;
        }

        if (timestamp_us < trigger_us) {
            stats.stale_frames++;
            esp_camera_fb_return(frame);
            continue;
        }

        if (timestamp_us >= deadline_us) {
            stats.after_deadline_frames++;
            esp_camera_fb_return(frame);
            break;
        }

        int64_t interval_us = 0;
        if (stats.complete_frames == 0U) {
            stats.first_frame_us = timestamp_us;
        } else {
            interval_us = timestamp_us - stats.last_frame_us;
            if (stats.complete_frames == 1U ||
                interval_us < stats.min_interval_us) {
                stats.min_interval_us = interval_us;
            }
            if (interval_us > stats.max_interval_us) {
                stats.max_interval_us = interval_us;
            }

            if (interval_us > GAP_THRESHOLD_US) {
                stats.gap_events++;
                const uint32_t represented_periods =
                    (uint32_t)((interval_us +
                                EXPECTED_FRAME_PERIOD_US / 2LL) /
                               EXPECTED_FRAME_PERIOD_US);
                if (represented_periods > 1U) {
                    stats.estimated_missing_frames +=
                        represented_periods - 1U;
                }
                ESP_LOGW(TAG,
                         "Delivery gap before complete frame %u: "
                         "interval=%" PRId64 " us, estimated_missing=%u",
                         (unsigned)(stats.complete_frames + 1U),
                         interval_us,
                         (unsigned)stats.estimated_missing_frames);
            }
        }

        stats.last_frame_us = timestamp_us;
        stats.complete_frames++;

        if (stats.complete_frames == 1U ||
            (stats.complete_frames % 10U) == 0U) {
            ESP_LOGI(TAG,
                     "FPS test frame %u: timestamp_offset=%" PRId64
                     " us interval=%" PRId64 " us",
                     (unsigned)stats.complete_frames,
                     timestamp_us - trigger_us,
                     interval_us);
        }

        /* No application copy or SD write: isolate camera HAL/DMA delivery. */
        esp_camera_fb_return(frame);
    }

    const int64_t span_us =
        stats.complete_frames > 1U
            ? stats.last_frame_us - stats.first_frame_us
            : 0;
    const uint32_t timestamp_milli_fps =
        stats.complete_frames > 1U && span_us > 0
            ? (uint32_t)(((uint64_t)(stats.complete_frames - 1U) *
                          1000000000ULL) /
                         (uint64_t)span_us)
            : 0;
    const uint32_t window_milli_fps =
        (uint32_t)(((uint64_t)stats.complete_frames * 1000ULL) /
                   TEST_DURATION_SECONDS);
    const uint32_t deficit =
        stats.complete_frames < EXPECTED_FRAME_COUNT
            ? EXPECTED_FRAME_COUNT - stats.complete_frames
            : 0U;

    const uint32_t target_milli_fps = TARGET_FPS * 1000U;
    const bool fps_in_range =
        timestamp_milli_fps >= target_milli_fps - FPS_TOLERANCE_MILLI &&
        timestamp_milli_fps <= target_milli_fps + FPS_TOLERANCE_MILLI;
    const bool pass =
        stats.complete_frames >= EXPECTED_FRAME_COUNT &&
        stats.gap_events == 0U &&
        stats.invalid_frames == 0U &&
        stats.timeout_count == 0U &&
        stats.timestamp_errors == 0U &&
        fps_in_range;

    ESP_LOGI(TAG,
             "DMA test result: %s complete=%u expected~%u deficit=%u "
             "window_fps=%u.%03u timestamp_fps=%u.%03u",
             pass ? "PASS" : "FAIL",
             (unsigned)stats.complete_frames,
             (unsigned)EXPECTED_FRAME_COUNT,
             (unsigned)deficit,
             (unsigned)(window_milli_fps / 1000U),
             (unsigned)(window_milli_fps % 1000U),
             (unsigned)(timestamp_milli_fps / 1000U),
             (unsigned)(timestamp_milli_fps % 1000U));
    ESP_LOGI(TAG,
             "DMA test details: get_calls=%u stale=%u after_deadline=%u "
             "invalid=%u timeout=%u timestamp_error=%u gaps=%u "
             "estimated_missing=%u interval_min=%" PRId64
             " us interval_max=%" PRId64 " us",
             (unsigned)stats.get_calls,
             (unsigned)stats.stale_frames,
             (unsigned)stats.after_deadline_frames,
             (unsigned)stats.invalid_frames,
             (unsigned)stats.timeout_count,
             (unsigned)stats.timestamp_errors,
             (unsigned)stats.gap_events,
             (unsigned)stats.estimated_missing_frames,
             stats.min_interval_us,
             stats.max_interval_us);

    return pass ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static void capture_task(void *argument)
{
    (void)argument;
    recording_stats_t *stats = &s_recording.stats;
    int64_t previous_seen_us = 0;

    for (;;) {
        stats->get_calls++;
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == NULL) {
            stats->capture_timeouts++;
            stats->capture_result = ESP_ERR_TIMEOUT;
            ESP_LOGE(TAG, "Capture timed out after %u in-window frames",
                     (unsigned)stats->in_window_frames);
            break;
        }

        const int64_t timestamp_us = frame_timestamp_us(frame);
        const int64_t received_us = esp_timer_get_time();
        if (timestamp_us <= 0 || timestamp_us > received_us ||
            (previous_seen_us != 0 && timestamp_us <= previous_seen_us)) {
            stats->timestamp_errors++;
            if (stats->capture_result == ESP_OK) {
                stats->capture_result = ESP_ERR_INVALID_STATE;
            }
            esp_camera_fb_return(frame);
            continue;
        }
        previous_seen_us = timestamp_us;

        if (!frame_is_valid(frame)) {
            stats->invalid_frames++;
            if (stats->capture_result == ESP_OK) {
                stats->capture_result = ESP_ERR_INVALID_SIZE;
            }
            esp_camera_fb_return(frame);
            continue;
        }

        if (timestamp_us < s_recording.trigger_us) {
            stats->stale_frames++;
            esp_camera_fb_return(frame);
            continue;
        }
        if (timestamp_us >= s_recording.deadline_us) {
            esp_camera_fb_return(frame);
            break;
        }

        int64_t interval_us = 0;
        if (stats->in_window_frames == 0U) {
            stats->first_frame_us = timestamp_us;
        } else {
            interval_us = timestamp_us - stats->last_frame_us;
            if (stats->in_window_frames == 1U ||
                interval_us < stats->min_interval_us) {
                stats->min_interval_us = interval_us;
            }
            if (interval_us > stats->max_interval_us) {
                stats->max_interval_us = interval_us;
            }
            if (interval_us > GAP_THRESHOLD_US) {
                stats->frame_gaps++;
            }
        }
        stats->last_frame_us = timestamp_us;

        const uint32_t frame_id = stats->in_window_frames++;
        const ready_frame_t ready_frame = {
            .frame = frame,
            .frame_id = frame_id,
            .timestamp_us = timestamp_us,
        };
        if (xQueueSend(s_ready_queue, &ready_frame, 0) != pdTRUE) {
            stats->pool_overflows++;
            esp_camera_fb_return(frame);
            if (stats->pool_overflows == 1U ||
                (stats->pool_overflows % 10U) == 0U) {
                ESP_LOGW(TAG,
                         "Ready queue full at frame %u: dropped=%u ready=%u/%u",
                         (unsigned)frame_id,
                         (unsigned)stats->pool_overflows,
                         (unsigned)uxQueueMessagesWaiting(s_ready_queue),
                         (unsigned)FRAME_QUEUE_CAPACITY);
            }
            continue;
        }

        /* Writer owns this driver framebuffer until esp_camera_fb_return(). */
        stats->enqueued_frames++;
        const uint32_t ready = uxQueueMessagesWaiting(s_ready_queue);
        if (ready > stats->peak_ready_slots) {
            stats->peak_ready_slots = ready;
        }
    }

    stats->capture_end_us = esp_timer_get_time();
    xEventGroupSetBits(s_record_events, CAPTURE_DONE_BIT);
    vTaskDelete(NULL);
}

static void writer_task(void *argument)
{
    (void)argument;
    recording_stats_t *stats = &s_recording.stats;

    for (;;) {
        ready_frame_t ready_frame = {0};
        if (xQueueReceive(s_ready_queue, &ready_frame,
                          pdMS_TO_TICKS(RECORD_QUEUE_WAIT_MS)) == pdTRUE) {
            if (stats->writer_result == ESP_OK) {
                const int64_t frame_write_start_us = esp_timer_get_time();
                const esp_err_t err = write_frame_to_sd(
                    s_recording.fd, ready_frame.frame->buf,
                    &stats->max_sd_chunk_us);
                const int64_t frame_write_us =
                    esp_timer_get_time() - frame_write_start_us;
                if (frame_write_us > stats->max_sd_frame_us) {
                    stats->max_sd_frame_us = frame_write_us;
                }
                if (err == ESP_OK) {
                    stats->saved_frames++;
                    if ((stats->saved_frames % 10U) == 0U) {
                        ESP_LOGI(TAG,
                                 "Saved %u frames; ready=%u/%u",
                                 (unsigned)stats->saved_frames,
                                 (unsigned)uxQueueMessagesWaiting(
                                     s_ready_queue),
                                 (unsigned)FRAME_QUEUE_CAPACITY);
                    }
                } else {
                    stats->writer_result = err;
                    ESP_LOGE(TAG,
                             "SD write failed at frame %u: errno=%d (%s)",
                             (unsigned)ready_frame.frame_id,
                             errno, strerror(errno));
                }
            }

            if (ready_frame.frame != NULL) {
                esp_camera_fb_return(ready_frame.frame);
            }
            continue;
        }

        if ((xEventGroupGetBits(s_record_events) & CAPTURE_DONE_BIT) != 0 &&
            uxQueueMessagesWaiting(s_ready_queue) == 0U) {
            break;
        }
    }

    const off_t saved_bytes =
        (off_t)((uint64_t)stats->saved_frames * IMAGE_PIXELS);
    if (ftruncate(s_recording.fd, saved_bytes) != 0 ||
        fsync(s_recording.fd) != 0) {
        if (stats->writer_result == ESP_OK) {
            stats->writer_result = ESP_FAIL;
        }
        ESP_LOGE(TAG, "RAW finalize failed: errno=%d (%s)",
                 errno, strerror(errno));
    }

    stats->writer_end_us = esp_timer_get_time();
    xEventGroupSetBits(s_record_events, WRITER_DONE_BIT);
    vTaskDelete(NULL);
}

static esp_err_t record_raw_sequence(void)
{
    char path[RECORD_PATH_BYTES];
    int fd = -1;
    ESP_RETURN_ON_ERROR(open_next_raw_file(path, sizeof(path), &fd),
                        TAG, "Cannot prepare RAW recording file");

    esp_err_t err = reset_record_queues();
    if (err != ESP_OK) {
        close(fd);
        unlink(path);
        return err;
    }

    memset(&s_recording, 0, sizeof(s_recording));
    s_recording.fd = fd;
    s_recording.stats.capture_result = ESP_OK;
    s_recording.stats.writer_result = ESP_OK;
    s_recording.trigger_us = wait_for_record_trigger();
    s_recording.deadline_us =
        s_recording.trigger_us +
        (int64_t)RECORD_DURATION_SECONDS * 1000000LL;

    ESP_LOGI(TAG,
             "Recording to %s: timestamp window=%u s, expected~%u frames, "
             "camera_fb=%u ready_queue=%u",
             path, (unsigned)RECORD_DURATION_SECONDS,
             (unsigned)RECORD_EXPECTED_FRAMES,
             (unsigned)CAMERA_FRAME_BUFFERS,
             (unsigned)FRAME_QUEUE_CAPACITY);

    if (xTaskCreatePinnedToCore(writer_task, "sd_writer",
                                WRITER_TASK_STACK_BYTES, NULL,
                                WRITER_TASK_PRIORITY, NULL,
                                WRITER_TASK_CORE) != pdPASS) {
        close(fd);
        unlink(path);
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreatePinnedToCore(capture_task, "camera_capture",
                                CAPTURE_TASK_STACK_BYTES, NULL,
                                CAPTURE_TASK_PRIORITY, NULL,
                                CAPTURE_TASK_CORE) != pdPASS) {
        s_recording.stats.capture_result = ESP_ERR_NO_MEM;
        xEventGroupSetBits(s_record_events, CAPTURE_DONE_BIT);
    }

    xEventGroupWaitBits(s_record_events,
                        CAPTURE_DONE_BIT | WRITER_DONE_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    if (close(fd) != 0 && s_recording.stats.writer_result == ESP_OK) {
        s_recording.stats.writer_result = ESP_FAIL;
    }

    recording_stats_t *stats = &s_recording.stats;
    const int64_t timestamp_span_us =
        stats->in_window_frames > 1U
            ? stats->last_frame_us - stats->first_frame_us
            : 0;
    const uint32_t timestamp_milli_fps =
        stats->in_window_frames > 1U && timestamp_span_us > 0
            ? (uint32_t)(((uint64_t)(stats->in_window_frames - 1U) *
                          1000000000ULL) /
                         (uint64_t)timestamp_span_us)
            : 0U;

    ESP_LOGI(TAG,
             "Recording finished: file=%s in_window=%u enqueued=%u saved=%u "
             "dropped=%u timestamp_fps=%u.%03u wall=%" PRId64 " ms",
             path,
             (unsigned)stats->in_window_frames,
             (unsigned)stats->enqueued_frames,
             (unsigned)stats->saved_frames,
             (unsigned)stats->pool_overflows,
             (unsigned)(timestamp_milli_fps / 1000U),
             (unsigned)(timestamp_milli_fps % 1000U),
             (stats->writer_end_us - s_recording.trigger_us) / 1000);
    ESP_LOGI(TAG,
             "Recording details: stale=%u invalid=%u timeout=%u "
             "timestamp_error=%u gaps=%u queue_error=%u peak_ready=%u/%u "
             "sd_frame_max=%" PRId64 " us sd_chunk_max=%" PRId64 " us",
             (unsigned)stats->stale_frames,
             (unsigned)stats->invalid_frames,
             (unsigned)stats->capture_timeouts,
             (unsigned)stats->timestamp_errors,
             (unsigned)stats->frame_gaps,
             (unsigned)stats->queue_errors,
             (unsigned)stats->peak_ready_slots,
             (unsigned)FRAME_QUEUE_CAPACITY,
             stats->max_sd_frame_us,
             stats->max_sd_chunk_us);

    if (stats->capture_result != ESP_OK) {
        return stats->capture_result;
    }
    if (stats->writer_result != ESP_OK) {
        return stats->writer_result;
    }
    if (stats->saved_frames < RECORD_EXPECTED_FRAMES ||
        stats->saved_frames != stats->in_window_frames ||
        stats->pool_overflows != 0U || stats->queue_errors != 0U ||
        stats->frame_gaps != 0U) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

void app_main(void)
{
    sensor_t *sensor = NULL;
    esp_err_t err = init_camera(&sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }

    err = set_auto_controls(sensor, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot enable camera auto controls: %s",
                 esp_err_to_name(err));
        esp_camera_deinit();
        return;
    }

    err = discard_warmup_frames(sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera warm-up failed: %s", esp_err_to_name(err));
        esp_camera_deinit();
        return;
    }

    err = set_auto_controls(sensor, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot lock camera controls: %s",
                 esp_err_to_name(err));
        esp_camera_deinit();
        return;
    }

    /* Qualification must pass before any SD mount or filesystem access. */
    err = run_startup_fps_test();
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "Startup FPS test failed; SD capture remains disabled");
        esp_camera_deinit();
        return;
    }

    err = init_sdcard();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD initialization failed: %s",
                 esp_err_to_name(err));
        esp_camera_deinit();
        return;
    }

    err = init_record_pipeline();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Recording pipeline initialization failed: %s",
                 esp_err_to_name(err));
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_sd_card);
        s_sd_card = NULL;
        esp_camera_deinit();
        return;
    }

    init_record_button();

    while (true) {
        err = record_raw_sequence();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Recording completed with error: %s",
                     esp_err_to_name(err));
        }
    }
}