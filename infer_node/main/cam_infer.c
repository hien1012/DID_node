/*
 * 相機推論管線 (ESP32 / TTGO T8 版，由 DID_NODE/infer_node/main/cam_infer.c 移植)。
 *
 * 上機、感測器設定、暖機、開機 FPS 測試和錄影韌體 main/cam_record.c 相同 (兩份的感測器
 * 程式本來就一樣；ESP32 專屬的是 XCLK 提示值、5 s FPS 測試與 pause 時停 XCLK)。輸入永遠是
 * 感測器直出的 GRAY8 640x480，FD 與 CNN 都直接讀它，沒有 JPEG 解碼。差別在暖機之後：
 * capture + write 換成 capture + detect + infer 並連續執行，只有低信賴度的幀才寫 SD
 * (INFER_SAVE_NON_BACKGROUND=1 時非背景的幀也寫；存 raw 或 JPEG 由 INFER_CAM_SAMPLE_JPEG 決定，只影響存檔)。
 *
 *   cam_task (驅動, Core 1)：I2S0 平行輸入 DMA → 取 Y → PSRAM fb
 *   camera_capture (Core 1, 優先權 6)：取 fb 指標、檢查時戳，
 *       送進 s_ready_queue，不搬像素
 *   cam_detect (Core 1, 優先權 5, 常駐)：下採樣 ×2 → FD → 追蹤器。
 *       沒有候選就歸還 fb；有候選就把 fb 和候選包成 frame_job_t 送進 s_cnn_queue
 *   cam_infer (Core 0, 優先權 1, 常駐)：crop 64x64 → CNN → 結果立刻回傳。
 *       全部直接讀 fb (零拷貝)。有低信賴 ROI 就把工作交給 cam_writer，否則歸還 fb
 *   cam_writer (Core 0, 優先權 5)：全幀 + ROI + meta 寫 SD，
 *       寫完才歸還 fb
 *
 * CNN 只有一條工作線：來不及時候選留在追蹤器，等有空的 frame_job_t 再送，
 * FD 和追蹤不必等 CNN，所以不會因為 CNN 而丟幀。
 *
 * 參數與 FD_did_cam_v4/main/main_functions.cc 的相機模式相同。
 */
#include "cam_infer.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "esp_camera.h"         /* 官方相機驅動介面 (相機初始化、擷取影格) */
#include "cam_hal.h"            /* 停止/恢復 VSYNC 與 I2S DMA 擷取 */
#include "driver/ledc.h"        /* 暫停/恢復 XCLK (LEDC) 輸出 */
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "board_pins.h"
#include "camera_registers.h"
#include "cam_model.h"
#include "fd_mix.h"
#include "infer_config.h"
#include "infer_time.h"
#include "node_storage.h"
#include "result_report.h"
#include "sample_store.h"
#include "stage2.h"
#include "task_usage.h"
#if INFER_CAM_SAMPLE_JPEG
#include "img_converters.h"
#endif

/* --- 影像規格參數 --- */
#define IMAGE_WIDTH             CAM_IMAGE_WIDTH             /* 影像寬度：640 像素 (VGA 規格) */
#define IMAGE_HEIGHT            CAM_IMAGE_HEIGHT            /* 影像高度：480 像素 (VGA 規格) */
#define IMAGE_PIXELS            CAM_IMAGE_FRAME_BYTES       /* 每一幀灰階像素總數：640*480 = 307,200 位元組 (Bytes) */
#define SENSOR_YUV422_BYTES     (IMAGE_PIXELS * 2U)         /* 感測器原始輸出為 YUV422，每像素佔 2 位元組 = 614,400 位元組 */

/*
 * --- 相機感測器時脈源選擇 (Clock-Source Configuration) ---
 *
 * 說明：
 * 本專案硬體可能搭載不同的相機感測器模組 (如 OV2640、OV7670、OV7725)。
 * 部分 OV2640 模組自帶 12 MHz 石英震盪器 (Onboard Oscillator) 直連 XVCLK。
 * 若感測器已有自帶震盪器，ESP32 絕不可再由內部輸出 XCLK，否則兩組時脈訊號會互相打架短路。
 * 反之，OV7670/OV7725 模組需要由 ESP32 的 LEDC 產生 16 MHz XCLK 供給它。
 */
#define USE_OV2640_ONBOARD_XVCLK 0                          /* 0: 由 ESP32 產生 16MHz XCLK (OV7670/OV7725); 1: 使用 OV2640 板載 12MHz 時脈 */
#define ESP32_CAMERA_XCLK_HZ     16000000U                  /* ESP32 輸出的 XCLK 時脈頻率：16 MHz */
#define OV2640_ONBOARD_XVCLK_HZ  12000000U                  /* OV2640 外部板載震盪器頻率：12 MHz */
#define OV2640_HAL_XCLK_HINT_HZ  12000000U                  /* 提供給相機底層驅動的時脈提示值 (ESP32：超過 10MHz 會影響 I2S 採樣模式選擇；同 cam_record.c) */

#if USE_OV2640_ONBOARD_XVCLK
#define CAMERA_CONFIG_XCLK_PIN   (-1)                       /* -1 代表不啟用 ESP32 的 XCLK 輸出腳位，由板載震盪器自行驅動 */
#define CAMERA_CONFIG_XCLK_HZ    OV2640_HAL_XCLK_HINT_HZ
#else
#define CAMERA_CONFIG_XCLK_PIN   CAM_PIN_XCLK               /* 指向 camera_pinout.h 中定義的 XCLK 實體 GPIO 腳位 */
#define CAMERA_CONFIG_XCLK_HZ    ESP32_CAMERA_XCLK_HZ       /* 設定為 16 MHz */
#endif

/* --- 錄影與影格率驗證參數 --- */
#define CAMERA_WARMUP_FRAMES    100U                         /* 開機暖機幀數：丟棄前 10 幀，讓感測器自動曝光/白平衡穩定 */
#define CAMERA_FRAME_BUFFERS    8U                          /* 配置於外部 PSRAM 的影格緩衝區數量 (總計 8 * 300KB ≈ 2.4MB；ESP32 只映射 4 MB PSRAM，不用錄影韌體的 10 個) */
#define TEST_DURATION_SECONDS   5U                          /* 開機硬體資格測試時間：5 秒 (同 cam_record.c) */
#define TARGET_FPS              CAM_TARGET_FPS              /* 目標影格率：每秒 5 幀 */
#define EXPECTED_FRAME_COUNT     (TEST_DURATION_SECONDS * TARGET_FPS) /* 5 秒預期應收到的影格數：25 幀 */
#define EXPECTED_FRAME_PERIOD_US (1000000LL / TARGET_FPS)   /* 每一幀的理想間隔微秒數：1,000,000 / 5 = 200,000 us (200 毫秒) */
#define GAP_THRESHOLD_US         (EXPECTED_FRAME_PERIOD_US * 3LL / 2LL) /* 掉幀判定門檻：間隔超過 1.5 週期 (300 毫秒) 即視為掉幀事件 */
#define FPS_TOLERANCE_MILLI      250U                       /* 影格率容許誤差：±0.25 FPS (4.75 ~ 5.25 FPS 視為合格) */
/* XCLK 停止後重新輸出，感測器內部時序需要約兩幀才穩定；曝光已在開機鎖定，不必重新暖機 */
#define CAMERA_XCLK_SETTLE_MS    500U

/* --- SD 卡寫入緩衝參數 --- */
#define SD_DMA_BUFFER_BYTES      CAM_INFER_STAGING_BYTES       /* 內部 DMA RAM：SD 不能直接 DMA 讀 PSRAM */

/*
 * --- 佇列容量 ---
 * 8 個 fb：1 個 DMA 正在收、1 個下一個 VSYNC 備用、1 個 cam_detect 正在處理，
 * 其餘 5 個分給排隊：等偵測 2 個、frame_job_t 3 個 (等 CNN、CNN 中或等寫入都佔一個 fb)。
 * 等偵測的佇列滿才丟幀 (不擋硬體)；工作用完時候選留在追蹤器，不丟幀。
 */
#define READY_QUEUE_CAPACITY          2U
#define FRAME_JOB_COUNT               3U
/* 恢復相機後丟掉這段時間的幀：任務就緒前留在驅動裡的舊幀 */
#define START_MARGIN_US               300000LL
/* 每隔這麼多幀回報一次累計統計 (5 fps = 60 s) */
#define STATS_PERIOD_FRAMES           300U
/* 整體 task CPU 占比的一輪：10 幀 = 5 fps 下 2 s (和麥克風的視窗一樣長)；表由 cam_infer 每分鐘印 */
#define USAGE_CYCLE_FRAMES            10U
/* cam_infer 回報 CNN 分層與 task 占比的週期；一直沒有候選時它也要醒來看一下 */
#define CNN_REPORT_PERIOD_MS          60000U

/* --- FD / ROI 參數 (FD_did_cam_v4 相機模式) --- */
#define FD_DOWNSCALE_FACTOR           2
#define FD_PROC_WIDTH                 (FD_WIDTH / FD_DOWNSCALE_FACTOR)
#define FD_PROC_HEIGHT                (FD_HEIGHT / FD_DOWNSCALE_FACTOR)
#define FD_PROC_BYTES                 ((size_t)FD_PROC_WIDTH * (size_t)FD_PROC_HEIGHT)
#define ROI_EXPAND_PX                 5
#define ROI_RESIZE_BILINEAR           1

/* --- FreeRTOS 任務堆疊與優先權設定 --- */
#define CAPTURE_TASK_STACK_BYTES      4096U
#define DETECT_TASK_STACK_BYTES       8192U                 /* FD + qsort + 統計回報的 snprintf；看 Stack free log 再縮 */
#define INFER_TASK_STACK_BYTES        10240U                /* TFLM Invoke + 分層統計 */
#if INFER_CAM_SAMPLE_JPEG
#define WRITER_TASK_STACK_BYTES       14336U                /* 再加上 stack 上的 jpge::jpeg_encoder (約 8 KB) */
#else
#define WRITER_TASK_STACK_BYTES       6144U                 /* FATFS + meta snprintf */
#endif
#define CAPTURE_TASK_PRIORITY         6U
#define DETECT_TASK_PRIORITY          5U                    /* 低於 camera_capture 和 cam_task，不影響取幀 */
#define WRITER_TASK_PRIORITY          5U
/*
 * 候選多的時候 cam_infer 會一直有工作做。Core 0 上的 main (優先權 1，上傳與校時) 綁死在
 * 這顆核心，所以 cam_infer 不能比它高，否則上傳會被擠到跑不動；同優先權時兩者輪流執行。
 */
#define INFER_TASK_PRIORITY           1U

#if CONFIG_FREERTOS_UNICORE
#define CAPTURE_TASK_CORE             0
#define DETECT_TASK_CORE              0
#define INFER_TASK_CORE               0
#define WRITER_TASK_CORE              0
#else
#define CAPTURE_TASK_CORE             1                     /* 和驅動的 cam_task 同核心，不受 Wi-Fi 和推論影響 */
#define DETECT_TASK_CORE              1                     /* 要跟上每一幀，不被 CNN 和 Wi-Fi 擋住 */
#define INFER_TASK_CORE               0                     /* 晚一點沒關係，被 Wi-Fi 搶佔也不會掉幀 */
#define WRITER_TASK_CORE              0                     /* Core 0 負責較慢的 SDMMC 檔案寫入 */
#endif

#define SAMPLE_META_BYTES             1536U

_Static_assert(CAM_MODEL_ROI_SIZE == 64, "The ROI pipeline assumes a 64x64 model input");
_Static_assert(CAM_MODEL_MAX_OPS == INFER_CNN_OPS_MAX, "Op timing tables must match");
_Static_assert(CAM_IMAGE_WIDTH == FD_WIDTH && CAM_IMAGE_HEIGHT == FD_HEIGHT,
               "FD geometry must match the camera frame");
_Static_assert(READY_QUEUE_CAPACITY + FRAME_JOB_COUNT == CAMERA_FRAME_BUFFERS - 3U,
               "Queued frames must leave the driver its active and spare buffers");
_Static_assert(INFER_CLASS_COUNT == CAM_MODEL_CLASS_COUNT, "Record and model class counts differ");
_Static_assert((CAM_MODEL_ROI_BYTES % 512U) == 0U, "ROI bytes must stay on SD sector boundaries");

/* ============================================================================
 * 感測器專屬暫存器定義與除頻計算 (Sensor Register Maps)
 * ============================================================================ */

/* --- OV7725 暫存器與位元遮罩 --- */
#define OV7725_REG_COM3             0x000CU                 /* 控制暫存器 3：設定 YUV 輸出順序 */
#define OV7725_REG_COM4             0x000DU                 /* 控制暫存器 4：PLL 鎖相環控制 */
#define OV7725_REG_COM5             0x000EU                 /* 控制暫存器 5：自動影格率調節控制 */
#define OV7725_REG_CLKRC            0x0011U                 /* 內部時脈控制暫存器 (除頻器) */
#define OMNIVISION_REG_COM8         0x0013U                 /* 自動曝光/增益/白平衡控制暫存器 */
#define OV7725_REG_ADVFL            0x002DU                 /* 垂直同步插入虛擬行低位元組 */
#define OV7725_REG_ADVFH            0x002EU                 /* 垂直同步插入虛擬行高位元組 */
#define OV7725_REG_DM_LNL           0x0033U                 /* 虛擬列低位元組 */
#define OV7725_REG_DM_LNH           0x0034U                 /* 虛擬列高位元組 */
#define OV7725_REG_DSP_CTRL3        0x0066U                 /* DSP 控制暫存器 3 */
#define OV7725_COM3_SWAP_YUV        0x10U                   /* COM3 第 4 位元：反轉 YUV 輸出順序 (確保第 0 位元組為 Y 亮度訊號) */
#define OV7725_COM4_PLL_MASK        0xC0U                   /* COM4 PLL 遮罩 */
#define OV7725_COM5_AUTO_FPS        0x80U                   /* COM5 第 7 位元：低光源自動降頻開關 (必須關閉以維持固定 5 FPS) */
#define OV7725_CLKRC_DIV_MASK       0x3FU                   /* CLKRC 除頻遮罩 */
#define OV7725_CLKRC_DIVIDER        3U                      /* 除頻值：設定除以 4，將原始時脈減半 */
#define OMNIVISION_COM8_AUTO_MASK   0x07U                   /* COM8 自動功能遮罩 (AGC, AEC, AWB) */
#define OMNIVISION_COM8_BANDING_MASK 0x30U                  /* COM8 bit5 banding filter、bit4 AEC 低於 banding 值；OV7670 驅動兩者皆關 */

/* --- OV2640 暫存器與位元遮罩 (第 8 位元編碼 Bank 0 或 Bank 1) --- */
#define OV2640_DSP_R_BYPASS         0x0005U                 /* DSP 旁路控制暫存器 */
#define OV2640_DSP_ZMOW             0x005AU                 /* 縮放輸出寬度低位元組 */
#define OV2640_DSP_ZMOH             0x005BU                 /* 縮放輸出高度低位元組 */
#define OV2640_DSP_ZMHH             0x005CU                 /* 縮放輸出高階位元組 */
#define OV2640_SENSOR_COM1          0x0103U                 /* bit7:6 dummy frame 控制 (自動降頻插幀) */
#define OV2640_SENSOR_CLKRC         0x0111U                 /* 感測器時脈控制暫存器 */
#define OV2640_SENSOR_COM8          0x0113U                 /* 感測器自動曝光/增益控制 */
#define OV2640_SENSOR_REG2A         0x012AU                 /* 虛擬行擴展高位暫存器 */
#define OV2640_SENSOR_FRARL         0x012BU                 /* 虛擬行擴展低位暫存器 */
#define OV2640_SENSOR_ADDVSL        0x012DU                 /* VSYNC 偏移低位暫存器 */
#define OV2640_SENSOR_ADDVSH        0x012EU                 /* VSYNC 偏移高位暫存器 */
#define OV2640_SENSOR_FLL           0x0146U                 /* 影格長度低位元組 */
#define OV2640_SENSOR_FLH           0x0147U                 /* 影格長度高位元組 */
#define OV2640_DSP_CTRL0            0x00C2U                 /* DSP 控制暫存器 0 */
#define OV2640_DSP_CTRL1            0x00C3U                 /* DSP 控制暫存器 1 */
#define OV2640_DSP_R_DVP_SP         0x00D3U                 /* DVP PCLK 輸出速度除頻控制 */
#define OV2640_DSP_IMAGE_MODE       0x00DAU                 /* 影像輸出格式暫存器 */
#define OV2640_R_BYPASS_DSP_MASK    0x01U
#define OV2640_CTRL0_YUV422_MASK    0x0CU
#define OV2640_IMAGE_MODE_FMT_MASK  0x5CU
#define OV2640_VGA_ZMOW_VALUE       0xA0U                   /* VGA 寬度縮放暫存器值 */
#define OV2640_VGA_ZMOH_VALUE       0x78U                   /* VGA 高度縮放暫存器值 */
#define OV2640_VGA_ZMHH_VALUE       0x00U
#define OV2640_COM8_AEC_AGC_MASK    0x05U
#define OV2640_COM8_BANDING_MASK    0x20U                   /* COM8 bit5 banding filter */
#define OV2640_COM1_DUMMY_FRAME_MASK 0xC0U
#define OV2640_CTRL1_AWB_MASK       0x08U
#define OV2640_CLKRC_CLOCK_MASK     0xBFU

#if USE_OV2640_ONBOARD_XVCLK
#define OV2640_XVCLK_HZ              OV2640_ONBOARD_XVCLK_HZ
#define OV2640_CLK_MULTIPLIER        2ULL
#define OV2640_CLKRC_DOUBLER         0x80U
#define OV2640_CLKRC_DIVIDER         5U
#else
#define OV2640_XVCLK_HZ              ESP32_CAMERA_XCLK_HZ   /* 16 MHz */
#define OV2640_CLK_MULTIPLIER        1ULL
#define OV2640_CLKRC_DOUBLER         0x00U
#define OV2640_CLKRC_DIVIDER         3U
#endif

#define OV2640_CLKRC_VALUE          (OV2640_CLKRC_DOUBLER | OV2640_CLKRC_DIVIDER)
#define OV2640_REG2A_FRARH_MASK     0xF0U
#define OV2640_DVP_SP_AUTO_MODE     0x80U
#define OV2640_DVP_PCLK_DIVIDER     6U
#define OV2640_DVP_SP_VALUE         (OV2640_DVP_SP_AUTO_MODE | OV2640_DVP_PCLK_DIVIDER)

/* --- OV7670 暫存器與位元遮罩 --- */
#define OV7670_REG_COM3             0x000CU
#define OV7670_REG_COM4             0x000DU
#define OV7670_REG_CLKRC            0x0011U
#define OV7670_REG_ADVFL            0x002DU
#define OV7670_REG_ADVFH            0x002EU
#define OV7670_REG_TSLB             0x003AU
#define OV7670_REG_COM11            0x003BU
#define OV7670_REG_DBLV             0x006BU                 /* PLL 倍頻暫存器 */
#define OV7670_REG_DM_LNL           0x0092U
#define OV7670_REG_DM_LNH           0x0093U
#define OV7670_CLKRC_CLOCK_MASK     0x7FU
#define OV7670_CLKRC_DIVIDER        7U                      /* 除以 8，調降影格率至 5 FPS */
#define OV7670_DBLV_PLL_MASK        0xC0U
#define OV7670_DBLV_PLL_4X          0x40U                   /* 啟用 PLL 4 倍頻 */
#define OV7670_COM11_NIGHT_MODE     0x80U                   /* 夜間自動降頻遮罩 (需關閉) */

/* --- 感測器硬體時脈與 FPS 數學理論模型計算 --- */
#define OV7725_PLL_MULTIPLIER       1U
#define OV7725_INTERNAL_CLOCK_HZ \
    (ESP32_CAMERA_XCLK_HZ * OV7725_PLL_MULTIPLIER / \
     ((OV7725_CLKRC_DIVIDER + 1U) * 2U))

#define OV7725_VGA_LINES_PER_FRAME       510U               /* OV7725 VGA 總行數 (含消隱區) */
#define OV7725_VGA_TP_PER_LINE           784U               /* 每行傳輸週期 */
#define OV7725_YUV_PCLK_PER_TP           2U
#define OV7725_PCLK_PER_FRAME \
    (OV7725_VGA_LINES_PER_FRAME * OV7725_VGA_TP_PER_LINE)
#define OV7725_CALCULATED_FPS \
    (OV7725_INTERNAL_CLOCK_HZ / OV7725_PCLK_PER_FRAME)
#define OV7725_CALCULATED_MILLI_FPS \
    ((uint32_t)(((uint64_t)OV7725_INTERNAL_CLOCK_HZ * 1000ULL) / \
                OV7725_PCLK_PER_FRAME))

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

/*
 * --- 編譯時期靜態斷言驗證 (Static Assertions) ---
 * 目的：在編譯程式碼的第一時間，由編譯器直接檢查數學常數與硬體限制。
 * 若工程師修改了常數導致時脈不合或空間溢位，編譯將立即報錯中斷，防止燒錄出有潛在問題的韌體。
 */
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

_Static_assert(IMAGE_PIXELS == 307200U, "VGA GRAY8 must contain 307200 bytes");
_Static_assert(EXPECTED_FRAME_COUNT == 25U, "5 seconds at 5 fps must contain about 25 frames");
_Static_assert(OV7725_CALCULATED_FPS == TARGET_FPS, "OV7725 clock model must match TARGET_FPS");
_Static_assert((SD_DMA_BUFFER_BYTES % 512U) == 0U,
               "SD DMA staging buffer must contain complete SD sectors");
_Static_assert((IMAGE_PIXELS % 512U) == 0U, "Each GRAY8 frame must end on an SD sector boundary");

/* ============================================================================
 * 資料結構定義 (Data Structures)
 * ============================================================================ */

/**
 * @brief 開機相機硬體 DMA 接收測試統計結構體
 * 用於在正式錄影前，檢驗相機硬體是否能穩定、按時輸出 5 FPS，完全不掉幀。
 */
typedef struct {
    uint32_t get_calls;                 /* 呼叫 esp_camera_fb_get() 的總次數 */
    uint32_t complete_frames;           /* 成功完整收到的合法影格總數 */
    uint32_t invalid_frames;            /* 格式或大小不合法的異常影格數 */
    uint32_t stale_frames;              /* 過期影格數 (時間戳早於觸發時間點) */
    uint32_t after_deadline_frames;     /* 逾期影格數 (時間戳超過 10 秒測試視窗) */
    uint32_t timeout_count;             /* 呼叫逾時未取得影格的次數 */
    uint32_t timestamp_errors;          /* 時間戳倒退或異常的錯誤計數 */
    uint32_t gap_events;                /* 影格間隔過長 (大於 300ms) 的斷層事件數 */
    uint32_t estimated_missing_frames;  /* 根據時間推估遺失的影格數量 */
    int64_t first_frame_us;             /* 第一幀到達的硬體時戳 (微秒) */
    int64_t last_frame_us;              /* 最後一幀到達的硬體時戳 (微秒) */
    int64_t min_interval_us;            /* 觀察到的兩幀間最小時間間隔 (微秒) */
    int64_t max_interval_us;            /* 觀察到的兩幀間最大時間間隔 (微秒) */
} dma_receive_test_stats_t;

/**
 * @brief 零拷貝 queue 元素：camera_capture 交給 cam_detect，只傳指標與時間
 */
typedef struct {
    camera_fb_t *frame;
    uint32_t frame_id;                  /* 啟動以來的影格序號 */
    int64_t timestamp_us;               /* 幀開始 VSYNC (esp_timer) */
    int64_t enqueue_us;
} ready_frame_t;

typedef struct {
    int32_t track_id;
    int32_t bbox[4];
    uint32_t crop_us;
    uint32_t invoke_us;         /* cam_model_predict()：複製輸入 + Invoke + 反量化 */
    uint32_t invoke_only_us;    /* 其中 Invoke() 本身 (只印 log) */
    uint32_t run_us;            /* 同一段期間 cam_infer 自己佔用 CPU 的時間 (含等 cache miss 的停頓) */
    uint32_t fd_overlap_us;     /* 同一段期間 Core 1 的 cam_detect 在處理幀的時間 */
    uint8_t pred;
    float conf;
    float probs[CAM_MODEL_CLASS_COUNT];
    bool low_conf;
} roi_result_t;

/* 一段時間內某個階段的平均與最大值 */
typedef struct {
    uint64_t total_us;
    uint32_t max_us;
} stage_stat_t;

/*
 * 一個回報週期內每次推論的 (FD 重疊量 x, 推論時間 y) 累加值，用來算相關係數與斜率。
 * 單位 ms；y 分兩組：wall = 實際經過時間，run = cam_infer 自己的 CPU 時間 (扣掉被搶占)。
 */
typedef struct {
    uint32_t n;
    double sum_x;
    double sum_xx;
    double sum_wall;
    double sum_wall2;
    double sum_x_wall;
    double sum_run;
    double sum_run2;
    double sum_x_run;
} overlap_stats_t;

typedef enum {
    STAGE_CAPTURE,
    STAGE_QUEUE,
    STAGE_DOWNSCALE,
    STAGE_FD,
    STAGE_FD_DIFF,
    STAGE_FD_MORPH,
    STAGE_FD_CONTOUR,
    STAGE_FD_REFINE,
    STAGE_FD_NMS,
    STAGE_TRACK,
    STAGE_DETECT,
    STAGE_CNN_WAIT,
    STAGE_CROP,
    STAGE_CNN,
    STAGE_COUNT,
} stage_id_t;

/* 每個 task 各有一張表，只有自己讀寫，兩顆核心之間不必上鎖 */
typedef struct {
    stage_stat_t stats[STAGE_COUNT];
    uint32_t frames;
} stage_table_t;

/**
 * @brief 一幀從 cam_detect → cam_infer → (低信賴才) cam_writer 的工作 (PSRAM)。
 * fb 所有權跟著工作走，最後一棒才歸還；低信賴時要存整幀，所以 CNN 期間不能先還。
 */
typedef struct {
    camera_fb_t *frame;
    int64_t handed_over_us;             /* 交給下一棒的時間 */
    infer_cam_frame_record_t timing;    /* cam_detect 填到 detect_us，cam_infer 填完後回報 */
    size_t candidate_count;
    Stage2_CnnCandidate candidates[INFER_CAM_MAX_ROI_PER_FRAME];
    size_t roi_count;
    bool any_low;                       /* 有低信賴 ROI */
    bool any_non_background;            /* 有非背景 ROI (INFER_SAVE_NON_BACKGROUND) */
    roi_result_t rois[INFER_CAM_MAX_ROI_PER_FRAME];
    uint8_t roi_pixels[INFER_CAM_MAX_ROI_PER_FRAME][CAM_MODEL_ROI_BYTES];
    char meta[SAMPLE_META_BYTES];
} frame_job_t;

/** @brief 連續運行的累計統計；各欄位只有一個 task 寫入 */
typedef struct {
    int64_t session_epoch;              /* 啟動時間，當作這次執行的 ID */
    int64_t trigger_us;
    /* camera_capture */
    uint32_t frames;
    uint32_t stale;
    uint32_t invalid;
    uint32_t timestamp_errors;
    uint32_t timeouts;
    uint32_t dropped;
    uint32_t gaps;
    int64_t first_frame_us;
    int64_t last_frame_us;
    int64_t interval_min_us;
    int64_t interval_max_us;
    /* cam_detect */
    uint32_t processed;
    /* cam_infer */
    uint32_t rois;
    uint32_t low_conf;
    /* cam_writer */
    uint32_t saved;
    uint32_t save_failed;
    stage_stat_t save_wait;
    stage_stat_t save;
} session_state_t;


/* ============================================================================
 * 全域與靜態變數 (Global Variables)
 * ============================================================================ */

static const char *TAG = "cam_infer";
static uint8_t *s_sd_dma_buffer = NULL;      /* 共用 I/O 緩衝 (32KB 內部 DMA RAM)；app_main 開機時分配，常駐 */
static const size_t s_sd_dma_buffer_bytes = SD_DMA_BUFFER_BYTES;
static bool s_initialized = false;
static bool s_camera_paused = false;
static bool s_xclk_stopped = false;          /* XCLK (LEDC) 是否已在暫停期間停止輸出 */
static int64_t s_xclk_started_us = 0;        /* 最近一次恢復 XCLK 的時間，用來等待感測器穩定 */

static StaticQueue_t s_ready_queue_control;
static uint8_t s_ready_queue_storage[READY_QUEUE_CAPACITY * sizeof(ready_frame_t)];
static QueueHandle_t s_ready_queue = NULL;

/* 工作在 PSRAM，佇列只傳指標；三個佇列深度都等於工作數，一定放得下 */
static StaticQueue_t s_free_jobs_control;
static StaticQueue_t s_cnn_queue_control;
static StaticQueue_t s_save_queue_control;
static uint8_t s_free_jobs_storage[FRAME_JOB_COUNT * sizeof(frame_job_t *)];
static uint8_t s_cnn_queue_storage[FRAME_JOB_COUNT * sizeof(frame_job_t *)];
static uint8_t s_save_queue_storage[FRAME_JOB_COUNT * sizeof(frame_job_t *)];
static QueueHandle_t s_free_jobs = NULL;
static QueueHandle_t s_cnn_queue = NULL;
static QueueHandle_t s_save_queue = NULL;

static session_state_t s_session;
static cam_infer_stats_t s_stats;

/* 每次回報後歸零 */
static stage_table_t s_detect_stages;       /* 只有 cam_detect 存取 */
static stage_table_t s_cnn_stages;          /* 只有 cam_infer 存取 */
static const char *const STAGE_NAMES[STAGE_COUNT] = {
    [STAGE_CAPTURE] = "capture",
    [STAGE_QUEUE] = "queue",
    [STAGE_DOWNSCALE] = "downscale",
    [STAGE_FD] = "fd",
    [STAGE_FD_DIFF] = "diff",
    [STAGE_FD_MORPH] = "morph",
    [STAGE_FD_CONTOUR] = "contour",
    [STAGE_FD_REFINE] = "refine",
    [STAGE_FD_NMS] = "nms",
    [STAGE_TRACK] = "track",
    [STAGE_DETECT] = "detect",
    [STAGE_CNN_WAIT] = "cnn_wait",
    [STAGE_CROP] = "crop",
    [STAGE_CNN] = "cnn",
};

/* cam_detect 的工作緩衝 (PSRAM)：全部只有 cam_detect 存取 */
static uint8_t *s_ds_prev = NULL;
static uint8_t *s_ds_curr = NULL;
static uint8_t *s_fd_temp = NULL;
static FD_BoundingBox *s_fullres_boxes = NULL;
static Stage2_Tracker *s_tracker = NULL;
static FD_Parameters s_fd_params;

static StaticTask_t s_detect_tcb;
static StackType_t s_detect_stack[DETECT_TASK_STACK_BYTES];
static TaskHandle_t s_detect_task = NULL;
static StaticTask_t s_infer_tcb;
static StackType_t s_infer_stack[INFER_TASK_STACK_BYTES];
static TaskHandle_t s_infer_task = NULL;
static uint32_t s_incomplete_at_start = 0;
/* 模型類別表中 background 的索引；-1 = 不以非背景為存檔條件 (見 INFER_SAVE_NON_BACKGROUND) */
static int s_background_class = -1;

/*
 * 推論期間 Core 1 的 FD 忙碌量，用來驗證 CNN 時間的波動是不是 PSRAM 匯流排競爭。
 * 不讀 IDLE1 的 ulRunTimeCounter：另一顆核心上正在跑的 task，計數器要等它被切換才更新，
 * 單次推論的誤差可達數十 ms。改由 cam_detect 用 esp_timer 記錄 detect_frame() 的起訖，
 * cam_infer 在 Core 0 讀累計值 (跨核心，用 spinlock)。
 * 不含驅動 cam_task 搬像素的時間 (每幀固定，大致和推論長度成正比)。
 */
static portMUX_TYPE s_fd_busy_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_fd_busy_since_us = 0;      /* 0 = cam_detect 目前沒在處理幀 */
static uint64_t s_fd_busy_total_us = 0;
static overlap_stats_t s_overlap;           /* 只有 cam_infer 存取 */

static esp_err_t cam_infer_resume(void);

/**
 * @brief 官方相機驅動組態設定 (esp_camera configuration)
 */
static camera_config_t s_camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,               /* 電源關閉控制腳位 */
    .pin_reset = CAM_PIN_RESET,             /* 硬體重置腳位 */
    .pin_xclk = CAMERA_CONFIG_XCLK_PIN,     /* 主時脈 XCLK 腳位 */
    .pin_sccb_sda = CAM_PIN_SIOD,           /* SCCB (I2C) 資料線腳位 (用於控制感測器暫存器) */
    .pin_sccb_scl = CAM_PIN_SIOC,           /* SCCB (I2C) 時脈線腳位 */
    .pin_d7 = CAM_PIN_D7,                   /* 平行資料匯流排 D0 - D7 */
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,             /* 垂直同步訊號 (VSYNC，標誌一幀開始) */
    .pin_href = CAM_PIN_HREF,               /* 水平參考訊號 (HREF，標誌一行有效資料) */
    .pin_pclk = CAM_PIN_PCLK,               /* 像素時脈訊號 (PCLK，每個邊緣採樣一個位元組) */
    .xclk_freq_hz = CAMERA_CONFIG_XCLK_HZ,  /* 輸入感測器的時脈頻率 */
    .ledc_timer = LEDC_TIMER_0,             /* 用於產生 XCLK 的 LEDC 定時器編號 */
    .ledc_channel = LEDC_CHANNEL_0,         /* 用於產生 XCLK 的 LEDC 通道編號 */

    /*
     * 影像輸出格式設定：
     * 感測器硬體輸出 YUV422 格式，ESP32 的 I2S0 平行輸入 DMA 先收進內部 RAM，
     * 再由驅動的 cam_task (Core 1) 取出每個像素第 0 位元組的 Y，
     * 存入 PSRAM 緩衝區，得到 GRAY8 灰階影像 (因此 Y 必須排在第一個位元組)。
     */
    .pixel_format = PIXFORMAT_GRAYSCALE,    /* 灰階模式 (GRAY8) */
    .frame_size = FRAMESIZE_VGA,            /* 640x480 解析度 */
    .jpeg_quality = 12,                     /* 非 JPEG 模式下此參數不作用 */
    
    /*
     * 緩衝區數量與位置：
     * 配置 8 個緩衝區於外部 PSRAM 中。
     * cam_writer 在完成 SD 寫入前會一直持有該緩衝區指標，
     * 寫入完畢後呼叫 return，使硬體能循環重用這 8 個緩衝區。
     */
    .fb_count = CAMERA_FRAME_BUFFERS,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,        /* 閒置等待觸發時，只保留最新完成的一幀，其餘自動丟棄 */
};

/* ============================================================================
 * XCLK 開關 (同 main/cam_record.c)：開機測試後到 cam_infer_start() 之前停止 16 MHz 時脈，
 * 減少開機期間對 Wi-Fi 的干擾。連續推論開始後 XCLK 一直開著。
 * ============================================================================ */

/* esp32-camera 的 xclk.c 固定使用 LEDC_LOW_SPEED_MODE。
 * 停止期間不可存取 SCCB：OV7670 的 SCCB 需要 XCLK 才能運作。 */
static esp_err_t xclk_stop(void)
{
    if (s_camera_config.pin_xclk < 0 || s_xclk_stopped) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ledc_stop(LEDC_LOW_SPEED_MODE, s_camera_config.ledc_channel, 0),
                        TAG, "XCLK output stop failed");
    ESP_RETURN_ON_ERROR(ledc_timer_pause(LEDC_LOW_SPEED_MODE, s_camera_config.ledc_timer),
                        TAG, "XCLK timer pause failed");
    s_xclk_stopped = true;
    return ESP_OK;
}

static esp_err_t xclk_start(void)
{
    if (s_camera_config.pin_xclk < 0 || !s_xclk_stopped) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ledc_timer_resume(LEDC_LOW_SPEED_MODE, s_camera_config.ledc_timer),
                        TAG, "XCLK timer resume failed");
    /* ledc_update_duty 會重新開啟 ledc_stop 關掉的通道輸出 */
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, s_camera_config.ledc_channel),
                        TAG, "XCLK output restart failed");
    s_xclk_stopped = false;
    s_xclk_started_us = esp_timer_get_time();
    return ESP_OK;
}

/* 只等待尚未經過的部分 */
static void xclk_wait_settled(void)
{
    const int64_t settled_us = s_xclk_started_us + (int64_t)CAMERA_XCLK_SETTLE_MS * 1000LL;
    const int64_t now_us = esp_timer_get_time();
    if (now_us < settled_us) {
        const uint32_t remaining_ms = (uint32_t)((settled_us - now_us + 999LL) / 1000LL);
        vTaskDelay(pdMS_TO_TICKS(remaining_ms) + 1U);
    }
}

/* ============================================================================
 * 感測器底層暫存器調校函式 (Sensor Register Configuration)
 * ============================================================================ */

/**
 * @brief 取得感測器型號名稱字串
 * @param pid 感測器產品識別碼 (Product ID)
 * @return 型號字串 ("OV2640", "OV7670", "OV7725" 或 "unsupported")
 */
static const char *sensor_name(uint16_t pid)
{
    switch (pid) {
    case OV2640_PID: return "OV2640";
    case OV7670_PID: return "OV7670";
    case OV7725_PID: return "OV7725";
    default: return "unsupported";
    }
}

/**
 * @brief 設定 OV7725 感測器：鎖定固定 5 FPS、確保灰階位元組順序，
 *        banding filter / 虛擬行比照 OV7670 關閉
 */
static esp_err_t configure_ov7725(sensor_t *sensor)
{
    uint8_t com3 = 0;
    uint8_t dsp_ctrl3 = 0;
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_COM3, 0xFF, &com3),
                        TAG, "Cannot read OV7725 COM3");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_DSP_CTRL3,
                                         0xFF, &dsp_ctrl3),
                        TAG, "Cannot read OV7725 DSP_CTRL3");

    /* 驗證 COM3[4] 是否為 1：在目前腳位走線下，此位元必須為 1 才能保證 Y 訊號排在第一個位元組 */
    if ((com3 & OV7725_COM3_SWAP_YUV) == 0) {
        ESP_LOGE(TAG,
                 "OV7725 COM3=0x%02X: COM3[4] must remain 1 for GRAY8 capture",
                 com3);
        return ESP_ERR_INVALID_STATE;
    }

    /* 透過 CLKRC 暫存器調降感測器時脈，並關閉低光自動降頻，鎖定在 5 FPS */
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

    /* 與 OV7670 一致：驅動預設 COM8=0xFF 開著 banding filter，強光下曝光最低只能到
     * 一個 banding 單位而過曝；並清除虛擬行，避免影格被拉長 */
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OMNIVISION_REG_COM8,
                                         OMNIVISION_COM8_BANDING_MASK, 0x00),
                        TAG, "Cannot disable OV7725 banding filter");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7725_REG_ADVFL, 0xFF, 0x00),
                        TAG, "Cannot clear OV7725 ADVFL");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7725_REG_ADVFH, 0xFF, 0x00),
                        TAG, "Cannot clear OV7725 ADVFH");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7725_REG_DM_LNL, 0xFF, 0x00),
                        TAG, "Cannot clear OV7725 DM_LNL");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7725_REG_DM_LNH, 0xFF, 0x00),
                        TAG, "Cannot clear OV7725 DM_LNH");

    /* 回讀暫存器，確保寫入生效 */
    uint8_t com4 = 0;
    uint8_t com5 = 0;
    uint8_t clkrc = 0;
    uint8_t com8 = 0, advfl = 0, advfh = 0, dm_lnl = 0, dm_lnh = 0;
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_COM4, 0xFF, &com4),
                        TAG, "Cannot verify OV7725 COM4");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_COM5, 0xFF, &com5),
                        TAG, "Cannot verify OV7725 COM5");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_CLKRC, 0xFF, &clkrc),
                        TAG, "Cannot verify OV7725 CLKRC");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OMNIVISION_REG_COM8, 0xFF, &com8),
                        TAG, "Cannot verify OV7725 COM8");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_ADVFL, 0xFF, &advfl),
                        TAG, "Cannot verify OV7725 ADVFL");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_ADVFH, 0xFF, &advfh),
                        TAG, "Cannot verify OV7725 ADVFH");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_DM_LNL, 0xFF, &dm_lnl),
                        TAG, "Cannot verify OV7725 DM_LNL");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV7725_REG_DM_LNH, 0xFF, &dm_lnh),
                        TAG, "Cannot verify OV7725 DM_LNH");

    if ((com4 & OV7725_COM4_PLL_MASK) != 0 ||
        (clkrc & OV7725_CLKRC_DIV_MASK) != OV7725_CLKRC_DIVIDER ||
        (com5 & OV7725_COM5_AUTO_FPS) != 0 ||
        (com8 & OMNIVISION_COM8_BANDING_MASK) != 0 ||
        advfl != 0 || advfh != 0 || dm_lnl != 0 || dm_lnh != 0) {
        ESP_LOGE(TAG, "OV7725 clock verification failed: COM4=0x%02X "
                 "CLKRC=0x%02X COM5=0x%02X COM8=0x%02X ADVF=%02X%02X "
                 "DM_LN=%02X%02X",
                 com4, clkrc, com5, com8, advfh, advfl, dm_lnh, dm_lnl);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG,
             "OV7725 GRAY8: COM3=0x%02X DSP_CTRL3=0x%02X COM8=0x%02X; "
             "XCLK=%u Hz, calculated=%u.%03u fps",
             com3, dsp_ctrl3, com8, (unsigned)ESP32_CAMERA_XCLK_HZ,
             (unsigned)(OV7725_CALCULATED_MILLI_FPS / 1000U),
             (unsigned)(OV7725_CALCULATED_MILLI_FPS % 1000U));
    return ESP_OK;
}

/**
 * @brief 設定 OV7670 感測器：鎖定固定 5 FPS、停用夜間降頻並清除虛擬行擴展
 */
static esp_err_t configure_ov7670(sensor_t *sensor)
{
    /* 設定 PLL 4倍頻並透過 CLKRC 除頻，使 VSYNC 硬體頻率精準落在 5 FPS */
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7670_REG_DBLV,
                                         OV7670_DBLV_PLL_MASK,
                                         OV7670_DBLV_PLL_4X),
                        TAG, "Cannot set OV7670 PLL x4");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV7670_REG_CLKRC,
                                         OV7670_CLKRC_CLOCK_MASK,
                                         OV7670_CLKRC_DIVIDER),
                        TAG, "Cannot set OV7670 CLKRC divider");

    /* 停用夜間模式與清除行擴展，防止低光下自動降速 */
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

    /* 驗證暫存器設定 */
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

    ESP_LOGI(TAG,
             "OV7670 GRAY8: TSLB=0x%02X COM3=0x%02X COM4=0x%02X "
             "CLKRC=0x%02X DBLV=0x%02X; hardware=%u.%03u fps",
             tslb, com3, com4, clkrc, dblv,
             (unsigned)(OV7670_CALCULATED_MILLI_FPS / 1000U),
             (unsigned)(OV7670_CALCULATED_MILLI_FPS % 1000U));
    return ESP_OK;
}

/**
 * @brief 設定 OV2640 感測器：設定 DSP 縮放、DVP 輸出除頻與 5 FPS 時脈樹，
 *        banding filter / dummy frame 比照 OV7670 關閉
 */
static esp_err_t configure_ov2640(sensor_t *sensor)
{
    /* 設定 DVP 輸出時脈與感測器核心時脈 */
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_DSP_R_DVP_SP,
                                         0xFF, OV2640_DVP_SP_VALUE),
                        TAG, "Cannot set OV2640 DVP PCLK divider");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_SENSOR_CLKRC,
                                         OV2640_CLKRC_CLOCK_MASK,
                                         OV2640_CLKRC_VALUE),
                        TAG, "Cannot set OV2640 CLKRC doubler/divider");

    /* 啟用 YUV422 格式管線與 DSP 縮放至 VGA 尺寸 */
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

    /* 清除歷史殘留的虛擬行與曝光補償暫存器 */
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

    /* 與 OV7670 一致：關閉 banding filter（SVGA 表開著，強光下曝光最低卡在約 50 ms），
     * 並禁止自動插入 dummy frame（等同 OV7670 COM11 夜間降頻） */
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_SENSOR_COM8,
                                         OV2640_COM8_BANDING_MASK, 0x00),
                        TAG, "Cannot disable OV2640 banding filter");
    ESP_RETURN_ON_ERROR(camera_reg_write(sensor, OV2640_SENSOR_COM1,
                                         OV2640_COM1_DUMMY_FRAME_MASK, 0x00),
                        TAG, "Cannot disable OV2640 dummy frames");

    /* 回讀暫存器並完整驗證 */
    uint8_t clkrc = 0, dvp_sp = 0, reg2a = 0, frarl = 0;
    uint8_t addvsl = 0, addvsh = 0, fll = 0, flh = 0;
    uint8_t com8 = 0, com1 = 0;
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
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_SENSOR_COM8,
                                         0xFF, &com8),
                        TAG, "Cannot verify OV2640 COM8");
    ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_SENSOR_COM1,
                                         0xFF, &com1),
                        TAG, "Cannot verify OV2640 COM1");
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
        addvsl != 0 || addvsh != 0 || fll != 0 || flh != 0 ||
        (com8 & OV2640_COM8_BANDING_MASK) != 0 ||
        (com1 & OV2640_COM1_DUMMY_FRAME_MASK) != 0) {
        ESP_LOGE(TAG, "OV2640 clock verification failed: CLKRC=0x%02X "
                 "R_DVP_SP=0x%02X "
                 "FRAR=%01X%02X ADDVS=%02X%02X FL=%02X%02X COM8=0x%02X COM1=0x%02X",
                 clkrc, dvp_sp, (reg2a >> 4), frarl,
                 addvsh, addvsl, flh, fll, com8, com1);
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "OV2640 banding filter off, dummy frames off: COM8=0x%02X COM1=0x%02X",
             com8, com1);

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

/**
 * @brief 統一感測器設定入口：依據讀取到的硬體 PID 轉派對應的設定常式
 */
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
        ssize_t written = write(fd, data + offset, bytes - offset);
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

/**
 * @brief 分段將影像 (raw 300KB 或 JPEG) 從外部 PSRAM 複製到內部 DMA 暫存區，再寫入 SD 卡
 */
static esp_err_t write_image_to_sd(int fd, const uint8_t *pixels, size_t bytes, int64_t *max_chunk_us)
{
    size_t offset = 0;
    while (offset < bytes) {
        const size_t remaining = bytes - offset;
        const size_t chunk = remaining < s_sd_dma_buffer_bytes
                                 ? remaining
                                 : s_sd_dma_buffer_bytes;

        /* PSRAM -> 內部 SRAM 記憶體複製 */
        memcpy(s_sd_dma_buffer, pixels + offset, chunk);

        /* 內部 SRAM -> SD 卡寫入 (透過 SDMMC DMA) */
        const int64_t start_us = esp_timer_get_time();
        const esp_err_t err = write_all(fd, s_sd_dma_buffer, chunk);
        const int64_t chunk_us = esp_timer_get_time() - start_us;
        if (chunk_us > *max_chunk_us) {
            *max_chunk_us = chunk_us;                       /* 紀錄最長分段寫入耗時 */
        }
        if (err != ESP_OK) {
            return err;
        }
        offset += chunk;
    }
    return ESP_OK;
}

/* ============================================================================
 * 相機控制與前置驗證 (Camera Control and Qualification)
 * ============================================================================ */

/**
 * @brief 初始化相機驅動、檢查 PSRAM 並調校底層感測器
 */
static esp_err_t init_camera(sensor_t **sensor_out)
{
    const size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram_total == 0U) {
        ESP_LOGE(TAG, "PSRAM is required for the VGA camera framebuffers");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "PSRAM: %u bytes total, %u bytes free, largest block=%u",
             (unsigned)psram_total,
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

/**
 * @brief 啟用或鎖定相機自動功能 (自動增益 AGC、自動曝光 AEC、自動白平衡 AWB)
 * 說明：錄影前需先鎖定曝光與白平衡，避免每幀畫面明暗色調跳動，造成影像閃爍。
 */
static esp_err_t set_auto_controls(sensor_t *sensor, bool enable)
{
    if (sensor->set_gain_ctrl == NULL || sensor->set_exposure_ctrl == NULL ||
        sensor->set_whitebal == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (sensor->set_gain_ctrl(sensor, enable) < 0 ||
        sensor->set_exposure_ctrl(sensor, enable) < 0 ||
        sensor->set_whitebal(sensor, enable) < 0) {
        return ESP_FAIL;
    }

    if (sensor->id.PID == OV2640_PID) {
        uint8_t com8 = 0, ctrl1 = 0;
        ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_SENSOR_COM8, 0xFF, &com8),
                            TAG, "Cannot verify OV2640 COM8");
        ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OV2640_DSP_CTRL1, 0xFF, &ctrl1),
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
        ESP_RETURN_ON_ERROR(camera_reg_read(sensor, OMNIVISION_REG_COM8, 0xFF, &com8),
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

/**
 * @brief 驗證單一影格緩衝區是否完全符合預期的解析度與長度
 */
static bool frame_is_valid(const camera_fb_t *frame)
{
    return frame != NULL && frame->format == PIXFORMAT_GRAYSCALE &&
           frame->width == IMAGE_WIDTH && frame->height == IMAGE_HEIGHT &&
           frame->len == IMAGE_PIXELS;
}

/**
 * @brief 計算影格的微秒 (us) 系統時戳
 */
static int64_t frame_timestamp_us(const camera_fb_t *frame)
{
    return (int64_t)frame->timestamp.tv_sec * 1000000LL +
           (int64_t)frame->timestamp.tv_usec;
}

/**
 * @brief 開機暖機常式：接收並丟棄前 10 幀，讓感測器自動曝光收斂穩定
 */
static esp_err_t discard_warmup_frames(sensor_t *sensor)
{
    ESP_LOGI(TAG,
             "Warm-up: discard %u complete frames with AE/AGC/AWB enabled",
             (unsigned)CAMERA_WARMUP_FRAMES);

    int64_t first_us = 0;
    int64_t last_us = 0;

    for (uint32_t complete = 0; complete < CAMERA_WARMUP_FRAMES; ++complete) {
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
        esp_camera_fb_return(frame);    /* 丟棄並歸還緩衝區 */
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

    ESP_LOGI(TAG,
             "%s warm-up delivery: %u intervals in %u ms, measured=%u.%03u fps",
             sensor_name(sensor->id.PID), (unsigned)intervals,
             (unsigned)(elapsed_us / 1000ULL),
             (unsigned)(milli_fps / 1000U),
             (unsigned)(milli_fps % 1000U));
    return ESP_OK;
}

/**
 * @brief 開機相機效能資格驗證 (Startup FPS Test)
 * 說明：
 * 在不掛載 SD 卡的情況下，純粹接收 10 秒影格並檢驗時戳間隔。
 * 必須通過 5 FPS 且完全零掉幀的嚴格檢驗，系統才允許掛載 SD 卡進入錄影作業。
 */
static esp_err_t run_startup_fps_test(void)
{
    dma_receive_test_stats_t stats = {0};
    const int64_t trigger_us = esp_timer_get_time();
    const int64_t deadline_us = trigger_us + (int64_t)TEST_DURATION_SECONDS * 1000000LL;
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

        /* 時戳合法性檢查 */
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

        /* 影格格式與大小檢查 */
        if (!frame_is_valid(frame)) {
            stats.invalid_frames++;
            ESP_LOGE(TAG,
                     "Invalid returned frame: format=%d width=%u height=%u len=%u",
                     (int)frame->format, (unsigned)frame->width,
                     (unsigned)frame->height, (unsigned)frame->len);
            esp_camera_fb_return(frame);
            continue;
        }

        /* 濾除觸發前已在硬體管線殘留的歷史影格 */
        if (timestamp_us < trigger_us) {
            stats.stale_frames++;
            esp_camera_fb_return(frame);
            continue;
        }

        /* 超出 10 秒視窗則測試結束 */
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
            if (stats.complete_frames == 1U || interval_us < stats.min_interval_us) {
                stats.min_interval_us = interval_us;
            }
            if (interval_us > stats.max_interval_us) {
                stats.max_interval_us = interval_us;
            }

            /* 檢查是否發生掉幀 (間隔大於 300ms) */
            if (interval_us > GAP_THRESHOLD_US) {
                stats.gap_events++;
                const uint32_t represented_periods =
                    (uint32_t)((interval_us + EXPECTED_FRAME_PERIOD_US / 2LL) /
                               EXPECTED_FRAME_PERIOD_US);
                if (represented_periods > 1U) {
                    stats.estimated_missing_frames += represented_periods - 1U;
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

        if (stats.complete_frames == 1U || (stats.complete_frames % 10U) == 0U) {
            ESP_LOGI(TAG,
                     "FPS test frame %u: timestamp_offset=%" PRId64
                     " us interval=%" PRId64 " us",
                     (unsigned)stats.complete_frames,
                     timestamp_us - trigger_us,
                     interval_us);
        }

        esp_camera_fb_return(frame);    /* 立即歸還，不進行任何 SD 寫入 */
    }

    /* 計算平均 FPS 與檢驗結果 */
    const int64_t span_us = stats.complete_frames > 1U
                                ? stats.last_frame_us - stats.first_frame_us
                                : 0;
    const uint32_t timestamp_milli_fps =
        stats.complete_frames > 1U && span_us > 0
            ? (uint32_t)(((uint64_t)(stats.complete_frames - 1U) * 1000000000ULL) /
                         (uint64_t)span_us)
            : 0;
    const uint32_t window_milli_fps =
        (uint32_t)(((uint64_t)stats.complete_frames * 1000ULL) / TEST_DURATION_SECONDS);
    const uint32_t deficit =
        stats.complete_frames < EXPECTED_FRAME_COUNT
            ? EXPECTED_FRAME_COUNT - stats.complete_frames
            : 0U;

    const uint32_t target_milli_fps = TARGET_FPS * 1000U;
    const bool fps_in_range =
        timestamp_milli_fps >= target_milli_fps - FPS_TOLERANCE_MILLI &&
        timestamp_milli_fps <= target_milli_fps + FPS_TOLERANCE_MILLI;
    
    /* 驗證通過準則：影格數達標、零斷層、零異常幀、零逾時、零時戳錯亂且 FPS 在誤差內 */
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

/* ============================================================================
 * 擷取 → 推論 → (低信賴才) 寫入
 * ============================================================================ */

/**
 * @brief camera_capture 任務：管理影格指標，不搬像素
 * 運行於：CPU Core 1，優先權 6；常駐。
 * 職責：
 *  1. esp_camera_fb_get() 等驅動交出一張完成的 fb。
 *  2. 檢查時戳 (遞增、不在未來) 與大小，不合格的直接歸還並計數。
 *  3. 啟動前留在驅動裡的舊幀丟棄。
 *  4. 合格的幀包成 ready_frame_t 非阻塞送進 s_ready_queue，fb 所有權交給 cam_detect；
 *     佇列滿 (FD 跟不上) 就丟這一幀並歸還。
 */
static void camera_capture(void *argument)
{
    (void)argument;
    session_state_t *session = &s_session;
    int64_t previous_seen_us = 0;

    for (;;) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == NULL) {
            /* 連續運行不能因為一次逾時就停：計數後繼續等 */
            session->timeouts++;
            ESP_LOGE(TAG, "Capture timed out after %u frames (timeouts=%u)",
                     (unsigned)session->frames, (unsigned)session->timeouts);
            continue;
        }

        const int64_t timestamp_us = frame_timestamp_us(frame);
        const int64_t received_us = esp_timer_get_time();
        if (timestamp_us <= 0 || timestamp_us > received_us ||
            (previous_seen_us != 0 && timestamp_us <= previous_seen_us)) {
            session->timestamp_errors++;
            esp_camera_fb_return(frame);
            continue;
        }
        previous_seen_us = timestamp_us;

        if (!frame_is_valid(frame)) {
            session->invalid++;
            esp_camera_fb_return(frame);
            continue;
        }

        /* 濾除觸發前的歷史殘留影格 */
        if (timestamp_us < session->trigger_us) {
            session->stale++;
            esp_camera_fb_return(frame);
            continue;
        }

        if (session->frames == 0U) {
            session->first_frame_us = timestamp_us;
        } else {
            const int64_t interval_us = timestamp_us - session->last_frame_us;
            if (session->frames == 1U || interval_us < session->interval_min_us) {
                session->interval_min_us = interval_us;
            }
            if (interval_us > session->interval_max_us) {
                session->interval_max_us = interval_us;
            }
            if (interval_us > GAP_THRESHOLD_US) {
                session->gaps++;
            }
        }
        session->last_frame_us = timestamp_us;

        const ready_frame_t ready = {
            .frame = frame,
            .frame_id = session->frames++,
            .timestamp_us = timestamp_us,
            .enqueue_us = esp_timer_get_time(),
        };
        if (xQueueSend(s_ready_queue, &ready, 0) != pdTRUE) {
            session->dropped++;
            esp_camera_fb_return(frame);
            if (session->dropped == 1U || (session->dropped % 10U) == 0U) {
                ESP_LOGW(TAG, "Ready queue full at frame %u: dropped=%u",
                         (unsigned)ready.frame_id, (unsigned)session->dropped);
            }
        }
    }
}

/* nearest 下採樣 (與原專案相同)，fb 全解析度不動 */
static void downscale_frame(const uint8_t *full, uint8_t *out)
{
    for (int y = 0; y < FD_PROC_HEIGHT; y++) {
        const uint8_t *row = full + (size_t)(y * FD_DOWNSCALE_FACTOR) * FD_WIDTH;
        for (int x = 0; x < FD_PROC_WIDTH; x++) {
            out[x] = row[x * FD_DOWNSCALE_FACTOR];
        }
        out += FD_PROC_WIDTH;
    }
}

/* FD 結果 (低解析度 bbox) 換回全解析度，回傳框數 */
static int run_frame_difference(FD_StageTiming *timing)
{
    int num_boxes = 0;
    FD_DetectionResult *result = fd_process_frames_gray(s_ds_prev, s_ds_curr, s_fd_temp,
                                                        &s_fd_params, &num_boxes,
                                                        FD_PROC_WIDTH, FD_PROC_HEIGHT, timing);
    int count = 0;
    if (result != NULL) {
        for (int i = 0; i < num_boxes && count < STAGE2_MAX_DETECTIONS; i++) {
            const FD_BoundingBox *low = &result->bboxes[i];
            int x1 = low->x1 * FD_DOWNSCALE_FACTOR;
            int y1 = low->y1 * FD_DOWNSCALE_FACTOR;
            int x2 = low->x2 * FD_DOWNSCALE_FACTOR + (FD_DOWNSCALE_FACTOR - 1);
            int y2 = low->y2 * FD_DOWNSCALE_FACTOR + (FD_DOWNSCALE_FACTOR - 1);
            if (x2 > FD_WIDTH - 1) {
                x2 = FD_WIDTH - 1;
            }
            if (y2 > FD_HEIGHT - 1) {
                y2 = FD_HEIGHT - 1;
            }
            if (x1 < 0 || y1 < 0 || x2 < x1 || y2 < y1) {
                continue;
            }
            FD_BoundingBox *out = &s_fullres_boxes[count++];
            out->x1 = x1;
            out->y1 = y1;
            out->x2 = x2;
            out->y2 = y2;
            out->area = (x2 - x1 + 1) * (y2 - y1 + 1);
        }
        fd_free_detection_result(result);
    }
    return count;
}

/* cam_detect 呼叫：開始 / 結束處理一幀 */
static void fd_busy_mark(bool busy)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_fd_busy_lock);
    if (busy) {
        s_fd_busy_since_us = now_us;
    } else if (s_fd_busy_since_us != 0) {
        s_fd_busy_total_us += (uint64_t)(now_us - s_fd_busy_since_us);
        s_fd_busy_since_us = 0;
    }
    portEXIT_CRITICAL(&s_fd_busy_lock);
}

/* cam_infer 呼叫：開機以來 cam_detect 處理幀的累計時間，含正在處理的這一幀 */
static uint64_t fd_busy_now_us(void)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_fd_busy_lock);
    uint64_t total_us = s_fd_busy_total_us;
    /* now_us 在鎖外讀，Core 1 可能在這之後才開始新的一幀 */
    if (s_fd_busy_since_us != 0 && now_us > s_fd_busy_since_us) {
        total_us += (uint64_t)(now_us - s_fd_busy_since_us);
    }
    portEXIT_CRITICAL(&s_fd_busy_lock);
    return total_us;
}

/*
 * 呼叫者自己的 ulRunTimeCounter (us)。計數器只在 vTaskSwitchContext 時結算，
 * 所以先 yield：就算沒有切到別的 task，到目前為止的時間也會記進去。
 */
static uint32_t self_run_time_us(void)
{
    taskYIELD();
    TaskStatus_t status;
    vTaskGetInfo(NULL, &status, pdFALSE, eRunning);
    return status.ulRunTimeCounter;
}

static void overlap_note(const roi_result_t *roi)
{
    const double x = (double)roi->fd_overlap_us / 1000.0;
    const double wall = (double)roi->invoke_us / 1000.0;
    const double run = (double)roi->run_us / 1000.0;
    s_overlap.n++;
    s_overlap.sum_x += x;
    s_overlap.sum_xx += x * x;
    s_overlap.sum_wall += wall;
    s_overlap.sum_wall2 += wall * wall;
    s_overlap.sum_x_wall += x * wall;
    s_overlap.sum_run += run;
    s_overlap.sum_run2 += run * run;
    s_overlap.sum_x_run += x * run;
}

/* Pearson 相關係數與斜率 (y 每多 1 ms 的 x 增加多少 ms)；資料不夠或 x / y 沒變化時回傳 false */
static bool overlap_fit(double n, double sx, double sxx, double sy, double syy, double sxy,
                        double *r, double *slope)
{
    const double var_x = n * sxx - sx * sx;
    const double var_y = n * syy - sy * sy;
    if (n < 3.0 || var_x <= 0.0 || var_y <= 0.0) {
        return false;
    }
    const double cov = n * sxy - sx * sy;
    *r = cov / sqrt(var_x * var_y);
    *slope = cov / var_x;
    return true;
}

/*
 * r 接近 1、斜率明顯大於 0 → Core 1 越忙推論越慢 (PSRAM / flash 匯流排競爭)。
 * r 接近 0 → 要找其他原因 (例如 flash cache 被其他程式碼洗掉)。
 * run 的 r 比 wall 的 r 更能代表匯流排競爭，因為已經扣掉被搶占的時間。
 */
static void overlap_report(void)
{
    if (s_overlap.n == 0U) {
        return;
    }
    const double n = (double)s_overlap.n;
    double r_wall = 0.0;
    double slope_wall = 0.0;
    double r_run = 0.0;
    double slope_run = 0.0;
    const bool wall_ok = overlap_fit(n, s_overlap.sum_x, s_overlap.sum_xx, s_overlap.sum_wall,
                                     s_overlap.sum_wall2, s_overlap.sum_x_wall, &r_wall, &slope_wall);
    const bool run_ok = overlap_fit(n, s_overlap.sum_x, s_overlap.sum_xx, s_overlap.sum_run,
                                    s_overlap.sum_run2, s_overlap.sum_x_run, &r_run, &slope_run);
    ESP_LOGI(TAG,
             "CNN vs Core 1 FD over %u invokes: avg invoke=%.1f run=%.1f preempt=%.1f fd_overlap=%.1f ms",
             (unsigned)s_overlap.n, s_overlap.sum_wall / n, s_overlap.sum_run / n,
             (s_overlap.sum_wall - s_overlap.sum_run) / n, s_overlap.sum_x / n);
    if (wall_ok && run_ok) {
        ESP_LOGI(TAG,
                 "CNN vs Core 1 FD fit: run r=%.2f slope=%.2f ms/ms | invoke r=%.2f slope=%.2f ms/ms",
                 r_run, slope_run, r_wall, slope_wall);
    } else {
        ESP_LOGI(TAG, "CNN vs Core 1 FD fit: not enough spread in the data");
    }
    memset(&s_overlap, 0, sizeof(s_overlap));
}

/* 候選 → 外擴 → 補正方形 → crop 64x64 → CNN。結果寫進 out，成功回傳 true */
static bool classify_candidate(const uint8_t *full_frame, const Stage2_CnnCandidate *candidate,
                               uint8_t *roi, roi_result_t *out)
{
    const int64_t crop_start_us = esp_timer_get_time();
    FD_BoundingBox box = candidate->bbox;
    if (!expand_bbox(&box, ROI_EXPAND_PX, FD_WIDTH, FD_HEIGHT) ||
        !expand_roi_to_square(&box, FD_WIDTH, FD_HEIGHT) ||
        !crop_resize_bbox(full_frame, FD_WIDTH, FD_HEIGHT, roi, CAM_MODEL_ROI_SIZE,
                          ROI_RESIZE_BILINEAR, &box)) {
        return false;
    }

    int pred = -1;
    out->crop_us = (uint32_t)(esp_timer_get_time() - crop_start_us);
    /* 推論一次 = 一個 64x64 ROI 的 CNN。統計本身的時間不算進 invoke_us */
    task_usage_begin();
    const uint32_t run_start_us = self_run_time_us();
    const uint64_t fd_start_us = fd_busy_now_us();
    const int64_t invoke_start_us = esp_timer_get_time();
    if (cam_model_predict(roi, out->probs, &pred) != ESP_OK || pred < 0) {
        return false;
    }
    out->invoke_us = (uint32_t)(esp_timer_get_time() - invoke_start_us);
    out->invoke_only_us = cam_model_last_invoke_us();
    out->fd_overlap_us = (uint32_t)(fd_busy_now_us() - fd_start_us);
    /* 無號相減，計數器 32 位元 (約 71 分鐘) 繞回也正確 */
    out->run_us = self_run_time_us() - run_start_us;
    overlap_note(out);
    task_usage_end();
    out->track_id = candidate->track_id;
    out->bbox[0] = box.x1;
    out->bbox[1] = box.y1;
    out->bbox[2] = box.x2;
    out->bbox[3] = box.y2;
    out->pred = (uint8_t)pred;
    out->conf = out->probs[pred];
    out->low_conf = out->conf < INFER_LOW_CONF_THRESHOLD;
    return true;
}

static void report_roi(const frame_job_t *job, const roi_result_t *roi)
{
    infer_record_t record = {.type = INFER_RECORD_CAM_ROI};
    infer_cam_roi_record_t *out = &record.data.roi;
    out->session = s_session.session_epoch;
    out->frame = job->timing.frame;
    out->ts_epoch_ms = job->timing.ts_epoch_ms;
    out->track_id = roi->track_id;
    memcpy(out->bbox, roi->bbox, sizeof(out->bbox));
    out->invoke_us = roi->invoke_us;
    out->pred = roi->pred;
    out->label = cam_model_class_name(roi->pred);
    out->conf = roi->conf;
    memcpy(out->probs, roi->probs, sizeof(out->probs));
    out->low_conf = roi->low_conf;
    result_report_push(&record);
    const uint32_t preempt_us = roi->invoke_us > roi->run_us ? roi->invoke_us - roi->run_us : 0U;
    ESP_LOGI(TAG, "frame %u track %d: %s %.3f [%.3f %.3f %.3f] cnn=%u ms invoke=%.1f ms "
             "run=%.1f preempt=%.1f fd_overlap=%.1f ms%s",
             (unsigned)job->timing.frame, (int)roi->track_id, out->label, (double)roi->conf,
             (double)roi->probs[0], (double)roi->probs[1], (double)roi->probs[2],
             (unsigned)(roi->invoke_us / 1000U), (double)roi->invoke_only_us / 1000.0,
             (double)roi->run_us / 1000.0, (double)preempt_us / 1000.0,
             (double)roi->fd_overlap_us / 1000.0, roi->low_conf ? " LOW" : "");
}

static void stage_add(stage_stat_t *stat, uint32_t elapsed_us)
{
    stat->total_us += elapsed_us;
    if (elapsed_us > stat->max_us) {
        stat->max_us = elapsed_us;
    }
}

static void stage_table_note(stage_table_t *table, const uint32_t values[STAGE_COUNT])
{
    for (size_t i = 0; i < STAGE_COUNT; ++i) {
        stage_add(&table->stats[i], values[i]);
    }
    ++table->frames;
}

/**
 * @brief 下採樣 → FD → 追蹤一幀，fb 的所有權在這裡交出去 (歸還驅動或交給 cam_infer)
 * @param has_prev [in,out] s_ds_prev 是否已有上一幀
 */
static void detect_frame(const ready_frame_t *ready, bool *has_prev)
{
    const int64_t started_us = esp_timer_get_time();
    infer_cam_frame_record_t timing = {
        .session = s_session.session_epoch,
        .frame = ready->frame_id,
        .ts_epoch_ms = infer_epoch_ms_from_timer(ready->timestamp_us),
        .capture_us = (uint32_t)(ready->enqueue_us - ready->timestamp_us),
        .queue_us = (uint32_t)(started_us - ready->enqueue_us),
    };

    downscale_frame(ready->frame->buf, s_ds_curr);
    const int64_t downscale_done_us = esp_timer_get_time();
    timing.downscale_us = (uint32_t)(downscale_done_us - started_us);

    frame_job_t *job = NULL;
    if (!*has_prev) {
        /* 第一幀只當 prev */
        *has_prev = true;
    } else {
        FD_StageTiming fd_stages;
        const int box_count = run_frame_difference(&fd_stages);
        const int64_t fd_done_us = esp_timer_get_time();
        timing.fd_us = (uint32_t)(fd_done_us - downscale_done_us);
        timing.fd_diff_us = fd_stages.diff_us;
        timing.fd_morph_us = fd_stages.morph_us;
        timing.fd_contour_us = fd_stages.contour_us;
        timing.fd_refine_us = fd_stages.refine_us;
        timing.fd_nms_us = fd_stages.nms_us;
        timing.boxes = (uint16_t)box_count;
        s_stats.last_fd_us = timing.fd_us;

        /* 有空的工作才收候選；沒有時 max_candidates = 0，候選留到該 track 下一次配對 */
        Stage2_CnnCandidate no_slot;
        (void)xQueueReceive(s_free_jobs, &job, 0);
        Stage2_CandidateBuffer candidates = {
            .candidates = job != NULL ? job->candidates : &no_slot,
            .max_candidates = job != NULL ? (int)INFER_CAM_MAX_ROI_PER_FRAME : 0,
            .num_candidates = 0,
        };
        if (!stage2_update_core(s_tracker, box_count > 0 ? s_fullres_boxes : NULL, box_count,
                                (int)ready->frame_id, &candidates)) {
            ESP_LOGE(TAG, "stage2_update_core failed at frame %u", (unsigned)ready->frame_id);
        }
        timing.track_us = (uint32_t)(esp_timer_get_time() - fd_done_us);
        timing.candidates = (uint16_t)candidates.num_candidates;
    }

    /* 這一幀變成下一輪的 prev；舊的 prev 已被 FD 當暫存覆蓋 */
    uint8_t *swap = s_ds_prev;
    s_ds_prev = s_ds_curr;
    s_ds_curr = swap;

    s_session.processed++;
    s_stats.frames++;
    timing.detect_us = (uint32_t)(esp_timer_get_time() - started_us);
    timing.total_us = timing.detect_us;

    const uint32_t values[STAGE_COUNT] = {
        [STAGE_CAPTURE] = timing.capture_us,
        [STAGE_QUEUE] = timing.queue_us,
        [STAGE_DOWNSCALE] = timing.downscale_us,
        [STAGE_FD] = timing.fd_us,
        [STAGE_FD_DIFF] = timing.fd_diff_us,
        [STAGE_FD_MORPH] = timing.fd_morph_us,
        [STAGE_FD_CONTOUR] = timing.fd_contour_us,
        [STAGE_FD_REFINE] = timing.fd_refine_us,
        [STAGE_FD_NMS] = timing.fd_nms_us,
        [STAGE_TRACK] = timing.track_us,
        [STAGE_DETECT] = timing.detect_us,
    };
    stage_table_note(&s_detect_stages, values);

    if (timing.candidates > 0U) {
        /* 這幀的記錄等 CNN 做完才回報 */
        job->frame = ready->frame;
        job->timing = timing;
        job->candidate_count = timing.candidates;
        job->handed_over_us = esp_timer_get_time();
        (void)xQueueSend(s_cnn_queue, &job, 0);
        return;
    }

    if (job != NULL) {
        (void)xQueueSend(s_free_jobs, &job, 0);
    }
    esp_camera_fb_return(ready->frame);
    infer_record_t record = {.type = INFER_RECORD_CAM_FRAME};
    record.data.frame = timing;
    result_report_push(&record);
}

static void report_stats(void);
static void report_cnn_work(void);

/**
 * @brief cam_detect 任務：常駐，FD 與追蹤器只由這個 task 呼叫
 * 運行於：CPU Core 1，優先權 5。追蹤器跨幀一直延續，不等 CNN。
 */
static void cam_detect(void *argument)
{
    (void)argument;
    bool has_prev = false;
    ready_frame_t ready;
    while (true) {
        (void)xQueueReceive(s_ready_queue, &ready, portMAX_DELAY);
        fd_busy_mark(true);
        detect_frame(&ready, &has_prev);
        fd_busy_mark(false);
        /*
         * ESP32 的 PSRAM 較慢，FD 可能超過一個幀週期，佇列一直有幀時這裡不會阻塞，
         * IDLE1 排不到就會觸發 5 s 看門狗警告；跟不上時讓出 1 tick (跟得上時不影響)。
         */
        if (uxQueueMessagesWaiting(s_ready_queue) > 0U) {
            vTaskDelay(1);
        }
        /* 只抓 snapshot，不印 log；印表交給 cam_infer，不佔 FD 的 200 ms */
        if ((s_session.processed % USAGE_CYCLE_FRAMES) == 0U) {
            task_usage_cycle_mark();
        }
        if ((s_session.processed % STATS_PERIOD_FRAMES) == 0U) {
            report_stats();
        }
    }
}

/**
 * @brief 跑完一幀的所有候選；fb 的所有權在這裡結束 (歸還驅動或交給 cam_writer)
 */
static void run_cnn_job(frame_job_t *job)
{
    const int64_t started_us = esp_timer_get_time();
    infer_cam_frame_record_t *timing = &job->timing;
    timing->cnn_wait_us = (uint32_t)(started_us - job->handed_over_us);

    bool any_low = false;
    bool any_non_background = false;
    job->roi_count = 0;
    for (size_t k = 0; k < job->candidate_count; ++k) {
        const Stage2_CnnCandidate *candidate = &job->candidates[k];
        if (!candidate->valid) {
            continue;
        }
        roi_result_t *roi = &job->rois[job->roi_count];
        if (!classify_candidate(job->frame->buf, candidate, job->roi_pixels[job->roi_count], roi)) {
            ESP_LOGW(TAG, "ROI failed: frame %u track %d", (unsigned)timing->frame,
                     candidate->track_id);
            continue;
        }
        ++job->roi_count;
        timing->crop_us += roi->crop_us;
        timing->cnn_us += roi->invoke_us;
        report_roi(job, roi);
        s_stats.last_invoke_us = roi->invoke_us;
        any_low = any_low || roi->low_conf;
        any_non_background = any_non_background ||
                             (s_background_class >= 0 && roi->pred != (uint8_t)s_background_class);
    }
    job->any_low = any_low;
    job->any_non_background = any_non_background;
    s_session.rois += (uint32_t)job->roi_count;
    s_stats.rois += (uint32_t)job->roi_count;
    timing->roi_us = (uint32_t)(esp_timer_get_time() - started_us);
    timing->total_us = timing->detect_us + timing->roi_us;

    const uint32_t values[STAGE_COUNT] = {
        [STAGE_CNN_WAIT] = timing->cnn_wait_us,
        [STAGE_CROP] = timing->crop_us,
        [STAGE_CNN] = timing->cnn_us,
    };
    stage_table_note(&s_cnn_stages, values);

    infer_record_t record = {.type = INFER_RECORD_CAM_FRAME};
    record.data.frame = *timing;
    result_report_push(&record);

    if (any_low) {
        s_session.low_conf++;
        s_stats.low_conf++;
    }
    /* 低信賴或非背景：整幀交給 cam_writer，fb 由 writer 寫完才歸還 */
    if (any_low || any_non_background) {
        job->handed_over_us = esp_timer_get_time();
        (void)xQueueSend(s_save_queue, &job, 0);
        return;
    }
    esp_camera_fb_return(job->frame);
    job->frame = NULL;
    (void)xQueueSend(s_free_jobs, &job, 0);
}

/**
 * @brief cam_infer 任務：常駐，TFLM 只由這個 task 呼叫
 * 運行於：CPU Core 0，優先權 1 (理由見 INFER_TASK_PRIORITY)。
 */
static void cam_infer(void *argument)
{
    (void)argument;
    int64_t last_report_us = esp_timer_get_time();
    while (true) {
        frame_job_t *job = NULL;
        if (xQueueReceive(s_cnn_queue, &job, pdMS_TO_TICKS(CNN_REPORT_PERIOD_MS)) == pdTRUE) {
            run_cnn_job(job);
            /* 候選一直來時上面不會阻塞，IDLE0 排不到就會觸發 5 s 看門狗警告 */
            vTaskDelay(1);
        }
        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_report_us >= (int64_t)CNN_REPORT_PERIOD_MS * 1000LL) {
            last_report_us = now_us;
            report_cnn_work();
        }
    }
}

static esp_err_t write_roi_pixels(int fd, const frame_job_t *job)
{
    for (size_t i = 0; i < job->roi_count; ++i) {
        memcpy(s_sd_dma_buffer, job->roi_pixels[i], CAM_MODEL_ROI_BYTES);
        ESP_RETURN_ON_ERROR(write_all(fd, s_sd_dma_buffer, CAM_MODEL_ROI_BYTES), TAG, "ROI write");
    }
    return ESP_OK;
}

/* meta 格式：server_mix.py 依 image_bytes、roi_count、roi_size 切開資料檔 */
static bool build_sample_meta(frame_job_t *job, size_t image_bytes, int64_t encode_us,
                              int64_t write_us)
{
    size_t used = 0;
    int written = snprintf(
        job->meta, SAMPLE_META_BYTES,
        "{\"media\":\"video\",\"session\":%" PRId64 ",\"frame\":%" PRIu32 ",\"ts_ms\":%" PRId64
        ",\"width\":%u,\"height\":%u,\"format\":\"%s\",\"image_bytes\":%u"
        ",\"roi_size\":%d,\"roi_count\":%u"
        ",\"threshold\":%.2f,\"queue_us\":%" PRIu32 ",\"downscale_us\":%" PRIu32
        ",\"fd_us\":%" PRIu32 ",\"track_us\":%" PRIu32 ",\"encode_us\":%" PRId64
        ",\"write_us\":%" PRId64 ",\"save_reason\":\"%s\",\"labels\":[\"%s\",\"%s\",\"%s\"],\"rois\":[",
        s_session.session_epoch, job->timing.frame, job->timing.ts_epoch_ms,
        (unsigned)CAM_IMAGE_WIDTH, (unsigned)CAM_IMAGE_HEIGHT,
        INFER_CAM_SAMPLE_JPEG ? "JPEG" : "GRAY8", (unsigned)image_bytes, CAM_MODEL_ROI_SIZE,
        (unsigned)job->roi_count, (double)INFER_LOW_CONF_THRESHOLD, job->timing.queue_us,
        job->timing.downscale_us, job->timing.fd_us, job->timing.track_us, encode_us, write_us,
        sample_store_save_reason(job->any_low, job->any_non_background),
        cam_model_class_name(0), cam_model_class_name(1), cam_model_class_name(2));
    if (written < 0 || (size_t)written >= SAMPLE_META_BYTES) {
        return false;
    }
    used = (size_t)written;
    for (size_t i = 0; i < job->roi_count; ++i) {
        const roi_result_t *roi = &job->rois[i];
        written = snprintf(job->meta + used, SAMPLE_META_BYTES - used,
                           "%s{\"track\":%" PRId32 ",\"bbox\":[%" PRId32 ",%" PRId32 ",%" PRId32
                           ",%" PRId32 "],\"invoke_us\":%" PRIu32 ",\"pred\":%u,\"label\":\"%s\""
                           ",\"conf\":%.4f,\"low\":%s,\"probs\":[%.4f,%.4f,%.4f]}",
                           i > 0U ? "," : "", roi->track_id, roi->bbox[0], roi->bbox[1],
                           roi->bbox[2], roi->bbox[3], roi->invoke_us, (unsigned)roi->pred,
                           cam_model_class_name(roi->pred), (double)roi->conf,
                           roi->low_conf ? "true" : "false", (double)roi->probs[0],
                           (double)roi->probs[1], (double)roi->probs[2]);
        if (written < 0 || (size_t)written >= SAMPLE_META_BYTES - used) {
            return false;
        }
        used += (size_t)written;
    }
    written = snprintf(job->meta + used, SAMPLE_META_BYTES - used, "]}");
    return written > 0 && (size_t)written < SAMPLE_META_BYTES - used;
}

/* 資料檔 = 影像 (raw 640x480 或 JPEG) + 每個 ROI 64x64 (raw)，直接接在一起 */
static esp_err_t write_sample(frame_job_t *job, const uint8_t *image, size_t image_bytes,
                              int64_t encode_us)
{
    sample_file_t file;
    ESP_RETURN_ON_ERROR(sample_store_begin(&file, SAMPLE_MEDIA_VIDEO, job->timing.ts_epoch_ms),
                        TAG, "begin sample");
    const int64_t write_start_us = esp_timer_get_time();
    int64_t max_chunk_us = 0;
    esp_err_t err = write_image_to_sd(file.fd, image, image_bytes, &max_chunk_us);
    if (err == ESP_OK) {
        err = write_roi_pixels(file.fd, job);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD write failed at frame %u: errno=%d (%s)",
                 (unsigned)job->timing.frame, errno, strerror(errno));
        sample_store_abort(&file);
        return err;
    }
    if (!build_sample_meta(job, image_bytes, encode_us, esp_timer_get_time() - write_start_us)) {
        ESP_LOGE(TAG, "Sample meta does not fit");
        sample_store_abort(&file);
        return ESP_ERR_INVALID_SIZE;
    }
    return sample_store_commit(&file, job->meta);
}

static esp_err_t save_frame_sample(frame_job_t *job)
{
#if INFER_CAM_SAMPLE_JPEG
    /* frame2jpg 配置輸出緩衝 (128 KB，PSRAM)；太複雜的畫面超過時編碼失敗，這幀不存 */
    uint8_t *jpeg = NULL;
    size_t jpeg_bytes = 0;
    const int64_t encode_start_us = esp_timer_get_time();
    if (!frame2jpg(job->frame, INFER_CAM_JPEG_QUALITY, &jpeg, &jpeg_bytes)) {
        ESP_LOGE(TAG, "JPEG encode failed at frame %u", (unsigned)job->timing.frame);
        return ESP_FAIL;
    }
    const esp_err_t err = write_sample(job, jpeg, jpeg_bytes,
                                       esp_timer_get_time() - encode_start_us);
    free(jpeg);
    return err;
#else
    return write_sample(job, job->frame->buf, IMAGE_PIXELS, 0);
#endif
}

/**
 * @brief cam_writer 任務：把低信賴度 (或 INFER_SAVE_NON_BACKGROUND 時非背景) 的幀寫進 SD 卡
 * 運行於：CPU Core 0，優先權 5；常駐。
 * 職責：從 s_save_queue 取工作，影像 (PSRAM → 內部暫存 → SD) + ROI + meta 寫成一個樣本，
 * 寫完才 esp_camera_fb_return() (零拷貝：fb 一直被持有到這裡)。
 */
static void cam_writer(void *argument)
{
    (void)argument;
    while (true) {
        frame_job_t *job = NULL;
        (void)xQueueReceive(s_save_queue, &job, portMAX_DELAY);
        const int64_t save_start_us = esp_timer_get_time();
        stage_add(&s_session.save_wait, (uint32_t)(save_start_us - job->handed_over_us));
        const esp_err_t save_err = save_frame_sample(job);
        stage_add(&s_session.save, (uint32_t)(esp_timer_get_time() - save_start_us));
        if (save_err == ESP_OK) {
            s_session.saved++;
            s_stats.saved++;
        } else {
            s_session.save_failed++;
        }
        esp_camera_fb_return(job->frame);
        job->frame = NULL;
        (void)xQueueSend(s_free_jobs, &job, 0);
    }
}

/* 上次回報以來每個階段的平均 / 最大 (ms)，每次回報後歸零 */
static void log_stage_times(stage_table_t *table, const char *what)
{
    if (table->frames == 0U) {
        return;
    }
    char line[384];
    size_t used = 0;
    line[0] = '\0';
    for (size_t i = 0; i < STAGE_COUNT; ++i) {
        if (table->stats[i].max_us == 0U) {
            continue;   /* 另一個 task 的階段 */
        }
        const int written = snprintf(line + used, sizeof(line) - used, "%s=%.1f/%.1f ",
                                     STAGE_NAMES[i],
                                     (double)table->stats[i].total_us / table->frames / 1000.0,
                                     (double)table->stats[i].max_us / 1000.0);
        if (written < 0 || (size_t)written >= sizeof(line) - used) {
            break;
        }
        used += (size_t)written;
    }
    ESP_LOGI(TAG, "%s stage avg/max ms over %u frames: %s", what, (unsigned)table->frames, line);
    memset(table, 0, sizeof(*table));
}

/* CNN 每一層的平均耗時：回傳 server 並印出，看哪一層最花時間 */
static void report_cnn_ops(void)
{
    cam_model_op_time_t ops[CAM_MODEL_MAX_OPS];
    uint32_t invokes = 0;
    const size_t count = cam_model_take_op_times(ops, &invokes);
    if (invokes == 0U) {
        return;
    }

    infer_record_t record = {.type = INFER_RECORD_CAM_OPS};
    infer_cam_ops_record_t *out = &record.data.ops;
    out->session = s_session.session_epoch;
    out->frame = s_session.processed;
    out->invokes = invokes;
    out->op_count = (uint8_t)count;

    char line[768];
    size_t used = 0;
    line[0] = '\0';
    for (size_t i = 0; i < count; ++i) {
        out->ops[i].name = ops[i].name;
        out->ops[i].avg_us = ops[i].total_us / invokes;
        const int written = snprintf(line + used, sizeof(line) - used, "%s=%" PRIu32 " ",
                                     ops[i].name, out->ops[i].avg_us);
        if (written > 0 && (size_t)written < sizeof(line) - used) {
            used += (size_t)written;
        }
    }
    result_report_push(&record);
    ESP_LOGI(TAG, "CNN op avg us over %u invokes: %s", (unsigned)invokes, line);
}

/* TFLM 和推論期間的 task_usage 只有 cam_infer 能讀，所以由它自己回報；一輪的表也在這裡印 (有鎖) */
static void report_cnn_work(void)
{
    log_stage_times(&s_cnn_stages, "CNN");
    report_cnn_ops();
    overlap_report();
    task_usage_report("video");
    task_usage_cycle_report("video cycle");
}

/*
 * 開機以來的累計統計，由 cam_detect 呼叫：dropped / gaps / 驅動丟棄幀看 FD 跟不跟得上，
 * rois_expired 看 CNN 跟不跟得上。
 */
static void report_stats(void)
{
    const session_state_t *session = &s_session;
    const uint32_t incomplete = cam_get_incomplete_frames() - s_incomplete_at_start;
    const uint32_t save_attempts = session->saved + session->save_failed;
    s_stats.dropped = session->dropped;

    infer_record_t record = {.type = INFER_RECORD_CAM_SESSION};
    infer_cam_session_record_t *out = &record.data.session;
    out->session = session->session_epoch;
    out->frames = session->frames;
    out->processed = session->processed;
    out->dropped = session->dropped;
    out->gaps = session->gaps;
    out->incomplete = incomplete;
    out->rois = session->rois;
    out->rois_expired = s_tracker->expired_candidates;
    out->low_conf = session->low_conf;
    out->saved = session->saved;
    out->save_skipped = session->save_failed;
    out->first_frame_offset_us =
        session->frames > 0U ? (int32_t)(session->first_frame_us - session->trigger_us) : 0;
    out->interval_min_us = (int32_t)session->interval_min_us;
    out->interval_max_us = (int32_t)session->interval_max_us;
    out->error = (int32_t)session->timeouts;
    if (save_attempts > 0U) {
        out->save_wait_avg_us = (uint32_t)(session->save_wait.total_us / save_attempts);
        out->save_avg_us = (uint32_t)(session->save.total_us / save_attempts);
    }
    out->save_wait_max_us = session->save_wait.max_us;
    out->save_max_us = session->save.max_us;
    result_report_push(&record);
    log_stage_times(&s_detect_stages, "Detect");

    ESP_LOGI(TAG,
             "Camera: frames=%u processed=%u dropped=%u gaps=%u incomplete=%u "
             "stale=%u invalid=%u timeout=%u timestamp_error=%u rois=%u expired=%u low=%u saved=%u skipped=%u "
             "interval=%" PRId64 "..%" PRId64 " us",
             (unsigned)session->frames, (unsigned)session->processed,
             (unsigned)session->dropped, (unsigned)session->gaps, (unsigned)incomplete,
             (unsigned)session->stale, (unsigned)session->invalid, (unsigned)session->timeouts,
             (unsigned)session->timestamp_errors, (unsigned)session->rois,
             (unsigned)out->rois_expired, (unsigned)session->low_conf, (unsigned)session->saved,
             (unsigned)out->save_skipped, session->interval_min_us, session->interval_max_us);
    ESP_LOGI(TAG, "Stack free: cam_detect=%u B cam_infer=%u B; internal RAM free=%u B min=%u B",
             (unsigned)uxTaskGetStackHighWaterMark(s_detect_task),
             (unsigned)uxTaskGetStackHighWaterMark(s_infer_task),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
}

esp_err_t cam_infer_start(uint8_t *staging_buffer)
{
    if (!s_initialized || !staging_buffer) {
        return ESP_ERR_INVALID_STATE;
    }
    s_sd_dma_buffer = staging_buffer;

    memset(&s_session, 0, sizeof(s_session));
    s_session.session_epoch = (int64_t)time(NULL);

    /* resume 會先恢復 XCLK 並等感測器穩定；起點放在之後，暫停前與 XCLK 剛恢復的幀都丟掉 */
    ESP_RETURN_ON_ERROR(cam_infer_resume(), TAG, "camera resume");
    s_session.trigger_us = esp_timer_get_time() + START_MARGIN_US;
    s_incomplete_at_start = cam_get_incomplete_frames();
    if (xTaskCreatePinnedToCore(cam_writer, "cam_writer", WRITER_TASK_STACK_BYTES, NULL,
                                WRITER_TASK_PRIORITY, NULL, WRITER_TASK_CORE) != pdPASS ||
        xTaskCreatePinnedToCore(camera_capture, "camera_capture", CAPTURE_TASK_STACK_BYTES, NULL,
                                CAPTURE_TASK_PRIORITY, NULL, CAPTURE_TASK_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Camera running continuously at %u fps", (unsigned)TARGET_FPS);
    return ESP_OK;
}

void cam_infer_get_stats(cam_infer_stats_t *stats)
{
    if (stats) {
        *stats = s_stats;
    }
}

/* 開機時配置一次，之後不再 malloc，錄影期間不動到 heap */
static esp_err_t init_pipeline(void)
{
    const uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    s_ds_prev = heap_caps_malloc(FD_PROC_BYTES, caps);
    s_ds_curr = heap_caps_malloc(FD_PROC_BYTES, caps);
    s_fd_temp = heap_caps_malloc(FD_PROC_BYTES, caps);
    s_fullres_boxes = heap_caps_malloc(sizeof(FD_BoundingBox) * STAGE2_MAX_DETECTIONS, caps);
    s_tracker = heap_caps_malloc(sizeof(Stage2_Tracker), caps);
    if (!s_ds_prev || !s_ds_curr || !s_fd_temp || !s_fullres_boxes || !s_tracker) {
        return ESP_ERR_NO_MEM;
    }

    s_ready_queue = xQueueCreateStatic(READY_QUEUE_CAPACITY, sizeof(ready_frame_t),
                                       s_ready_queue_storage, &s_ready_queue_control);
    s_free_jobs = xQueueCreateStatic(FRAME_JOB_COUNT, sizeof(frame_job_t *),
                                     s_free_jobs_storage, &s_free_jobs_control);
    s_cnn_queue = xQueueCreateStatic(FRAME_JOB_COUNT, sizeof(frame_job_t *),
                                     s_cnn_queue_storage, &s_cnn_queue_control);
    s_save_queue = xQueueCreateStatic(FRAME_JOB_COUNT, sizeof(frame_job_t *),
                                      s_save_queue_storage, &s_save_queue_control);
    if (!s_ready_queue || !s_free_jobs || !s_cnn_queue || !s_save_queue) {
        return ESP_ERR_NO_MEM;
    }
    for (size_t i = 0; i < FRAME_JOB_COUNT; ++i) {
        frame_job_t *job = heap_caps_malloc(sizeof(frame_job_t), caps);
        if (!job) {
            return ESP_ERR_NO_MEM;
        }
        (void)xQueueSend(s_free_jobs, &job, 0);
    }

    /* FD_did_cam_v4 相機模式的參數；blur 在原專案就是關閉的 */
    s_fd_params = (FD_Parameters){
        .thresh = 30,
        .bbox_thresh = 0,
        .iou_threshold = 1e-4f,
        .morph_kernel_size = 3,
        .morph_mode = FD_MORPH_DILATE,
        .roi_refine_enable = true,
        .roi_refine_expand_px = 5,
        .roi_refine_sobel_thresh = 100,
        .roi_refine_min_cc_area = 0,
        .roi_refine_keep_largest_only = false,
    };
    Stage2_Params track_params;
    stage2_default_params(&track_params);
    track_params.max_match_dist = 240;
    track_params.max_miss = 2;
    track_params.age_threshold = 3;
    track_params.enable_area_ratio_gate = true;
    track_params.area_ratio_min = 0.30f;
    track_params.area_ratio_max = 3.00f;
    track_params.max_active_tracks = STAGE2_MAX_TRACKS;
    stage2_init(s_tracker, &track_params);

    ESP_RETURN_ON_ERROR(cam_model_init(), TAG, "Camera model initialization failed");
    const char *class_names[CAM_MODEL_CLASS_COUNT];
    for (int i = 0; i < CAM_MODEL_CLASS_COUNT; ++i) {
        class_names[i] = cam_model_class_name(i);
    }
    s_background_class = sample_store_find_background(class_names, CAM_MODEL_CLASS_COUNT);
    ESP_LOGI(TAG, "Saving frames with %s", s_background_class >= 0 ?
             "low-confidence or non-background ROIs" : "low-confidence ROIs only");

    /* 常駐：TFLM 只由 cam_infer 呼叫，FD 與追蹤器只由 cam_detect 呼叫；static 不佔動態 heap */
    s_infer_task = xTaskCreateStaticPinnedToCore(cam_infer, "cam_infer", INFER_TASK_STACK_BYTES,
                                                 NULL, INFER_TASK_PRIORITY, s_infer_stack,
                                                 &s_infer_tcb, INFER_TASK_CORE);
    s_detect_task = xTaskCreateStaticPinnedToCore(cam_detect, "cam_detect", DETECT_TASK_STACK_BYTES,
                                                  NULL, DETECT_TASK_PRIORITY, s_detect_stack,
                                                  &s_detect_tcb, DETECT_TASK_CORE);
    if (!s_infer_task || !s_detect_task) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Inference pipeline ready: camera_fb=%u ready_queue=%u frame_jobs=%u, PSRAM free=%u",
             (unsigned)CAMERA_FRAME_BUFFERS, (unsigned)READY_QUEUE_CAPACITY,
             (unsigned)FRAME_JOB_COUNT, (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    return ESP_OK;
}

esp_err_t cam_infer_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    /* 步驟 1：相機感測器與外部 PSRAM 初始化 */
    sensor_t *sensor = NULL;
    esp_err_t err = init_camera(&sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 步驟 2：開啟自動曝光/白平衡，準備進行暖機 */
    err = set_auto_controls(sensor, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot enable camera auto controls: %s", esp_err_to_name(err));
        esp_camera_deinit();
        return err;
    }

    /* 步驟 3：丟棄暖機幀，讓感測器自動曝光收斂到適當明暗度 */
    err = discard_warmup_frames(sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera warm-up failed: %s", esp_err_to_name(err));
        esp_camera_deinit();
        return err;
    }

    /* 步驟 4：鎖定曝光與白平衡，或維持 auto 跟著當下光線調整；
     * 降頻 / 虛擬行已在 configure_sensor() 另外關閉，幀率不受影響。 */
#if APP_CAM_LOCK_AUTO_CONTROLS
    err = set_auto_controls(sensor, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot lock camera controls: %s", esp_err_to_name(err));
        esp_camera_deinit();
        return err;
    }
#else
    ESP_LOGW(TAG, "AE/AGC/AWB left enabled (APP_CAM_LOCK_AUTO_CONTROLS=0)");
#endif

    /* 步驟 5：開機 5 秒 5 FPS 資格驗證，只驗證硬體 */
    err = run_startup_fps_test();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Startup FPS test failed; camera inference stays disabled");
        esp_camera_deinit();
        return err;
    }

    /* 步驟 6：FD / 追蹤緩衝、模型與 cam_detect / cam_infer task */
    err = init_pipeline();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Inference pipeline initialization failed: %s", esp_err_to_name(err));
        esp_camera_deinit();
        return err;
    }

    s_initialized = true;
    s_camera_paused = false;
    ESP_LOGI(TAG, "Camera inference initialized");
    return ESP_OK;
}

esp_err_t cam_infer_pause(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_camera_paused) {
        return xclk_stop();
    }

    /* 任務皆已結束後，等待 cam_task 清除未完成的 DMA/影格事件。 */
    esp_err_t err = cam_stop();
    if (err != ESP_OK) {
        return err;
    }
    s_camera_paused = true;
    /* DMA 已停，才停 XCLK，避免 cam_task 收到半幀 */
    return xclk_stop();
}

static esp_err_t cam_infer_resume(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_camera_paused) {
        ESP_RETURN_ON_ERROR(xclk_start(), TAG, "XCLK restart failed");
        xclk_wait_settled();
        /* cam_start 只開 VSYNC；下一個 VSYNC 由 cam_task 呼叫 ll_cam_start。 */
        cam_start();
        s_camera_paused = false;
    }
    return ESP_OK;
}
