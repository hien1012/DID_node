/**
 * ============================================================================
 * 專案名稱：ESP32 野外影像採集節點 (Camera Node)
 * 檔案名稱：cam_app_main.c
 * 檔案說明：低階影像擷取 (DVP Camera)、零拷貝佇列管線與 SD 卡連續寫入核心實作
 * ============================================================================
 * 
 * 【給客戶與開發團隊的架構摘要說明】
 * ----------------------------------------------------------------------------
 * 1. 系統目標：
 *    在資源受限的 ESP32 晶片上，以極高穩定度達成 640x480 (VGA) 解析度、
 *    每秒 5 張 (5 FPS) 的無損連續灰階錄影，並即時寫入 FAT32 SD 卡中。
 * 
 * 2. 為什麼採用 RAW 格式 (GRAY8)？
 *    - JPEG 壓縮在 ESP32 上需要耗費大量 CPU 運算時間與記憶體，且每一幀壓縮時間
 *      浮動極大，容易導致硬體 FIFO 溢位而掉幀。
 *    - RAW 格式每一幀固定為 640 x 480 = 307,200 位元組 (約 300 KB)，資料大小完全固定，
 *      便於做精確的記憶體預分配與即時 SD 卡扇區寫入，最後再由伺服器轉為 MP4。
 * 
 * 3. 雙核心並行與零拷貝 (Zero-Copy) 管線設計：
 *    - 核心 1 (Core 1) [高優先權 6]：專注運行 capture_task，負責從相機 DMA 接收影格，
 *      僅傳遞指標進入 FreeRTOS 佇列，絕對不複製像素資料，確保不遺漏任何硬體訊號。
 *    - 核心 0 (Core 0) [低優先權 5]：專注運行 writer_task，負責將佇列中的影像寫入 SD 卡。
 *      寫入完成後才呼叫 esp_camera_fb_return() 將緩衝區釋放回硬體，避免記憶體碎裂。
 * 
 * 4. 關鍵記憶體限制與解決方案：
 *    - ESP32 內部 SRAM 空間不足以存放 VGA 影格，因此影格存放在外部 PSRAM (8 個緩衝區)。
 *    - 但 ESP32 的 SDMMC DMA 無法直接且高效地存取外部 PSRAM，因此我們在內部 SRAM 
 *      特別配置了一塊 32KB 的 DMA 暫存區 (s_sd_dma_buffer)，以分段複製的方式高速寫入 SD。
 * 
 * 5. 檔案預先配置 (Preallocation)：
 *    - SD 卡寫入時，若檔案大小動態增長，FAT 檔案系統需要頻繁更新目錄項與配置表，
 *      這會造成不可預測的幾百毫秒延遲 (SD 延遲峰值)。
 *    - 我們在錄影開始前，直接透過 lseek + write 將檔案預先撐大到 10 秒所需的完整大小 (15 MB)，
 *      錄影期間只需順序覆寫扇區，徹底消除了動態配置檔案造成的寫入卡頓。
 * ============================================================================
 */

/* ============================================================================
 * 標頭檔引用 (Includes)
 * ============================================================================ */

/* 標準 C 語言函式庫與 POSIX 系統呼叫 */
#include <errno.h>        /* 系統錯誤碼定義 (如 EEXIST, EINTR) */
#include <fcntl.h>        /* 檔案控制選項 (如 O_CREAT, O_WRONLY) */
#include <inttypes.h>     /* 標準整數型別格式化輸出 (如 PRIu64, PRId64) */
#include <stdbool.h>      /* 布林型別 (true, false) */
#include <stddef.h>       /* 標準定義 (size_t, NULL) */
#include <stdint.h>       /* 固定寬度整數型別 (uint8_t, int64_t 等) */
#include <stdio.h>        /* 標準輸入輸出 (snprintf, printf) */
#include <string.h>       /* 字串與記憶體操作 (memcpy, memset, strerror) */
#include <sys/stat.h>     /* 檔案狀態結構 */
#include <unistd.h>       /* POSIX 系統呼叫 (open, close, write, lseek, unlink, ftruncate, fsync) */

/* ESP-IDF 硬體驅動與周邊控制 */
#include "driver/gpio.h"        /* GPIO 通用輸入輸出腳位控制 (按鈕偵測) */
#include "driver/sdmmc_host.h"  /* SDMMC 硬體主機控制器驅動 */
#include "esp_camera.h"         /* 官方相機驅動介面 (相機初始化、擷取影格) */
#include "esp_check.h"          /* ESP-IDF 錯誤檢查巨集 (ESP_RETURN_ON_ERROR) */
#include "esp_err.h"            /* ESP 錯誤碼型別定義 (esp_err_t) */
#include "esp_heap_caps.h"      /* 記憶體堆積能力分配器 (指定內部 SRAM、DMA、PSRAM) */
#include "esp_log.h"            /* 系統日誌輸出 (ESP_LOGI, ESP_LOGE 等) */
#include "esp_psram.h"          /* 外部 PSRAM 驅動與狀態查詢 */
#include "esp_timer.h"          /* 高精度硬體微秒計時器 (esp_timer_get_time) */
#include "esp_vfs_fat.h"        /* FAT 檔案系統虛擬檔案系統 (VFS) 整合介面 */
#include "sdmmc_cmd.h"          /* SD/MMC 協定命令控制 */
#include "sdkconfig.h"          /* 專案編譯組態 (menuconfig 生成的常數) */
#include "sensor.h"             /* 相機感測器底層暫存器讀寫與結構定義 */

/* 本專案自訂硬體腳位與暫存器定義 */
#include "camera_pinout.h"      /* 相機硬體腳位定義 (D0-D7, VSYNC, HREF, PCLK, XCLK, I2C 等) */
#include "camera_registers.h"   /* 各型號感測器專屬暫存器位址定義 */

/* ============================================================================
 * 巨集與常數定義 (Macros and Constants)
 * ============================================================================ */

/* --- 影像規格參數 --- */
#define IMAGE_WIDTH             640U                        /* 影像寬度：640 像素 (VGA 規格) */
#define IMAGE_HEIGHT            480U                        /* 影像高度：480 像素 (VGA 規格) */
#define IMAGE_PIXELS            (IMAGE_WIDTH * IMAGE_HEIGHT)/* 每一幀灰階像素總數：640*480 = 307,200 位元組 (Bytes) */
#define SENSOR_YUV422_BYTES     (IMAGE_PIXELS * 2U)         /* 感測器原始輸出為 YUV422，每像素佔 2 位元組 = 614,400 位元組 */

/*
 * --- 相機感測器時脈源選擇 (Clock-Source Configuration) ---
 *
 * 說明：
 * 本專案硬體可能搭載不同的相機感測器模組 (如 OV2640、OV7670、OV7725)。
 * 部分 OV2640 模組自帶 12 MHz 石英震盪器 (Onboard Oscillator) 直連 XVCLK。
 * 若感測器已有自帶震盪器，ESP32 絕不可再由內部輸出 XCLK，否則兩組時脈訊號會互相打架短路。
 * 反之，OV7670/OV7725 模組通常需要由 ESP32 的內部定時器產生 16 MHz XCLK 供給它。
 */
#define USE_OV2640_ONBOARD_XVCLK 0                          /* 0: 由 ESP32 產生 16MHz XCLK; 1: 使用 OV2640 板載 12MHz 時脈 */
#define ESP32_CAMERA_XCLK_HZ     16000000U                  /* ESP32 輸出的 XCLK 時脈頻率：16 MHz */
#define OV2640_ONBOARD_XVCLK_HZ  12000000U                  /* OV2640 外部板載震盪器頻率：12 MHz */
#define OV2640_HAL_XCLK_HINT_HZ  20000000U                  /* 提供給相機底層驅動的時脈提示值 (設定超過 10MHz 會影響 I2S 採樣模式選擇) */

#if USE_OV2640_ONBOARD_XVCLK
#define CAMERA_CONFIG_XCLK_PIN   (-1)                       /* -1 代表不啟用 ESP32 的 XCLK 輸出腳位，由板載震盪器自行驅動 */
#define CAMERA_CONFIG_XCLK_HZ    OV2640_HAL_XCLK_HINT_HZ
#else
#define CAMERA_CONFIG_XCLK_PIN   CAM_PIN_XCLK               /* 指向 camera_pinout.h 中定義的 XCLK 實體 GPIO 腳位 */
#define CAMERA_CONFIG_XCLK_HZ    ESP32_CAMERA_XCLK_HZ       /* 設定為 16 MHz */
#endif

/* --- 錄影與影格率驗證參數 --- */
#define CAMERA_WARMUP_FRAMES    10U                         /* 開機暖機幀數：丟棄前 10 幀，讓感測器自動曝光/白平衡穩定 */
#define CAMERA_FRAME_BUFFERS    8U                          /* 配置於外部 PSRAM 的影格緩衝區數量 (總計 8 * 300KB ≈ 2.4MB) */
#define TEST_DURATION_SECONDS   10U                         /* 開機硬體資格測試時間：10 秒 */
#define TARGET_FPS               5U                         /* 目標影格率：每秒 5 幀 */
#define EXPECTED_FRAME_COUNT     (TEST_DURATION_SECONDS * TARGET_FPS) /* 10 秒預期應收到的影格數：50 幀 */
#define EXPECTED_FRAME_PERIOD_US (1000000LL / TARGET_FPS)   /* 每一幀的理想間隔微秒數：1,000,000 / 5 = 200,000 us (200 毫秒) */
#define GAP_THRESHOLD_US         (EXPECTED_FRAME_PERIOD_US * 3LL / 2LL) /* 掉幀判定門檻：間隔超過 1.5 週期 (300 毫秒) 即視為掉幀事件 */
#define FPS_TOLERANCE_MILLI      250U                       /* 影格率容許誤差：±0.25 FPS (4.75 ~ 5.25 FPS 視為合格) */

/* --- SD 卡與檔案系統參數 --- */
#define SD_MOUNT_POINT                "/sdcard"             /* SD 卡於 VFS 中的掛載路徑前綴 */
#define SD_MAX_OPEN_FILES             2                     /* SD 卡檔案系統允許同時開啟的最大檔案數 */
#define SD_FORMAT_ALLOCATION_BYTES    (16U * 1024U)         /* 格式化配置單元大小 (Cluster Size)：16 KB */
#define SD_DMA_BUFFER_PREFERRED_BYTES (32U * 1024U)         /* SD 寫入 DMA 暫存區偏好大小：32 KB (位於內部高速 SRAM) */
#define SD_DMA_BUFFER_MIN_BYTES       (8U * 1024U)          /* 記憶體不足時的最低容許 DMA 暫存大小：8 KB */

/*
 * --- 佇列容量與錄影規格 ---
 * 總共有 8 個 framebuffer，其中：
 * - 1 個正在被 writer_task 寫入 SD 卡
 * - 1 個為相機硬體 DMA 正在接收的活躍緩衝區
 * - 1 個為下一個 VSYNC 預備的備用緩衝區
 * 因此佇列最多只能排隊容納 8 - 3 = 5 個已完成的影格指標，避免底層相機硬體無可用緩衝區。
 */
#define FRAME_QUEUE_CAPACITY          (CAMERA_FRAME_BUFFERS - 3U) /* 佇列容量：5 */
#define RECORD_DURATION_SECONDS       10U                   /* 正式錄影長度：10 秒 */
#define RECORD_EXPECTED_FRAMES        (RECORD_DURATION_SECONDS * TARGET_FPS) /* 正式錄影預期總幀數：50 幀 */
#define RECORD_FILE_BYTES             ((uint64_t)RECORD_EXPECTED_FRAMES * IMAGE_PIXELS) /* 預先配置檔案大小：50 * 307200 = 15,360,000 Bytes (約 14.65 MB) */
#define RECORD_MAX_SEQUENCE           10000000U             /* 檔案序號上限 (R0000000.RAW ~ R9999999.RAW) */
#define RECORD_PATH_BYTES             32U                   /* 檔案路徑字串緩衝區大小 */
#define RECORD_QUEUE_WAIT_MS          50U                   /* 寫入任務等待佇列影格的逾時時間 (毫秒) */

/* --- FreeRTOS 任務堆疊與優先權設定 --- */
#define CAPTURE_TASK_STACK_BYTES      4096U                 /* capture_task 堆疊大小：4 KB */
#define WRITER_TASK_STACK_BYTES       4096U                 /* writer_task 堆疊大小：4 KB */
#define CAPTURE_TASK_PRIORITY         6U                    /* capture_task 優先權 (優先權較高，確保硬體影格及時提取) */
#define WRITER_TASK_PRIORITY          5U                    /* writer_task 優先權 (優先權略低，順應 SD 卡寫入速度) */

/* CPU 核心綁定 (SMP 對稱多核心架構) */
#if CONFIG_FREERTOS_UNICORE
#define CAPTURE_TASK_CORE             0                     /* 單核心晶片綁定於 Core 0 */
#define WRITER_TASK_CORE              0
#else
#define CAPTURE_TASK_CORE             1                     /* 雙核心晶片：Core 1 負責影像接收 (避免被 Wi-Fi/系統中斷干擾) */
#define WRITER_TASK_CORE              0                     /* Core 0 負責較慢的 SDMMC 檔案寫入 */
#endif

/* 錄影同步事件旗標 (EventGroup Bits) */
#define CAPTURE_DONE_BIT              BIT0                  /* 旗標：影像擷取任務已完成 */
#define WRITER_DONE_BIT               BIT1                  /* 旗標：SD 卡寫入任務已完成 */

/* --- 測試發起按鈕 (GPIO 0 / Boot Button) --- */
#define TEST_BUTTON_GPIO         GPIO_NUM_0                 /* 開發板上的 Boot 按鍵腳位 (按下去會拉低至 GND) */
#define BUTTON_POLL_MS          10U                         /* 按鍵輪詢間隔：10 毫秒 */
#define BUTTON_DEBOUNCE_MS      30U                         /* 軟體去彈跳延遲時間：30 毫秒 */

/* ============================================================================
 * 感測器專屬暫存器定義與除頻計算 (Sensor Register Maps)
 * ============================================================================ */

/* --- OV7725 暫存器與位元遮罩 --- */
#define OV7725_REG_COM3             0x000CU                 /* 控制暫存器 3：設定 YUV 輸出順序 */
#define OV7725_REG_COM4             0x000DU                 /* 控制暫存器 4：PLL 鎖相環控制 */
#define OV7725_REG_COM5             0x000EU                 /* 控制暫存器 5：自動影格率調節控制 */
#define OV7725_REG_CLKRC            0x0011U                 /* 內部時脈控制暫存器 (除頻器) */
#define OMNIVISION_REG_COM8         0x0013U                 /* 自動曝光/增益/白平衡控制暫存器 */
#define OV7725_REG_DSP_CTRL3        0x0066U                 /* DSP 控制暫存器 3 */
#define OV7725_COM3_SWAP_YUV        0x10U                   /* COM3 第 4 位元：反轉 YUV 輸出順序 (確保第 0 位元組為 Y 亮度訊號) */
#define OV7725_COM4_PLL_MASK        0xC0U                   /* COM4 PLL 遮罩 */
#define OV7725_COM5_AUTO_FPS        0x80U                   /* COM5 第 7 位元：低光源自動降頻開關 (必須關閉以維持固定 5 FPS) */
#define OV7725_CLKRC_DIV_MASK       0x3FU                   /* CLKRC 除頻遮罩 */
#define OV7725_CLKRC_DIVIDER        3U                      /* 除頻值：設定除以 4，將原始時脈減半 */
#define OMNIVISION_COM8_AUTO_MASK   0x07U                   /* COM8 自動功能遮罩 (AGC, AEC, AWB) */

/* --- OV2640 暫存器與位元遮罩 (第 8 位元編碼 Bank 0 或 Bank 1) --- */
#define OV2640_DSP_R_BYPASS         0x0005U                 /* DSP 旁路控制暫存器 */
#define OV2640_DSP_ZMOW             0x005AU                 /* 縮放輸出寬度低位元組 */
#define OV2640_DSP_ZMOH             0x005BU                 /* 縮放輸出高度低位元組 */
#define OV2640_DSP_ZMHH             0x005CU                 /* 縮放輸出高階位元組 */
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
 * @brief 零拷貝佇列元素結構體 (Ready Frame)
 * 在 capture_task 與 writer_task 之間傳遞，僅傳遞指標與中繼資料，不複製 300KB 的像素資料。
 */
typedef struct {
    camera_fb_t *frame;                 /* 指向官方驅動 PSRAM 中的影格緩衝區指標 */
    uint32_t frame_id;                  /* 本次錄影內的影格流水序號 (0, 1, 2...) */
    int64_t timestamp_us;               /* 該影格採集完成時的系統微秒時戳 */
} ready_frame_t;

/**
 * @brief 完整錄影品質監控統計結構體
 * 紀錄每次 10 秒錄影過程中的每一個關鍵指標，供健康診斷與除錯分析。
 */
typedef struct {
    uint32_t get_calls;                 /* 嘗試讀取相機的次數 */
    uint32_t in_window_frames;          /* 落在 10 秒合法時間視窗內的影格數 */
    uint32_t enqueued_frames;           /* 成功推入 FreeRTOS 佇列的影格數 */
    uint32_t saved_frames;              /* 成功完整寫入 SD 卡的影格數 */
    uint32_t stale_frames;              /* 丟棄的歷史過期影格數 */
    uint32_t invalid_frames;            /* 異常損毀影格數 */
    uint32_t timestamp_errors;          /* 時戳錯誤次數 */
    uint32_t capture_timeouts;          /* 擷取逾時次數 */
    uint32_t pool_overflows;            /* 佇列滿載導致掉幀 (Drop) 的次數 */
    uint32_t queue_errors;              /* 佇列操作錯誤次數 */
    uint32_t frame_gaps;                /* 掉幀事件計數 */
    uint32_t peak_ready_slots;          /* 佇列最高積壓深度 (歷史峰值，最高 5) */
    int64_t first_frame_us;             /* 第一幀時戳 (微秒) */
    int64_t last_frame_us;              /* 最後一幀時戳 (微秒) */
    int64_t min_interval_us;            /* 影格間最小間隔 (微秒) */
    int64_t max_interval_us;            /* 影格間最大間隔 (微秒) */
    int64_t max_sd_chunk_us;            /* SD 卡寫入單次 32KB 區塊的最長耗時 (微秒) */
    int64_t max_sd_frame_us;            /* SD 卡寫入完整單幀 (300KB) 的最長耗時 (微秒) */
    int64_t capture_end_us;             /* 擷取任務結束時戳 (微秒) */
    int64_t writer_end_us;              /* 寫入任務結束時戳 (微秒) */
    esp_err_t capture_result;           /* 擷取端最終執行結果狀態碼 */
    esp_err_t writer_result;            /* 寫入端最終執行結果狀態碼 */
} recording_stats_t;

/**
 * @brief 當前錄影全域上下文結構體
 */
typedef struct {
    int fd;                             /* 當前開啟的 SD 卡檔案描述符 (File Descriptor) */
    int64_t trigger_us;                 /* 錄影開始觸發的微秒時間點 */
    int64_t deadline_us;                /* 錄影預計結束的截止微秒時間點 (trigger + 10s) */
    recording_stats_t stats;            /* 本次錄影統計資訊實體 */
} recording_context_t;

/* 結構體與資料長度二度驗證 (確保二進位格式完全對齊 SD 扇區) */
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

/* ============================================================================
 * 全域與靜態變數 (Global Variables)
 * ============================================================================ */

static const char *TAG = "cam_record";       /* 系統日誌輸出標籤 */
static sdmmc_card_t *s_sd_card = NULL;       /* SD 卡控制器控制代碼 */
static uint8_t *s_sd_dma_buffer = NULL;      /* 位於內部 SRAM 的 32KB DMA 寫入暫存區指標 */
static size_t s_sd_dma_buffer_bytes = 0;     /* 實際成功分配的 DMA 暫存區大小 (位元組) */

/* 靜態記憶體配置的 FreeRTOS 佇列 (避免動態 malloc 造成記憶體碎裂) */
static StaticQueue_t s_ready_queue_control;
static uint8_t s_ready_queue_storage[FRAME_QUEUE_CAPACITY * sizeof(ready_frame_t)];
static QueueHandle_t s_ready_queue = NULL;

/* 靜態記憶體配置的 FreeRTOS 事件旗標群組 */
static StaticEventGroup_t s_record_events_control;
static EventGroupHandle_t s_record_events = NULL;

/* 當前錄影作業運行環境實體 */
static recording_context_t s_recording;

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
     * 感測器硬體輸出 YUV422 格式，ESP32 內部 I2S/DVP DMA 在接收時
     * 會自動捨棄 UV 色度訊號，僅保留 Y 亮度訊號存入緩衝區，得到完美的 GRAY8 灰階影像。
     */
    .pixel_format = PIXFORMAT_GRAYSCALE,    /* 灰階模式 (GRAY8) */
    .frame_size = FRAMESIZE_VGA,            /* 640x480 解析度 */
    .jpeg_quality = 12,                     /* 非 JPEG 模式下此參數不作用 */
    
    /*
     * 緩衝區數量與位置：
     * 配置 8 個緩衝區於外部 PSRAM 中。
     * writer_task 在完成 SD 寫入前會一直持有該緩衝區指標，
     * 寫入完畢後呼叫 return，使硬體能循環重用這 8 個緩衝區。
     */
    .fb_count = CAMERA_FRAME_BUFFERS,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,        /* 閒置等待觸發時，只保留最新完成的一幀，其餘自動丟棄 */
};

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
 * @brief 設定 OV7725 感測器：鎖定固定 5 FPS 並確保灰階位元組順序
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

    /* 回讀暫存器，確保寫入生效 */
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
 * @brief 設定 OV2640 感測器：設定 DSP 縮放、DVP 輸出除頻與 5 FPS 時脈樹
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

    /* 回讀暫存器並完整驗證 */
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

/* ============================================================================
 * SD 卡與底層管線初始化 (SDMMC and Buffer Management)
 * ============================================================================ */

/**
 * @brief 迴圈安全寫入工具函式：確保將指定長度資料完整寫入檔案描述符 (fd)
 * @param fd 檔案描述符
 * @param data 資料指標
 * @param bytes 欲寫入位元組數
 */
static esp_err_t write_all(int fd, const uint8_t *data, size_t bytes)
{
    size_t offset = 0;
    while (offset < bytes) {
        const ssize_t written = write(fd, data + offset, bytes - offset);
        if (written < 0) {
            if (errno == EINTR) {
                continue;   /* 收到中斷訊號，立即重試 */
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
 * @brief 初始化 SD 卡硬體 (SDMMC 1-bit 高速 40MHz 模式) 並掛載 FAT 檔案系統
 */
static esp_err_t init_sdcard(void)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();

    /*
     * TTGO T8 V1.7 / T-Camera 開發板 SDMMC 腳位定義：
     * CLK=GPIO14, CMD=GPIO15, D0=GPIO2
     * 啟用 40 MHz 高速傳輸模式，VGA GRAY8 @ 5fps 資料流約 1.54 MB/s，
     * 40 MHz 1-bit 模式提供超過 4 MB/s 頻寬，具備充足傳輸餘裕。
     */
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    slot.width = 1;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    const esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,                    /* 掛載失敗不自動格式化，防止資料誤刪 */
        .max_files = SD_MAX_OPEN_FILES,
        .allocation_unit_size = SD_FORMAT_ALLOCATION_BYTES,
    };

    ESP_RETURN_ON_ERROR(
        esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot, &mount, &s_sd_card),
        TAG, "SDMMC 1-bit 40 MHz mount failed");

    sdmmc_card_print_info(stdout, s_sd_card);               /* 終端機印出 SD 卡容量與製造商資訊 */
    return ESP_OK;
}

/**
 * @brief 在 ESP32 內部 SRAM 配置 4 位元組對齊的 DMA 暫存區
 * 說明：外部 PSRAM 無法直接供 SDMMC DMA 高效傳輸，故必須透過內部 SRAM 暫存中轉。
 */
static esp_err_t allocate_sd_dma_buffer(void)
{
    static const size_t candidate_sizes[] = {
        SD_DMA_BUFFER_PREFERRED_BYTES,  /* 優先嘗試 32 KB */
        16U * 1024U,                    /* 次選 16 KB */
        SD_DMA_BUFFER_MIN_BYTES,        /* 最低容許 8 KB */
    };

    for (size_t i = 0; i < sizeof(candidate_sizes) / sizeof(candidate_sizes[0]); ++i) {
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

/**
 * @brief 釋放錄影管線所佔用的 DMA 緩衝區與 FreeRTOS 物件
 */
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

/**
 * @brief 初始化錄影管線 (配置 DMA 暫存區、靜態佇列與事件旗標)
 */
static esp_err_t init_record_pipeline(void)
{
    ESP_RETURN_ON_ERROR(allocate_sd_dma_buffer(), TAG,
                        "SD DMA staging allocation failed");

    /* 使用 Static 靜態配置，確保運行期間不發生記憶體碎裂 */
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

/**
 * @brief 重置錄影佇列狀態：清空滯留影格並將其歸還相機驅動 (避免記憶體洩漏)
 */
static esp_err_t reset_record_queues(void)
{
    ready_frame_t pending = {0};
    while (xQueueReceive(s_ready_queue, &pending, 0) == pdTRUE) {
        if (pending.frame != NULL) {
            esp_camera_fb_return(pending.frame);    /* 歸還尚未處理的影格緩衝區 */
        }
    }
    xQueueReset(s_ready_queue);
    xEventGroupClearBits(s_record_events, CAPTURE_DONE_BIT | WRITER_DONE_BIT);
    return ESP_OK;
}

/**
 * @brief 預先配置 RAW 檔案大小 (核心效能優化關鍵)
 * 說明：
 * 在錄影前直接將檔案指標移至 15MB 結尾處寫入 1 位元組並同步回磁區，
 * 促使 FAT 檔案系統一次性配置連續簇 (Clusters)，後續錄影寫入完全零延遲。
 */
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

/**
 * @brief 建立並開啟下一個序號的 RAW 檔案 (如 /sdcard/R0000000.RAW)
 */
static esp_err_t open_next_raw_file(char *path, size_t path_bytes, int *fd_out)
{
    for (uint32_t id = 0; id < RECORD_MAX_SEQUENCE; ++id) {
        const int path_length = snprintf(path, path_bytes,
                                         SD_MOUNT_POINT "/R%07u.RAW",
                                         (unsigned)id);
        if (path_length < 0 || (size_t)path_length >= path_bytes) {
            return ESP_ERR_INVALID_SIZE;
        }

        /* 使用 O_EXCL：若檔案已存在則返回錯誤，自動尋找下一個未使用的檔名 */
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

/**
 * @brief 分段將一整幀 (300KB) 從外部 PSRAM 複製到內部 DMA 暫存區，再寫入 SD 卡
 */
static esp_err_t write_frame_to_sd(int fd, const uint8_t *pixels, int64_t *max_chunk_us)
{
    size_t offset = 0;
    while (offset < IMAGE_PIXELS) {
        const size_t remaining = IMAGE_PIXELS - offset;
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
 * @brief 初始化測試觸發按鈕 (GPIO 0 / Boot 按鍵)
 */
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

/**
 * @brief 阻塞等待使用者按下並放開 GPIO 0 按鍵以觸發錄影
 * @return 觸發發生的微秒時間點
 */
static int64_t wait_for_record_trigger(void)
{
    ESP_LOGI(TAG,
             "Press and release GPIO0 to record %u seconds "
             "(expected about %u GRAY8 frames)",
             (unsigned)RECORD_DURATION_SECONDS,
             (unsigned)RECORD_EXPECTED_FRAMES);

    for (;;) {
        /* 等待按鍵按下 (低電位) */
        while (gpio_get_level(TEST_BUTTON_GPIO) != 0) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
        if (gpio_get_level(TEST_BUTTON_GPIO) != 0) {
            continue;
        }

        /* 等待按鍵釋放 (高電位) */
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
 * 雙核心並行錄影任務實作 (Capture and Writer FreeRTOS Tasks)
 * ============================================================================ */

/**
 * @brief 影像擷取任務 (Capture Task)
 * 運行於：CPU Core 1，高優先權 (Priority 6)
 * 職責：
 * 從相機驅動讀取影格，嚴格審核時戳與格式，並封裝成 ready_frame_t 丟入佇列。
 * 本任務絕對不執行阻塞的檔案操作，確保相機硬體 FIFO 隨時能被及時清空。
 */
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

        /* 濾除觸發前的歷史殘留影格 */
        if (timestamp_us < s_recording.trigger_us) {
            stats->stale_frames++;
            esp_camera_fb_return(frame);
            continue;
        }
        
        /* 達到 10 秒錄影截止時間點，正常結束採集 */
        if (timestamp_us >= s_recording.deadline_us) {
            esp_camera_fb_return(frame);
            break;
        }

        /* 統計影格時間間隔 */
        int64_t interval_us = 0;
        if (stats->in_window_frames == 0U) {
            stats->first_frame_us = timestamp_us;
        } else {
            interval_us = timestamp_us - stats->last_frame_us;
            if (stats->in_window_frames == 1U || interval_us < stats->min_interval_us) {
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

        /* 將影格指標推入 FreeRTOS 佇列 (非阻塞呼叫) */
        if (xQueueSend(s_ready_queue, &ready_frame, 0) != pdTRUE) {
            /* 若佇列已滿 (代表 SD 卡寫入不及)，被迫放棄此影格 (Drop) 並歸還硬體 */
            stats->pool_overflows++;
            esp_camera_fb_return(frame);
            if (stats->pool_overflows == 1U || (stats->pool_overflows % 10U) == 0U) {
                ESP_LOGW(TAG,
                         "Ready queue full at frame %u: dropped=%u ready=%u/%u",
                         (unsigned)frame_id,
                         (unsigned)stats->pool_overflows,
                         (unsigned)uxQueueMessagesWaiting(s_ready_queue),
                         (unsigned)FRAME_QUEUE_CAPACITY);
            }
            continue;
        }

        /* 成功排隊：此緩衝區所有權暫時轉移給 writer_task */
        stats->enqueued_frames++;
        const uint32_t ready = uxQueueMessagesWaiting(s_ready_queue);
        if (ready > stats->peak_ready_slots) {
            stats->peak_ready_slots = ready;               /* 紀錄佇列最高水位 */
        }
    }

    stats->capture_end_us = esp_timer_get_time();
    xEventGroupSetBits(s_record_events, CAPTURE_DONE_BIT);  /* 通知系統：擷取任務已結束 */
    vTaskDelete(NULL);                                      /* 自我刪除任務 */
}

/**
 * @brief SD 卡寫入任務 (Writer Task)
 * 運行於：CPU Core 0，優先權 (Priority 5)
 * 職責：
 * 從佇列取出 ready_frame_t，將 300KB 資料分段透過 DMA 寫入 SD 卡。
 * 寫入完成後，負責呼叫 esp_camera_fb_return() 將緩衝區釋放回相機驅動。
 */
static void writer_task(void *argument)
{
    (void)argument;
    recording_stats_t *stats = &s_recording.stats;

    for (;;) {
        ready_frame_t ready_frame = {0};
        /* 等待佇列提供已完成影格，逾時時間 50 毫秒 */
        if (xQueueReceive(s_ready_queue, &ready_frame,
                          pdMS_TO_TICKS(RECORD_QUEUE_WAIT_MS)) == pdTRUE) {
            if (stats->writer_result == ESP_OK) {
                const int64_t frame_write_start_us = esp_timer_get_time();
                
                /* 呼叫分段寫入函式寫入 SD 卡 */
                const esp_err_t err = write_frame_to_sd(
                    s_recording.fd, ready_frame.frame->buf,
                    &stats->max_sd_chunk_us);
                
                const int64_t frame_write_us = esp_timer_get_time() - frame_write_start_us;
                if (frame_write_us > stats->max_sd_frame_us) {
                    stats->max_sd_frame_us = frame_write_us;/* 紀錄單幀最長寫入耗時 */
                }
                if (err == ESP_OK) {
                    stats->saved_frames++;
                    if ((stats->saved_frames % 10U) == 0U) {
                        ESP_LOGI(TAG,
                                 "Saved %u frames; ready=%u/%u",
                                 (unsigned)stats->saved_frames,
                                 (unsigned)uxQueueMessagesWaiting(s_ready_queue),
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

            /* 【關鍵零拷貝節點】：寫入完成後，正式歸還緩衝區供硬體下一輪使用 */
            if (ready_frame.frame != NULL) {
                esp_camera_fb_return(ready_frame.frame);
            }
            continue;
        }

        /* 當 capture_task 已完成且佇列中的影格已全部清空寫畢時，退出迴圈 */
        if ((xEventGroupGetBits(s_record_events) & CAPTURE_DONE_BIT) != 0 &&
            uxQueueMessagesWaiting(s_ready_queue) == 0U) {
            break;
        }
    }

    /* 錄影結束收尾：根據實際寫入的幀數截斷 (ftruncate) 預先配置的多餘空間並同步 (fsync) */
    const off_t saved_bytes = (off_t)((uint64_t)stats->saved_frames * IMAGE_PIXELS);
    if (ftruncate(s_recording.fd, saved_bytes) != 0 || fsync(s_recording.fd) != 0) {
        if (stats->writer_result == ESP_OK) {
            stats->writer_result = ESP_FAIL;
        }
        ESP_LOGE(TAG, "RAW finalize failed: errno=%d (%s)",
                 errno, strerror(errno));
    }

    stats->writer_end_us = esp_timer_get_time();
    xEventGroupSetBits(s_record_events, WRITER_DONE_BIT);   /* 通知系統：寫入任務已結束 */
    vTaskDelete(NULL);                                      /* 自我刪除任務 */
}

/**
 * @brief 協調整合單次完整 10 秒錄影程序
 */
static esp_err_t record_raw_sequence(void)
{
    char path[RECORD_PATH_BYTES];
    int fd = -1;
    
    /* 1. 建立並預先配置新檔案 */
    ESP_RETURN_ON_ERROR(open_next_raw_file(path, sizeof(path), &fd),
                        TAG, "Cannot prepare RAW recording file");

    /* 2. 重置佇列與旗標狀態 */
    esp_err_t err = reset_record_queues();
    if (err != ESP_OK) {
        close(fd);
        unlink(path);
        return err;
    }

    /* 3. 設定本次錄影時間參數 */
    memset(&s_recording, 0, sizeof(s_recording));
    s_recording.fd = fd;
    s_recording.stats.capture_result = ESP_OK;
    s_recording.stats.writer_result = ESP_OK;
    s_recording.trigger_us = wait_for_record_trigger();     /* 等待使用者按鍵觸發 */
    s_recording.deadline_us = s_recording.trigger_us +
                              (int64_t)RECORD_DURATION_SECONDS * 1000000LL;

    ESP_LOGI(TAG,
             "Recording to %s: timestamp window=%u s, expected~%u frames, "
             "camera_fb=%u ready_queue=%u",
             path, (unsigned)RECORD_DURATION_SECONDS,
             (unsigned)RECORD_EXPECTED_FRAMES,
             (unsigned)CAMERA_FRAME_BUFFERS,
             (unsigned)FRAME_QUEUE_CAPACITY);

    /* 4. 在 CPU 0 啟動 SD 卡寫入任務 */
    if (xTaskCreatePinnedToCore(writer_task, "sd_writer",
                                WRITER_TASK_STACK_BYTES, NULL,
                                WRITER_TASK_PRIORITY, NULL,
                                WRITER_TASK_CORE) != pdPASS) {
        close(fd);
        unlink(path);
        return ESP_ERR_NO_MEM;
    }

    /* 5. 在 CPU 1 啟動相機擷取任務 */
    if (xTaskCreatePinnedToCore(capture_task, "camera_capture",
                                CAPTURE_TASK_STACK_BYTES, NULL,
                                CAPTURE_TASK_PRIORITY, NULL,
                                CAPTURE_TASK_CORE) != pdPASS) {
        s_recording.stats.capture_result = ESP_ERR_NO_MEM;
        xEventGroupSetBits(s_record_events, CAPTURE_DONE_BIT);
    }

    /* 6. 阻塞等待兩大任務雙雙回報完成 (CAPTURE_DONE & WRITER_DONE) */
    xEventGroupWaitBits(s_record_events,
                        CAPTURE_DONE_BIT | WRITER_DONE_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    /* 7. 關閉檔案 */
    if (close(fd) != 0 && s_recording.stats.writer_result == ESP_OK) {
        s_recording.stats.writer_result = ESP_FAIL;
    }

    /* 8. 輸出完整效能報告與統計日誌 */
    recording_stats_t *stats = &s_recording.stats;
    const int64_t timestamp_span_us =
        stats->in_window_frames > 1U
            ? stats->last_frame_us - stats->first_frame_us
            : 0;
    const uint32_t timestamp_milli_fps =
        stats->in_window_frames > 1U && timestamp_span_us > 0
            ? (uint32_t)(((uint64_t)(stats->in_window_frames - 1U) * 1000000000ULL) /
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

    /* 9. 嚴格品質檢核：任一錯誤或掉幀即回傳失敗 */
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

/* ============================================================================
 * 應用程式進入點 (Application Entry Point)
 * ============================================================================ */

/**
 * @brief ESP32 系統開機主程式進入點
 */
void app_main(void)
{
    /* 步驟 1：相機感測器與外部 PSRAM 初始化 */
    sensor_t *sensor = NULL;
    esp_err_t err = init_camera(&sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(err));
        return;
    }

    /* 步驟 2：開啟自動曝光/白平衡，準備進行暖機 */
    err = set_auto_controls(sensor, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot enable camera auto controls: %s", esp_err_to_name(err));
        esp_camera_deinit();
        return;
    }

    /* 步驟 3：丟棄前 10 幀，讓感測器自動曝光收斂到適當明暗度 */
    err = discard_warmup_frames(sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera warm-up failed: %s", esp_err_to_name(err));
        esp_camera_deinit();
        return;
    }

    /* 步驟 4：鎖定曝光與白平衡，防止錄影期間畫面閃爍變色 */
    err = set_auto_controls(sensor, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot lock camera controls: %s", esp_err_to_name(err));
        esp_camera_deinit();
        return;
    }

    /* 步驟 5：執行開機 10 秒 5 FPS 資格驗證測試 (未掛載 SD 卡狀態下驗證硬體純度) */
    err = run_startup_fps_test();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Startup FPS test failed; SD capture remains disabled");
        esp_camera_deinit();
        return;
    }

    /* 步驟 6：初始化 SD 卡控制器並掛載 /sdcard FAT 檔案系統 */
    err = init_sdcard();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD initialization failed: %s", esp_err_to_name(err));
        esp_camera_deinit();
        return;
    }

    /* 步驟 7：建立錄影管線 (內部 SRAM DMA 暫存區、靜態佇列與事件群組) */
    err = init_record_pipeline();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Recording pipeline initialization failed: %s", esp_err_to_name(err));
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_sd_card);
        s_sd_card = NULL;
        esp_camera_deinit();
        return;
    }

    /* 步驟 8：設定 GPIO 0 按鍵輸入 */
    init_record_button();

    /* 步驟 9：進入無窮迴圈，等待按鈕觸發進行 10 秒錄影 */
    while (true) {
        err = record_raw_sequence();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Recording completed with error: %s", esp_err_to_name(err));
        }
    }
}