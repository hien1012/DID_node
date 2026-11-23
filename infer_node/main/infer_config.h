#pragma once

/*
 * 推論韌體的設定 (ESP32 / TTGO T8 版，由 DID_NODE/infer_node/main/infer_config.h 移植)。
 * Wi-Fi、server 位址、heartbeat 週期等沿用錄影韌體的 main/app_config.h (唯一來源)，
 * 這裡只放推論專屬的。腳位在 board_pins.h。
 */
#include "app_config.h"

/*
 * 這塊板子的功能：一個節點只跑一種，燒錄前在這裡選。
 * 另一條管線完全不初始化 (不啟動 I2S 時脈 / 相機)，模型也不會被連結進韌體。
 */
#define INFER_ROLE_CAM                1
#define INFER_ROLE_MIC                2
#ifndef INFER_NODE_ROLE
#define INFER_NODE_ROLE               INFER_ROLE_CAM
#endif

#if INFER_NODE_ROLE != INFER_ROLE_CAM && INFER_NODE_ROLE != INFER_ROLE_MIC
#error "INFER_NODE_ROLE must be INFER_ROLE_CAM or INFER_ROLE_MIC"
#endif

#define INFER_CAM_ENABLE              (INFER_NODE_ROLE == INFER_ROLE_CAM)
#define INFER_MIC_ENABLE              (INFER_NODE_ROLE == INFER_ROLE_MIC)

/*
 * Device ID = 前綴 + "-" + Wi-Fi MAC (12 hex)，例如 e32cam-24A1B2C3D4E5。
 * 同一台 server 同時接 S3 (node-s3-...) 與多個 ESP32 節點時，儀表板靠前綴分辨。
 * device_identity.c 的緩衝是 24 bytes：前綴最多 10 字元。
 */
#if INFER_CAM_ENABLE
#define INFER_DEVICE_ID_PREFIX        "e32cam"
#else
#define INFER_DEVICE_ID_PREFIX        "e32mic"
#endif
_Static_assert(sizeof(INFER_DEVICE_ID_PREFIX) - 1U <= 10U, "Device ID prefix is too long");

/* 預測類別的機率低於此值 = 低信賴度，原始資料存 SD。 */
#define INFER_LOW_CONF_THRESHOLD      0.0f

/*
 * 除了低信賴度，預測為「非背景」的也存 SD 並上傳 (audio：drone / helicopter；video：drone / bird)。
 *   0：只存低信賴度 (原本的行為)。
 *   1：低信賴度 或 非背景，任一成立就存。video 只要一幀內有一個 ROI 非背景就存整幀。
 * 偵測多時樣本量會大增 (video 每幀約 300 KB)；待上傳數達 INFER_MAX_PENDING_SAMPLES 後不再存新的。
 * 背景類別在開機時以名稱 INFER_BACKGROUND_LABEL 從模型的類別表找出 (不寫死索引：
 * audio 的 background 是 0，video 是 2)；找不到時這個開關無效，只存低信賴度。
 * 樣本 meta 的 save_reason 記錄為什麼存："low_conf" / "non_background" / "both"。
 */
#ifndef INFER_SAVE_NON_BACKGROUND
#define INFER_SAVE_NON_BACKGROUND     0
#endif
#define INFER_BACKGROUND_LABEL        "background"

/* 同一種 media 累積這麼多個低信賴樣本才上傳 (音訊、影像分開計算)。 */
#define INFER_UPLOAD_AFTER_SAMPLES    20U
/* SD 上待上傳樣本的上限，超過就不再存新的 (例如長期連不到 server)。 */
#define INFER_MAX_PENDING_SAMPLES     500U

/* 重新校時的週期。 */
#define INFER_CLOCK_RESYNC_SEC        (5U * 60U)

#define INFER_SERVER_RESULTS_URL      APP_SERVER_BASE_URL "/infer/results"
#define INFER_SERVER_SAMPLE_URL       APP_SERVER_BASE_URL "/infer/sample"

/*
 * 低信賴影像樣本的全幀存成什麼格式。感測器 → FD → CNN 的路徑永遠是 raw GRAY8 640x480，
 * 不受這個開關影響；這裡只決定寫進 SD 並回傳的影像。
 *   0：raw GRAY8 (307,200 B)，就是推論看到的資料。
 *   1：由 fb 以 CPU 編成 JPEG 再存。檔案小很多，但會失真，cam_writer 每幀多花編碼時間，
 *      也多用約 14 KB 內部 RAM (encoder 在 stack 上、掃描線緩衝)。
 */
#define INFER_CAM_SAMPLE_JPEG         0
#define INFER_CAM_JPEG_QUALITY        80U   /* 1..100，越大越清晰 */

/*
 * 每個 task 的 CPU 占比統計 (task_usage.c：TASK_USAGE log 與回傳 server 的 task_usage 記錄)。
 *   1：開啟。每次推論前後、相機每 10 幀各呼叫一次 uxTaskGetSystemState()。
 *   0：關閉，上面的函式都變成空的，不印 TASK_USAGE、不回傳 task_usage 記錄。
 * uxTaskGetSystemState() 在 IDF 5.1 雙核心下會關閉呼叫端核心的中斷並拿住 kernel lock，
 * 再逐 byte 掃描所有 task 的剩餘 stack (約 1–2 ms)。相機 DMA 只有 2 個半區 (各 3 行、約 1.2 ms)，
 * 中斷被擋住時兩次 EOF 會合併成一次，那幀就少了 3 行而被丟掉 (Camera: incomplete)。
 * cam_infer 每次推論的 run / preempt / fd_overlap 不受影響 (只讀自己的計數器，不掃 stack)。
 */
#ifndef INFER_TASK_USAGE_ENABLE
#define INFER_TASK_USAGE_ENABLE       0
#endif

/* cam 模型新舊版切換 (A/B 測試)：vision/cam_model.h 的 CAM_MODEL_USE_OLD (cam_model.cc 是 C++，不 include 這個檔) */

/* 每幀最多送幾個 ROI 進 CNN，避免一張雜訊幀佔住 fb 太久；超過的留到該 track 下一次配對。 */
#define INFER_CAM_MAX_ROI_PER_FRAME   4U

/*
 * 暖機後鎖定 AE/AGC/AWB (1，與錄影韌體相同) 或維持自動 (0)。
 * 降頻 / 虛擬行在 configure_sensor() 另外關閉，兩種設定都維持固定 5 fps。
 */
#ifndef APP_CAM_LOCK_AUTO_CONTROLS
#define APP_CAM_LOCK_AUTO_CONTROLS    1
#endif

/*
 * 麥克風 PCM 的取法 (模型輸入合約的一部分，存下的 WAV 就是推論用的那份資料)：
 *   0 (預設)：TTGO_I2S_Record_Test 格式，訓練資料就是這樣錄的。I2S 與 PDM 都讀 32-bit slot，
 *             (int16)(raw32 >> 16) × 30 後飽和。
 *   1：commit 8ddab23 (main/audio_record.c) 的 PDM 格式，只在 PDM 有效：讀 16-bit slot
 *      (不移位)，增益前加一階高通 (-3 dB @ MIC_PDM_HPF_CUTOFF_HZ)，再 × 30 飽和。
 *      ESP32 的 PDM RX 沒有硬體高通，DC 與 1 Hz 以下的漂移會被 × 30 放大到削波。
 *      和訓練資料的取法不同，切換後要確認模型輸出是否合理。
 * I2S 麥克風兩種設定相同 (32-bit >> 16，不濾波)。
 */
#ifndef MIC_PDM_16BIT_HPF
#define MIC_PDM_16BIT_HPF             1
#endif
#define MIC_PDM_HPF_CUTOFF_HZ         24.0f /* 對齊 ICS-43434 內建高通 (DS-000069 p.14) */
