#pragma once

/*
 * 推論韌體的設定。Wi-Fi、server 位址、heartbeat 週期等沿用 did_node 的
 * main/app_config.h (唯一來源)，這裡只放推論專屬的。
 */
#include "app_config.h"

/*
 * 這塊板子的功能：一個節點只跑一種，燒錄前在這裡選。
 * 另一條管線完全不初始化 (不啟動 I2S 時脈 / 相機)，模型也不會被連結進韌體。
 */
#define INFER_ROLE_CAM                1
#define INFER_ROLE_MIC                2
#define INFER_NODE_ROLE               INFER_ROLE_MIC

#if INFER_NODE_ROLE != INFER_ROLE_CAM && INFER_NODE_ROLE != INFER_ROLE_MIC
#error "INFER_NODE_ROLE must be INFER_ROLE_CAM or INFER_ROLE_MIC"
#endif

#define INFER_CAM_ENABLE              (INFER_NODE_ROLE == INFER_ROLE_CAM)
#define INFER_MIC_ENABLE              (INFER_NODE_ROLE == INFER_ROLE_MIC)

/* 預測類別的機率低於此值 = 低信賴度，原始資料存 SD。 */
#define INFER_LOW_CONF_THRESHOLD      0.5f

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

/* 每幀最多送幾個 ROI 進 CNN，避免一張雜訊幀佔住 fb 太久；超過的留到該 track 下一次配對。 */
#define INFER_CAM_MAX_ROI_PER_FRAME   4U
