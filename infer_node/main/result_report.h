#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define INFER_CLASS_COUNT 3U

typedef enum {
    INFER_RECORD_AUDIO = 0,     /* 一個 2 s 音訊視窗的推論結果 */
    INFER_RECORD_CAM_ROI,       /* 一個 ROI 的 CNN 結果 */
    INFER_RECORD_CAM_FRAME,     /* 一幀的處理時間記錄 */
    INFER_RECORD_CAM_SESSION,   /* 一次相機 session 的統計 */
    INFER_RECORD_CAM_OPS,       /* CNN 每一層的平均耗時 */
    INFER_RECORD_TASK_USAGE,    /* 推論一次的期間，每個 task 用掉的 CPU */
} infer_record_type_t;

#define INFER_CNN_OPS_MAX 28U
#define INFER_TASK_USAGE_MAX 24U

typedef struct {
    uint32_t window;            /* 開機以來第幾個視窗 */
    int64_t start_epoch_ms;
    int64_t end_epoch_ms;
    uint32_t queue_us;          /* 視窗收滿 → 開始處理 */
    uint32_t feature_us;
    uint32_t invoke_us;
    uint32_t overflows;         /* 這個視窗期間的 I2S DMA overflow 次數 */
    uint32_t dropped_windows;   /* 開機以來因推論來不及而丟掉的視窗數 */
    uint8_t pred;
    const char *label;
    float conf;
    float probs[INFER_CLASS_COUNT];
    bool low_conf;
} infer_audio_record_t;

typedef struct {
    int64_t session;
    uint32_t frame;
    int64_t ts_epoch_ms;        /* 幀開始 VSYNC */
    int32_t track_id;
    int32_t bbox[4];            /* 全解析度、送進 CNN 的正方形 ROI (inclusive) */
    uint32_t invoke_us;
    uint32_t result_us;         /* 幀開始 VSYNC → 這個 ROI 的 CNN 做完 (節點內，同一個時鐘) */
    uint8_t pred;
    const char *label;
    float conf;
    float probs[INFER_CLASS_COUNT];
    bool low_conf;
} infer_cam_roi_record_t;

typedef struct {
    int64_t session;
    uint32_t frame;
    int64_t ts_epoch_ms;
    uint32_t capture_us;        /* 幀開始 VSYNC → camera_capture 送進佇列 (含感測器讀出) */
    uint32_t queue_us;          /* 送進佇列 → cam_detect 取出 */
    uint32_t downscale_us;
    uint32_t fd_us;             /* 下面五步再加上座標換算 */
    uint32_t fd_diff_us;        /* 幀差 + 二值化 */
    uint32_t fd_morph_us;       /* 形態學膨脹 */
    uint32_t fd_contour_us;     /* flood fill 找框 */
    uint32_t fd_refine_us;      /* Sobel ROI refine */
    uint32_t fd_nms_us;
    uint32_t track_us;
    uint32_t detect_us;         /* cam_detect 處理這幀的時間，超過幀週期就會開始丟幀 */
    uint32_t cnn_wait_us;       /* cam_detect 交出 → cam_infer 開始 (只有含候選的幀) */
    uint32_t roi_us;            /* 所有 ROI 的 crop + CNN + 回報 */
    uint32_t crop_us;           /* 所有 ROI 的裁切縮放合計 */
    uint32_t cnn_us;            /* 所有 ROI 的 CNN 合計 */
    uint32_t total_us;          /* detect_us + roi_us，不含 cnn_wait_us */
    uint16_t boxes;
    uint16_t candidates;
} infer_cam_frame_record_t;

typedef struct {
    int64_t session;
    uint32_t frames;            /* 視窗內收到的幀數 */
    uint32_t processed;
    uint32_t dropped;           /* 佇列滿丟幀 */
    uint32_t gaps;
    uint32_t incomplete;        /* 驅動丟棄的 FB-SIZE 幀 */
    uint32_t rois;
    uint32_t rois_expired;      /* 達到觸發條件，但等不到 CNN 空位 track 就消失 */
    uint32_t low_conf;
    uint32_t saved;
    uint32_t save_skipped;      /* 低信賴但寫入失敗 */
    int32_t first_frame_offset_us;
    int32_t interval_min_us;
    int32_t interval_max_us;
    int32_t error;
    uint32_t save_wait_avg_us;  /* cam_infer 交出 → cam_writer 開始 (啟動以來) */
    uint32_t save_wait_max_us;
    uint32_t save_avg_us;       /* cam_writer 存一個樣本 (編碼 + 寫 SD + meta) */
    uint32_t save_max_us;
} infer_cam_session_record_t;

typedef struct {
    int64_t session;
    uint32_t frame;             /* 回報時已處理到第幾幀 */
    uint32_t invokes;           /* 這些平均值來自幾次推論 */
    uint8_t op_count;
    struct {
        const char *name;
        uint32_t avg_us;
    } ops[INFER_CNN_OPS_MAX];   /* 依執行順序 */
} infer_cam_ops_record_t;

typedef struct {
    const char *media;          /* "video" 或 "audio" */
    int64_t ts_epoch_ms;        /* 回報時間 */
    uint32_t inferences;        /* 這些平均值來自幾次推論 */
    uint32_t avg_wall_us;       /* 平均每次推論經過的時間 */
    uint8_t task_count;
    struct {
        const char *name;
        int8_t core;            /* 0、1；-1 = 不綁核心 */
        uint32_t avg_us;        /* 平均每次推論，這個 task 用掉的 CPU 時間 */
    } tasks[INFER_TASK_USAGE_MAX];  /* 由多到少 */
} infer_task_usage_record_t;

typedef struct {
    infer_record_type_t type;
    union {
        infer_audio_record_t audio;
        infer_cam_roi_record_t roi;
        infer_cam_frame_record_t frame;
        infer_cam_session_record_t session;
        infer_cam_ops_record_t ops;
        infer_task_usage_record_t task_usage;
    } data;
} infer_record_t;

/*
 * 結果立刻回傳 server：result_report task 把佇列內的記錄批次 POST 到
 * /infer/results，所以 server 斷線只卡住這個 task，不影響擷取與推論。
 */
esp_err_t result_report_start(void);

/** 不阻塞；佇列滿時丟棄並計數 (下一次 POST 回報)。 */
void result_report_push(const infer_record_t *record);
uint32_t result_report_dropped(void);
uint32_t result_report_sent(void);
