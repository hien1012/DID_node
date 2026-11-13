#ifndef FD_MIX_H
#define FD_MIX_H

/*
 * Stage1 幀差偵測 (由 DID_CAM_SEAN/FD_did_cam_v4/main/fd_mix.h 精簡)。
 * 只留相機管線用到的部分；stage2.h 以 "fd_mix.h" 取得 FD_BoundingBox，檔名不能改。
 *
 * 座標公約：FD_BoundingBox 一律 inclusive (x ∈ [x1..x2]、y ∈ [y1..y2])，
 * 寬 = x2-x1+1、高 = y2-y1+1。低解析度 bbox 回推全解析度 (倍率 f)：
 * X1 = x1*f, Y1 = y1*f, X2 = x2*f+(f-1), Y2 = y2*f+(f-1)。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FD_WIDTH 640
#define FD_HEIGHT 480
#define FD_MAX_BBOXES 100

typedef enum {
    FD_MORPH_DILATE = 2,    /* 數值沿用原專案 */
    FD_MORPH_ERODE = 3,
} FD_MorphMode;

typedef struct {
    int thresh;                    /**< 二值化閾值 */
    int bbox_thresh;               /**< 最小連通區面積閾值 */
    float iou_threshold;           /**< 非最大抑制 IoU 閾值 */
    int morph_kernel_size;         /**< 形態學操作核大小 (奇數) */
    FD_MorphMode morph_mode;
    bool roi_refine_enable;        /**< NMS 前用 Sobel + flood fill 精修 bbox */
    int roi_refine_expand_px;      /**< refine 前 bbox 向外擴張像素 */
    int roi_refine_sobel_thresh;   /**< Sobel magnitude 二值化閾值 */
    int roi_refine_min_cc_area;    /**< ROI 內最小連通區面積閾值 */
    bool roi_refine_keep_largest_only; /**< true：只取最大連通區；false：所有候選的最小外接矩形 */
} FD_Parameters;

typedef struct {
    int x1, y1, x2, y2;   /**< inclusive 座標 */
    int area;             /**< 邊界框面積 */
} FD_BoundingBox;

typedef struct {
    FD_BoundingBox* bboxes;
    int num_boxes;
} FD_DetectionResult;

/** FD 各步驟耗時；中途就沒有框時，後面的步驟維持 0 */
typedef struct {
    uint32_t diff_us;     /**< 幀差 + 二值化 */
    uint32_t morph_us;    /**< 形態學 */
    uint32_t contour_us;  /**< flood fill 找框 */
    uint32_t refine_us;   /**< Sobel ROI refine */
    uint32_t nms_us;
} FD_StageTiming;

/**
 * @brief 兩張灰階圖的幀差偵測
 * @param gray1 上一幀，會被覆蓋 (當二值遮罩)
 * @param gray2 當前幀，不改動
 * @param temp  與 gray1 同大小的暫存
 * @param timing 寫入各步驟耗時，不可為 NULL
 * @return 結果 (用 fd_free_detection_result 釋放)；沒有框時回傳 NULL
 */
FD_DetectionResult* fd_process_frames_gray(uint8_t* gray1, uint8_t* gray2, uint8_t* temp,
                                           FD_Parameters* params, int* num_boxes,
                                           int width, int height, FD_StageTiming* timing);
void fd_free_detection_result(FD_DetectionResult* result);

void fd_roi_refine(const uint8_t* gray2, int width, int height, FD_BoundingBox* boxes,
                   int* inout_num_boxes, const FD_Parameters* params);

bool expand_bbox(FD_BoundingBox* bbox, int expand_px, int image_width, int image_height);
bool expand_roi_to_square(FD_BoundingBox* bbox, int image_width, int image_height);

/** resize_mode：0 = nearest，1 = bilinear (原專案相機管線用 1) */
bool crop_resize_bbox(const uint8_t* src_gray, int src_width, int src_height,
                      uint8_t* dst_gray, int dst_size, int resize_mode,
                      const FD_BoundingBox* bbox);

#ifdef __cplusplus
}
#endif

#endif /* FD_MIX_H */
