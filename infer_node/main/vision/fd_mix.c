/*
 * Stage1 幀差偵測與 ROI 前處理。
 * 演算法函式從 DID_CAM_SEAN/FD_did_cam_v4/main/fd_mix.c 原樣搬來，只有兩點不同：
 *  - 所有配置改走 FD_MALLOC / FD_REALLOC (PSRAM)：SPIRAM_MALLOC_ALWAYSINTERNAL=16384
 *    會讓這裡每幀的小型 malloc (flood fill stack 8 KB、bbox 2 KB...) 全進內部 RAM，
 *    那是錄影期間最緊的資源，還會造成碎片。
 *  - 拿掉 SD 測試、JPEG 解碼、RGB 轉換等相機管線用不到的程式；輸入固定是 GRAY8。
 */
#include "fd_mix.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#define FD_TAG "FD_MIX"

#define FD_MALLOC(bytes)       heap_caps_malloc((bytes), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define FD_REALLOC(ptr, bytes) heap_caps_realloc((ptr), (bytes), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

typedef struct {
    uint16_t x, y;
} FD_Point;

static bool fd_flood_fill_optimized(uint8_t* mask, int width, int height, int start_x, int start_y,
                                    int* min_x, int* max_x, int* min_y, int* max_y, int* area);

static void fd_dilation_horizontal(uint8_t* src, uint8_t* dst, int width, int height, int kernel_size) {
    int offset = kernel_size / 2;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint8_t max_val = 0;
            
            // 只在水平方向找最大值
            for (int kx = -offset; kx <= offset; kx++) {
                int px = x + kx;
                
                // 邊界處理：使用邊界複製
                if (px < 0) px = 0;
                if (px >= width) px = width - 1;
                
                uint8_t val = src[y * width + px];
                if (val > max_val) max_val = val;
            }
            
            dst[y * width + x] = max_val;
        }
    }
}

static void fd_dilation_vertical(uint8_t* src, uint8_t* dst, int width, int height, int kernel_size) {
    int offset = kernel_size / 2;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint8_t max_val = 0;
            
            // 只在垂直方向找最大值
            for (int ky = -offset; ky <= offset; ky++) {
                int py = y + ky;
                
                // 邊界處理：使用邊界複製
                if (py < 0) py = 0;
                if (py >= height) py = height - 1;
                
                uint8_t val = src[py * width + x];
                if (val > max_val) max_val = val;
            }
            
            dst[y * width + x] = max_val;
        }
    }
}

static void fd_erosion_horizontal(uint8_t* src, uint8_t* dst, int width, int height, int kernel_size) {
    int offset = kernel_size / 2;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint8_t min_val = 255;
            
            // 只在水平方向找最小值
            for (int kx = -offset; kx <= offset; kx++) {
                int px = x + kx;
                
                // 邊界處理：使用邊界複製
                if (px < 0) px = 0;
                if (px >= width) px = width - 1;
                
                uint8_t val = src[y * width + px];
                if (val < min_val) min_val = val;
            }
            
            dst[y * width + x] = min_val;
        }
    }
}

static void fd_erosion_vertical(uint8_t* src, uint8_t* dst, int width, int height, int kernel_size) {
    int offset = kernel_size / 2;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint8_t min_val = 255;
            
            // 只在垂直方向找最小值
            for (int ky = -offset; ky <= offset; ky++) {
                int py = y + ky;
                
                // 邊界處理：使用邊界複製
                if (py < 0) py = 0;
                if (py >= height) py = height - 1;
                
                uint8_t val = src[py * width + x];
                if (val < min_val) min_val = val;
            }
            
            dst[y * width + x] = min_val;
        }
    }
}

static void fd_apply_morphology(uint8_t* src, uint8_t* dst, uint8_t* temp, int width, int height, FD_Parameters* params) {
    switch (params->morph_mode) {
        case FD_MORPH_ERODE:
            fd_erosion_horizontal(src, temp, width, height, params->morph_kernel_size);
            fd_erosion_vertical(temp, dst, width, height, params->morph_kernel_size);
            break;
        case FD_MORPH_DILATE:
        default:
            fd_dilation_horizontal(src, temp, width, height, params->morph_kernel_size);
            fd_dilation_vertical(temp, dst, width, height, params->morph_kernel_size);
            break;
    }
}

static void get_absdiff(uint8_t* frame1, uint8_t* frame2, uint8_t* diff, int width, int height) {
    for (int i = 0; i < width * height; i++) {
        int d = (int)frame2[i] - (int)frame1[i];  // 計算差值
        diff[i] = (d < 0) ? -d : d;               // 取絕對值，與Python cv2.absdiff一致
    }
}

static void fd_sobel_threshold_roi_3x3(
    const uint8_t* gray,
    int width,
    int height,
    int x1,
    int y1,
    int x2,
    int y2,
    int sobel_thresh,
    uint8_t* out_mask
) {
    int roi_w = x2 - x1 + 1;
    int roi_h = y2 - y1 + 1;
    if (!gray || !out_mask || roi_w <= 0 || roi_h <= 0) return;

    memset(out_mask, 0, (size_t)roi_w * (size_t)roi_h);

    // Sobel kernels (3x3, fixed)
    static const int kx[3][3] = {
        {-1, 0, 1},
        {-2, 0, 2},
        {-1, 0, 1},
    };
    static const int ky[3][3] = {
        {-1, -2, -1},
        { 0,  0,  0},
        { 1,  2,  1},
    };

    for (int gy = y1; gy <= y2; gy++) {
        for (int gx = x1; gx <= x2; gx++) {
            int oy = gy - y1;
            int ox = gx - x1;

            if (gx <= 0 || gx >= width - 1 || gy <= 0 || gy >= height - 1) {
                out_mask[oy * roi_w + ox] = 0;
                continue;
            }

            int sx = 0;
            int sy = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    uint8_t p = gray[(gy + dy) * width + (gx + dx)];
                    sx += (int)p * kx[dy + 1][dx + 1];
                    sy += (int)p * ky[dy + 1][dx + 1];
                }
            }

            int mag = (int)(sqrtf((float)(sx * sx + sy * sy)));
            out_mask[oy * roi_w + ox] = (mag > sobel_thresh) ? 255 : 0;
        }
    }
}

void fd_roi_refine(
    const uint8_t* gray2,
    int width,
    int height,
    FD_BoundingBox* boxes,
    int* inout_num_boxes,
    const FD_Parameters* params
) {
    if (!gray2 || !boxes || !inout_num_boxes || !params) return;
    if (!params->roi_refine_enable) return;
    if (*inout_num_boxes <= 0) return;

    int expand = params->roi_refine_expand_px;
    if (expand < 0) expand = 0;
    int sobel_thresh = params->roi_refine_sobel_thresh;
    if (sobel_thresh < 0) sobel_thresh = 0;
    int min_cc_area = params->roi_refine_min_cc_area;
    if (min_cc_area < 0) min_cc_area = 0;
    bool keep_largest_only = params->roi_refine_keep_largest_only;

    // First pass: compute max ROI area for a reusable buffer.
    // 目的：只 malloc 一次 roi_mask（大小=max_roi_area），避免每個 bbox 都 malloc/free。
    // 注意：每個 bbox 的 ROI 大小可能不同，但都會 <= max_roi_area。
    size_t max_roi_area = 0;
    for (int i = 0; i < *inout_num_boxes; i++) {
        int x1 = boxes[i].x1 - expand;
        int y1 = boxes[i].y1 - expand;
        int x2 = boxes[i].x2 + expand;
        int y2 = boxes[i].y2 + expand;
        if (x1 < 0) x1 = 0;
        if (y1 < 0) y1 = 0;
        if (x2 > width - 1) x2 = width - 1;
        if (y2 > height - 1) y2 = height - 1;
        if (x2 < x1 || y2 < y1) continue;
        size_t area = (size_t)(x2 - x1 + 1) * (size_t)(y2 - y1 + 1);
        if (area > max_roi_area) max_roi_area = area;
    }
    if (max_roi_area == 0) {
        *inout_num_boxes = 0;
        return;
    }
    // 分配 roi_mask 記憶體來裝「ROI 的 edge mask」。
    // roi_mask 的大小固定是 max_roi_area（最大 ROI 面積），之後每個 bbox 只使用其中前 roi_area 個 bytes。
    uint8_t* roi_mask = (uint8_t*)FD_MALLOC(max_roi_area);
    if (!roi_mask) {
        ESP_LOGE(FD_TAG, "fd_roi_refine() malloc failed: roi_mask (%d bytes)", (int)max_roi_area);
        *inout_num_boxes = 0;
        return;
    }
    // write_idx 是 refine後還存在的bbox要寫回去boxes的index (依序寫)
    int write_idx = 0;
    // 遍歷所有 bbox
    for (int i = 0; i < *inout_num_boxes; i++) {
        FD_BoundingBox b = boxes[i];
        
        // 擴大 ROI 並 clamp 到 [0..width-1]/[0..height-1]
        int rx1 = b.x1 - expand;
        int ry1 = b.y1 - expand;
        int rx2 = b.x2 + expand;
        int ry2 = b.y2 + expand;
        if (rx1 < 0) rx1 = 0;
        if (ry1 < 0) ry1 = 0;
        if (rx2 > width - 1) rx2 = width - 1;
        if (ry2 > height - 1) ry2 = height - 1;
        if (rx2 < rx1 || ry2 < ry1) continue;

        int roi_w = rx2 - rx1 + 1;
        int roi_h = ry2 - ry1 + 1;
        size_t roi_area = (size_t)roi_w * (size_t)roi_h;

        // 如果 ROI 面積超過 max_roi_area，則跳過
        // 這應該不會發生，因為在上面已經計算出 max_roi_area，但為了安全起見，還是加上
        if (roi_area > max_roi_area) {
            printf("[ERROR] fd_roi_refine() roi_area > max_roi_area (%zu > %zu)\n", roi_area, max_roi_area);
            continue;
        }

        // 在 ROI 上做 Sobel(3x3) + threshold 得到 edge mask(0/255)
        // 關於 roi_mask 重複使用、但每個 ROI 大小不同為什麼不會出問題：
        // - 我們對 roi_mask 的存取永遠只在 ROI-local 範圍 [0 .. roi_w*roi_h-1] 內（透過 roi_w/roi_h indexing）。
        // - fd_sobel_threshold_roi_3x3() 內部只會寫入 roi_w*roi_h 的範圍，不會碰到 roi_area 之外。
        // - 後續 flood fill 也只會在 roi_w*roi_h 的範圍內讀寫 roi_mask。
        fd_sobel_threshold_roi_3x3(gray2, width, height, rx1, ry1, rx2, ry2, sobel_thresh, roi_mask);

        // Connected component selection strategy (ROI-local, inclusive):
        // - keep_largest_only=true : pick ONLY the single largest CC (by area)
        // - keep_largest_only=false: take the union of ALL CCs whose area>=min_cc_area,
        //   then use their minimal enclosing rectangle (min_x/min_y/max_x/max_y).
        // NOTE: keep_largest_only=false is NOT "largest"; it is "union bbox across candidates".
        //
        // =============================
        // 這一段是在「ROI 內的 edge mask」上找連通區（Connected Component, CC）
        // 並依照 keep_largest_only 決定 refined bbox（ROI-local, inclusive 座標）
        //
        // 重要前提：
        // - roi_mask 是 ROI-local 的二值圖（0/255）
        // - fd_flood_fill_optimized() 會「就地修改 roi_mask」（把走過的 255 變成其它值），
        //   所以同一個 CC 不會被重複計算。
        //
        // 我們會同時維護兩種候選：
        // 1) best_* : 面積最大的那一個 CC 的 bbox（for keep_largest_only=true）
        // 2) union_*: 所有「通過 min_cc_area」的 CC 的 union bbox（for keep_largest_only=false）
        // =============================
        //
        // best_*：記錄目前找到的「最大 CC」(area 最大) 的 ROI-local bbox（inclusive）
        int best_area = 0;
        int best_min_x = 0, best_max_x = -1, best_min_y = 0, best_max_y = -1;
        //
        // union_*：記錄所有「候選 CC（area>=min_cc_area）」的 union bbox（inclusive）
        // 初始化成無效框（max_x < min_x / max_y < min_y）
        int union_min_x = 0, union_max_x = -1, union_min_y = 0, union_max_y = -1;
        //
        // have_any_candidate：是否至少找到一個 area>=min_cc_area 的 CC
        // - 若沒有任何候選 CC，代表 refine 失敗 → 丟掉這個 bbox（不 fallback）
        bool have_any_candidate = false;
        //
        // 掃描 ROI 內所有像素；遇到 255 代表一個新的 CC 起點
        for (int y = 0; y < roi_h; y++) {
            for (int x = 0; x < roi_w; x++) {
                if (roi_mask[y * roi_w + x] != 255) continue;
                //
                // 對這個 CC 做 flood fill，取得該 CC 的 ROI-local bbox（inclusive）與面積
                int min_x = 0, max_x = 0, min_y = 0, max_y = 0, area = 0;
                fd_flood_fill_optimized(roi_mask, roi_w, roi_h, x, y, &min_x, &max_x, &min_y, &max_y, &area);
                //
                // 面積太小視為雜訊：不加入候選，也不影響 union / best 的更新
                if (area < min_cc_area) continue;
                //
                // ---- 更新 union bbox：把「這個候選 CC 的 bbox」併入 union_* ----
                if (!have_any_candidate) {
                    // 第一個候選 CC：直接用它初始化 union bbox
                    union_min_x = min_x; union_max_x = max_x;
                    union_min_y = min_y; union_max_y = max_y;
                    have_any_candidate = true;
                } else {
                    // 後續候選 CC：用 min/max 擴張 union bbox
                    if (min_x < union_min_x) union_min_x = min_x;
                    if (min_y < union_min_y) union_min_y = min_y;
                    if (max_x > union_max_x) union_max_x = max_x;
                    if (max_y > union_max_y) union_max_y = max_y;
                }
                //
                // ---- 更新 best（最大 CC）----
                if (area > best_area) {
                    best_area = area;
                    best_min_x = min_x;
                    best_max_x = max_x;
                    best_min_y = min_y;
                    best_max_y = max_y;
                }
            }
        }
        //
        // 若 ROI 內沒有任何候選 CC（全部都 < min_cc_area 或根本沒有 edge），refine 失敗 → 丟掉 bbox
        if (!have_any_candidate) {
            continue;
        }
        //
        // 依 keep_largest_only 選擇輸出哪個 ROI-local bbox（inclusive）
        // - keep_largest_only=true  ：用 best_*（最大 CC）
        // - keep_largest_only=false ：用 union_*（候選 CC union 後的最小外接矩形）
        int out_min_x = keep_largest_only ? best_min_x : union_min_x;
        int out_min_y = keep_largest_only ? best_min_y : union_min_y;
        int out_max_x = keep_largest_only ? best_max_x : union_max_x;
        int out_max_y = keep_largest_only ? best_max_y : union_max_y;
        //
        // 防呆：若輸出框無效則視為 refine 失敗
        if (out_max_x < out_min_x || out_max_y < out_min_y) continue;

        // Map ROI-local bbox back to image (inclusive)
        b.x1 = rx1 + out_min_x;
        b.y1 = ry1 + out_min_y;
        b.x2 = rx1 + out_max_x;
        b.y2 = ry1 + out_max_y;

        if (b.x1 < 0) b.x1 = 0;
        if (b.y1 < 0) b.y1 = 0;
        if (b.x2 > width - 1) b.x2 = width - 1;
        if (b.y2 > height - 1) b.y2 = height - 1;
        if (b.x2 < b.x1 || b.y2 < b.y1) continue;

        b.area = (b.x2 - b.x1 + 1) * (b.y2 - b.y1 + 1);
        boxes[write_idx++] = b;
    }

    free(roi_mask);
    *inout_num_boxes = write_idx;
}

static void fd_binary_threshold(uint8_t* src, uint8_t* dst, int width, int height, int threshold) {
    for (int i = 0; i < width * height; i++) {
        dst[i] = (src[i] > threshold) ? 255 : 0;  // 二值化：大於閾值為白，否則為黑
    }
}

static bool fd_flood_fill_optimized(uint8_t* mask, int width, int height, int start_x, int start_y, 
                              int* min_x, int* max_x, int* min_y, int* max_y, int* area) {
    
    // 初始 stack 容量 (約 8KB)
    const int INIT_CAPACITY = 2048;
    // 最大容量 = 整張圖片 (640×480 = 307,200) 最大可以到 307,200* sizeof(FD_Point) = 1.2MB
    const int MAX_CAPACITY = width * height;
    
    // 分配初始 stack 
    FD_Point* stack = (FD_Point*)FD_MALLOC(INIT_CAPACITY * sizeof(FD_Point));
    
    if (!stack) {
        ESP_LOGE(FD_TAG, "fd_flood_fill_optimized() 初始記憶體分配失敗: stack (%d bytes)", 
                 INIT_CAPACITY * (int)sizeof(FD_Point));
        return false;  // 分配失敗
    }
    
    int capacity = INIT_CAPACITY;
    int stack_size = 0;
    bool is_complete = true;  // 標記是否完整處理
    
    // 初始化起點和邊界框
    stack[stack_size++] = (FD_Point){(uint16_t)start_x, (uint16_t)start_y};
    *min_x = *max_x = start_x;
    *min_y = *max_y = start_y;
    *area = 0;
    
    // Flood Fill 主循環
    while (stack_size > 0) {
        FD_Point p = stack[--stack_size];
        
        // 邊界檢查
        if (p.x >= width || p.y >= height) continue;
        if (mask[p.y * width + p.x] != 255) continue;
        
        // 標記為已訪問並更新統計
        mask[p.y * width + p.x] = 128;
        (*area)++;
        
        // 更新邊界框
        if (p.x < *min_x) *min_x = p.x;
        if (p.x > *max_x) *max_x = p.x;
        if (p.y < *min_y) *min_y = p.y;
        if (p.y > *max_y) *max_y = p.y;
        
        // 檢查是否需要擴展 stack
        // 因為每次可能同時加入4個鄰居到stack，若stack剩餘空間不足4個，下一步就可能溢出，所以要提前擴展容量
        if (stack_size + 4 >= capacity) {
            //(最終實驗精簡log)//memory_print_info("fd_flood_fill_optimized() Stack擴展前" );
            if (capacity >= MAX_CAPACITY) {
                // Stack 已經到達最大容量，無法再擴展
                // 保留已經找到的部分結果，停止處理
                //(最終實驗精簡log)//memory_print_info("fd_flood_fill_optimized() Stack 達到最大容量，物體太大");
                ESP_LOGW(FD_TAG, "fd_flood_fill_optimized() Stack 達到最大容量，物體太大");
                ESP_LOGW(FD_TAG, "保留部分結果: area=%d, bbox=[%d,%d,%d,%d]", 
                         *area, *min_x, *min_y, *max_x, *max_y);
                is_complete = false;  // 標記為未完整處理
                break;  // 跳出循環，保留已找到的結果
            }
            
            // 擴展策略：容量翻倍，最多擴展到MAX_CAPACITY
            int new_capacity = capacity * 2;
            if (new_capacity > MAX_CAPACITY) {
                new_capacity = MAX_CAPACITY;
            }
            
            // 使用 realloc 擴展記憶體
            FD_Point* new_stack = (FD_Point*)FD_REALLOC(stack, new_capacity * sizeof(FD_Point));
            if (!new_stack) {
                // realloc 失敗，但原始 stack 依然有效
                // 保留已經找到的 area 和 bbox，停止繼續處理
                //(最終實驗精簡log)//memory_print_info("fd_flood_fill_optimized() Stack 擴展失敗");
                ESP_LOGW(FD_TAG, "fd_flood_fill_optimized() Stack 擴展失敗 (需要 %d bytes)",
                         new_capacity * (int)sizeof(FD_Point));
                ESP_LOGW(FD_TAG, "物體過大，保留已處理的部分結果 (area=%d, bbox=[%d,%d,%d,%d])", 
                         *area, *min_x, *min_y, *max_x, *max_y);
                is_complete = false;  // 標記為未完整處理
                break;  // 跳出 while 循環，保留已找到的結果
            }
            
            //(最終實驗精簡log)//memory_print_info("Stack 擴展後");
            //(最終實驗精簡log)//ESP_LOGI(FD_TAG, "Stack 擴展: %d -> %d (%.1f KB)", capacity, new_capacity, 
            //(最終實驗精簡log)//         (new_capacity * sizeof(FD_Point)) / 1024.0f);
            
            stack = new_stack;
            capacity = new_capacity;
        }
        
        // 加入四個鄰居到堆疊 (四連通) - 加入邊界檢查避免 uint16_t 下溢
        if (p.x + 1 < width)  stack[stack_size++] = (FD_Point){p.x + 1, p.y};
        if (p.x > 0)          stack[stack_size++] = (FD_Point){p.x - 1, p.y};
        if (p.y + 1 < height) stack[stack_size++] = (FD_Point){p.x, p.y + 1};
        if (p.y > 0)          stack[stack_size++] = (FD_Point){p.x, p.y - 1};
    }
    
    free(stack);
    return is_complete;  // 返回是否完整處理
}

static FD_BoundingBox* fd_get_contour_detections(uint8_t* mask, int width, int height, FD_Parameters* params, int* num_detections) {
    // 分配邊界框陣列
    FD_BoundingBox* bboxes = (FD_BoundingBox*)FD_MALLOC(FD_MAX_BBOXES * sizeof(FD_BoundingBox)); // FD_BoundingBox 通常有 5 個 int（假設每個 4 bytes），所以如果 FD_MAX_BBOXES=100，大小約 2000 bytes
    if (!bboxes) {
        ESP_LOGE(FD_TAG, "fd_get_contour_detections() 記憶體分配失敗: bboxes (%d bytes)", FD_MAX_BBOXES * sizeof(FD_BoundingBox));
        *num_detections = 0;
        return NULL;
    }
    int bbox_count = 0;
    
    // 掃描整個圖像尋找白色像素
    for (int y = 0; y < height && bbox_count < FD_MAX_BBOXES; y++) {
        for (int x = 0; x < width && bbox_count < FD_MAX_BBOXES; x++) {
            // 找到一個沒有被標記過(flood fill處理過)的白色像素，開始找他的連通組件，也就是找這個可能的box的座標和面積
            if (mask[y * width + x] == 255) {  
                int min_x = 0, max_x = 0, min_y = 0, max_y = 0, area = 0;
                // 使用優化版 flood fill 找連通組件（動態擴展，記憶體更高效）
                fd_flood_fill_optimized(mask, width, height, x, y, &min_x, &max_x, &min_y, &max_y, &area);
                
                // 檢查連通組件面積是否達到閾值
                if (area >= params->bbox_thresh) {
                    // 轉換為Python格式 [x1, y1, x2, y2, area]
                    bboxes[bbox_count].x1 = min_x;
                    bboxes[bbox_count].y1 = min_y;
                    // bbox 座標定義：inclusive（四個端點都包含）
                    // 有效像素範圍：x∈[x1..x2], y∈[y1..y2]，因此寬高需要 +1 (因為包含最後一個像素)
                    bboxes[bbox_count].x2 = max_x;
                    bboxes[bbox_count].y2 = max_y;
                    bboxes[bbox_count].area = (max_x - min_x + 1) * (max_y - min_y + 1);  // 邊界框面積（inclusive）
                    bbox_count++;
                }
            }
        }
    }
    
    *num_detections = bbox_count;
    return bboxes; 
}

static void fd_remove_contained_bboxes(FD_BoundingBox* boxes, int num_boxes, int* keep_indices, int* keep_count) {
    *keep_count = 0;
    
    // 檢查每個邊界框是否被其他邊界框完全包含
    for (int i = 0; i < num_boxes; i++) {
        bool is_contained = false;
        
        // 與所有其他邊界框比較
        for (int j = 0; j < num_boxes; j++) {
            if (i == j) continue;  // 不與自己比較
            
            // 檢查 box i 是否被 box j 完全包含（inclusive）
            // 注意：若兩個框完全相同，這個條件會「雙向成立」，會導致兩個都被視為 contained，
            // 進而出現 keep_count==0 的情況（進到 malloc(0) 的誤判/或直接沒有候選）。
            // 因此需要一個 tie-break：
            // - 若 j 的 area 更大 → i 視為 contained
            // - 若 area 相同（完全相同框常見）→ 只讓「索引較大」的那個被視為 contained，確保至少保留一個
            bool j_contains_i =
                (boxes[j].x1 <= boxes[i].x1 && boxes[j].y1 <= boxes[i].y1 &&
                 boxes[j].x2 >= boxes[i].x2 && boxes[j].y2 >= boxes[i].y2);
            if (j_contains_i) {
                if (boxes[j].area > boxes[i].area || (boxes[j].area == boxes[i].area && j < i)) {
                is_contained = true;
                break;  // 找到包含關係，跳出內層循環
                }
            }
        }
        
        // 如果沒有被包含，則保留
        if (!is_contained) {
            keep_indices[(*keep_count)++] = i;
        }
    }
}

static float fd_compute_iou(FD_BoundingBox* box1, FD_BoundingBox* box2) {
    // 計算交集的邊界
    int xi_min = (box1->x1 > box2->x1) ? box1->x1 : box2->x1;  // 交集左邊界
    int yi_min = (box1->y1 > box2->y1) ? box1->y1 : box2->y1;  // 交集上邊界
    int xi_max = (box1->x2 < box2->x2) ? box1->x2 : box2->x2;  // 交集右邊界
    int yi_max = (box1->y2 < box2->y2) ? box1->y2 : box2->y2;  // 交集下邊界
    
    // 檢查是否有交集
    // bbox 座標定義：inclusive（四個端點都包含）
    // 沒有交集的條件是 min > max（而不是 >=）
    if (xi_min > xi_max || yi_min > yi_max) {
        return 0.0f; // 沒有交集
    }
    
    // 計算交集面積
    int inter_w = xi_max - xi_min + 1;
    int inter_h = yi_max - yi_min + 1;
    int intersection = inter_w * inter_h;
    
    // 計算兩個邊界框的面積
    int area1 = (box1->x2 - box1->x1 + 1) * (box1->y2 - box1->y1 + 1);
    int area2 = (box2->x2 - box2->x1 + 1) * (box2->y2 - box2->y1 + 1);
    
    // 計算聯集面積 = 面積1 + 面積2 - 交集面積
    int union_area = area1 + area2 - intersection;
    
    if (union_area == 0) return 0.0f;  // 避免除零錯誤
    return (float)intersection / union_area;  // 返回IoU值
}

static FD_BoundingBox* fd_non_max_suppression(FD_BoundingBox* boxes, int num_boxes, FD_Parameters* params, int* final_boxs_num) {
    if (num_boxes == 0) {
        //(最終實驗精簡log)//ESP_LOGI(FD_TAG, "fd_non_max_suppression() 輸入邊界框數量為0");
        *final_boxs_num = 0;
        return NULL;
    }
    
    // 步驟1: 按面積排序（氣泡排序，降序）
    for (int i = 0; i < num_boxes - 1; i++) {
        for (int j = 0; j < num_boxes - i - 1; j++) {
            if (boxes[j].area < boxes[j + 1].area) {
                FD_BoundingBox temp = boxes[j];
                boxes[j] = boxes[j + 1];
                boxes[j + 1] = temp;
            }
        }
    }
    
    // 步驟2: 移除完全包含的框
    
    // 分配 keep_indices 緩衝區 keep_indices 放要保留的box其index的陣列
    //memory_print_info("fd_non_max_suppression: malloc keep_indices 前");
    int* keep_indices = (int*)FD_MALLOC(num_boxes * sizeof(int));
    if (!keep_indices) {
        ESP_LOGE(FD_TAG, "fd_non_max_suppression() 記憶體分配失敗: keep_indices (%d bytes)", num_boxes * sizeof(int));
        *final_boxs_num = 0;
        return NULL;
    }
    
    int keep_count = 0;  // 要保留的box數量
    fd_remove_contained_bboxes(boxes, num_boxes, keep_indices, &keep_count);

    // 若 keep_count==0，代表「移除 contained」後沒有任何候選框。直接回傳「沒有框」即可。
    if (keep_count == 0) {
        //(最終實驗精簡log)//ESP_LOGI(FD_TAG, "fd_non_max_suppression() 移除 contained 後沒有任何候選框");
        free(keep_indices);
        *final_boxs_num = 0;
        return NULL;
    }
    
    // 步驟3: IoU-based NMS
    
    // final_keep陣列  若final_keep[i]=true,則表示keep_indices[i]要保留 也就是第二層篩選後要保留的box的index
    //memory_print_info("fd_non_max_suppression: malloc final_keep 前");
    bool* final_keep = (bool*)FD_MALLOC(keep_count * sizeof(bool));
    if (!final_keep) {
        ESP_LOGE(FD_TAG, "fd_non_max_suppression() 記憶體分配失敗: final_keep (%d bytes)", keep_count * sizeof(bool));
        free(keep_indices);
        *final_boxs_num = 0;
        return NULL;
    }
    
    // 初始化final_keep為保留狀態
    for (int i = 0; i < keep_count; i++) {
        final_keep[i] = true;
    }
    
    // NMS主循環：比較每對邊界框
    for (int i = 0; i < keep_count; i++) {
        if (!final_keep[i]) continue;  // 已標記為移除，跳過
        
        for (int j = i + 1; j < keep_count; j++) {
            if (!final_keep[j]) continue;  // 已標記為移除，跳過
            
            // 計算IoU
            float iou = fd_compute_iou(&boxes[keep_indices[i]], &boxes[keep_indices[j]]);
            if (iou > params->iou_threshold) {
                final_keep[j] = false;  // 標記為移除
            }
        }
    }
    
    // 計算最終保留的框數量
    int final_count = 0;
    for (int i = 0; i < keep_count; i++) {
        if (final_keep[i]) final_count++;
    }
    
    //(最終實驗精簡log)//ESP_LOGI(FD_TAG, "fd_non_max_suppression() 最終保留 %d 個邊界框", final_count);

    // final_count==0 時，代表 NMS 後沒有框；這不是 malloc 失敗，直接回傳空即可（避免 malloc(0)）
    if (final_count == 0) {
        free(keep_indices);
        free(final_keep);
        *final_boxs_num = 0;
        return NULL;
    }
    
    // 分配結果陣列
    //memory_print_info("fd_non_max_suppression: malloc result 前");
    FD_BoundingBox* result = (FD_BoundingBox*)FD_MALLOC(final_count * sizeof(FD_BoundingBox));
    if (!result) {
        ESP_LOGE(FD_TAG, "fd_non_max_suppression() 記憶體分配失敗: result (%d bytes)", final_count * sizeof(FD_BoundingBox));
        // 釋放已分配的臨時緩衝區
        free(keep_indices);
        free(final_keep);
        *final_boxs_num = 0;
        return NULL;
    }
    
    // 複製要保留的box到結果陣列
    int result_idx = 0;
    for (int i = 0; i < keep_count; i++) {
        if (final_keep[i]) {
            result[result_idx++] = boxes[keep_indices[i]];
        }
    }
    
    // 釋放臨時緩衝區
    free(keep_indices);
    free(final_keep);
    
    *final_boxs_num = final_count;
    return result;
}

bool expand_bbox(FD_BoundingBox* bbox, int expand_px, int image_width, int image_height) {
    if (!bbox || expand_px < 0 || image_width <= 0 || image_height <= 0) return false;

    // inclusive bbox 必須滿足 x1<=x2, y1<=y2
    if (bbox->x2 < bbox->x1 || bbox->y2 < bbox->y1) return false;

    int x1 = bbox->x1 - expand_px;
    int y1 = bbox->y1 - expand_px;
    int x2 = bbox->x2 + expand_px;
    int y2 = bbox->y2 + expand_px;

    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 > image_width - 1) x2 = image_width - 1;
    if (y2 > image_height - 1) y2 = image_height - 1;

    if (x2 < x1 || y2 < y1) return false;

    bbox->x1 = x1;
    bbox->y1 = y1;
    bbox->x2 = x2;
    bbox->y2 = y2;
    bbox->area = (x2 - x1 + 1) * (y2 - y1 + 1);
    return true;
}

bool expand_roi_to_square(FD_BoundingBox* bbox, int image_width, int image_height) {
    if (!bbox || image_width <= 0 || image_height <= 0) return false;
    if (bbox->x2 < bbox->x1 || bbox->y2 < bbox->y1) return false;

    int w = bbox->x2 - bbox->x1 + 1;
    int h = bbox->y2 - bbox->y1 + 1;
    if (w <= 0 || h <= 0) return false;

    // 保證輸出仍為正方形：side 固定後只做「位移」，不做不對稱 clamp
    int side = (w > h) ? w : h;
    // 若 side 大於影像任一邊，則不可能在影像內維持正方形，這裡改成取可行的最大正方形邊長
    if (side > image_width) side = image_width;
    if (side > image_height) side = image_height;
    if (side <= 0) return false;

    // 以 bbox 中心（整數）為基準展開正方形
    int cx = bbox->x1 + w / 2;
    int cy = bbox->y1 + h / 2;

    int x1 = cx - side / 2;
    int y1 = cy - side / 2;
    // 只做位移以維持正方形：x1/y1 clamp 到可放下 side 的範圍
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x1 > image_width - side) x1 = image_width - side;
    if (y1 > image_height - side) y1 = image_height - side;

    int x2 = x1 + side - 1;
    int y2 = y1 + side - 1;
    if (x2 < x1 || y2 < y1) return false;

    bbox->x1 = x1;
    bbox->y1 = y1;
    bbox->x2 = x2;
    bbox->y2 = y2;
    bbox->area = (x2 - x1 + 1) * (y2 - y1 + 1);
    return true;
}

bool crop_resize_bbox(
    const uint8_t* src_gray,
    int src_width,
    int src_height,
    uint8_t* dst_gray,
    int dst_size,
    int resize_mode,
    const FD_BoundingBox* bbox
) {
    if (!src_gray || !dst_gray || !bbox) return false;
    if (src_width <= 0 || src_height <= 0 || dst_size <= 0) return false;
    if (bbox->x1 < 0 || bbox->y1 < 0 || bbox->x2 >= src_width || bbox->y2 >= src_height) return false;
    if (bbox->x2 < bbox->x1 || bbox->y2 < bbox->y1) return false;

    const int roi_w = bbox->x2 - bbox->x1 + 1;
    const int roi_h = bbox->y2 - bbox->y1 + 1;
    if (roi_w <= 0 || roi_h <= 0) return false;

    // resize_mode:
    // - 0: nearest（最快；整數映射）
    // - 1: bilinear（更精準；浮點權重）
    if (resize_mode == 1) {
        // bilinear：以 dst pixel center 映射到 ROI 座標
        // 將 dst 的 [0..dst_size-1] 映射到 ROI 的 [0..roi_w-1] / [0..roi_h-1]
        const float sx_scale = (dst_size > 1) ? ((float)(roi_w - 1) / (float)(dst_size - 1)) : 0.0f;
        const float sy_scale = (dst_size > 1) ? ((float)(roi_h - 1) / (float)(dst_size - 1)) : 0.0f;
        for (int dy = 0; dy < dst_size; dy++) {
            float fy = (float)dy * sy_scale;
            int y0 = (int)fy;
            int y1 = y0 + 1;
            if (y1 > roi_h - 1) y1 = roi_h - 1;
            float wy = fy - (float)y0;
            int sy0 = bbox->y1 + y0;
            int sy1 = bbox->y1 + y1;
            for (int dx = 0; dx < dst_size; dx++) {
                float fx = (float)dx * sx_scale;
                int x0 = (int)fx;
                int x1 = x0 + 1;
                if (x1 > roi_w - 1) x1 = roi_w - 1;
                float wx = fx - (float)x0;
                int sx0 = bbox->x1 + x0;
                int sx1 = bbox->x1 + x1;

                uint8_t p00 = src_gray[sy0 * src_width + sx0];
                uint8_t p10 = src_gray[sy0 * src_width + sx1];
                uint8_t p01 = src_gray[sy1 * src_width + sx0];
                uint8_t p11 = src_gray[sy1 * src_width + sx1];

                float top = (1.0f - wx) * (float)p00 + wx * (float)p10;
                float bot = (1.0f - wx) * (float)p01 + wx * (float)p11;
                float v = (1.0f - wy) * top + wy * bot;
                int iv = (int)(v + 0.5f);
                if (iv < 0) iv = 0;
                if (iv > 255) iv = 255;
                dst_gray[dy * dst_size + dx] = (uint8_t)iv;
            }
        }
    } else {
        // nearest：dst(u,v) 對應 src(x,y)
        // 用整數映射避免浮點：x = x1 + floor(u * roi_w / dst_size)
        // 並確保最後一列/行能落在 bbox 內部。
        for (int dy = 0; dy < dst_size; dy++) {
            int sy = bbox->y1 + (dy * roi_h) / dst_size;
            if (sy > bbox->y2) sy = bbox->y2;
            for (int dx = 0; dx < dst_size; dx++) {
                int sx = bbox->x1 + (dx * roi_w) / dst_size;
                if (sx > bbox->x2) sx = bbox->x2;
                dst_gray[dy * dst_size + dx] = src_gray[sy * src_width + sx];
            }
        }
    }
    return true;
}

// 回傳上次到現在經過的時間，並把 mark 移到現在
static uint32_t fd_lap_us(int64_t* mark) {
    const int64_t now = esp_timer_get_time();
    const uint32_t elapsed = (uint32_t)(now - *mark);
    *mark = now;
    return elapsed;
}

// gray2 不改動；gray1、temp 輪流當中間暫存
FD_DetectionResult* fd_process_frames_gray(uint8_t* gray1, uint8_t* gray2, uint8_t* temp, FD_Parameters* params, int* num_boxes, int width, int height, FD_StageTiming* timing) {
    *num_boxes = 0;
    *timing = (FD_StageTiming){0};
    if (!temp) {
        ESP_LOGE(FD_TAG, "fd_process_frames_gray(): temp is NULL");
        return NULL;
    }
    int64_t mark = esp_timer_get_time();

    // 幀差 → 二值化 → morphology
    get_absdiff(gray1, gray2, temp, width, height);                   // temp <- |gray2-gray1|
    fd_binary_threshold(temp, gray1, width, height, params->thresh);  // gray1 <- binary(temp)
    timing->diff_us = fd_lap_us(&mark);
    fd_apply_morphology(gray1, gray1, temp, width, height, params);   // gray1 <- morph(gray1)
    timing->morph_us = fd_lap_us(&mark);

    // 輪廓
    int initial_box_num = 0;
    FD_BoundingBox* initial_boxes = fd_get_contour_detections(gray1, width, height, params, &initial_box_num);
    timing->contour_us = fd_lap_us(&mark);
    if (initial_box_num == 0 || !initial_boxes) {
        free(initial_boxes);
        return NULL;
    }

    // ROI refine (Sobel 3x3)；結果就地寫回 initial_boxes
    fd_roi_refine(gray2, width, height, initial_boxes, &initial_box_num, params);
    timing->refine_us = fd_lap_us(&mark);
    if (initial_box_num == 0) {
        free(initial_boxes);
        return NULL;
    }

    // NMS
    int final_box_count = 0;
    FD_BoundingBox* final_boxes = fd_non_max_suppression(initial_boxes, initial_box_num, params, &final_box_count);
    free(initial_boxes);
    timing->nms_us = fd_lap_us(&mark);
    if (final_box_count == 0 || !final_boxes) {
        free(final_boxes);
        return NULL;
    }

    FD_DetectionResult* result = (FD_DetectionResult*)FD_MALLOC(sizeof(FD_DetectionResult));
    if (!result) {
        ESP_LOGE(FD_TAG, "fd_process_frames_gray() result allocation failed");
        free(final_boxes);
        return NULL;
    }
    result->bboxes = final_boxes;
    result->num_boxes = final_box_count;
    *num_boxes = final_box_count;
    return result;
}

void fd_free_detection_result(FD_DetectionResult* result) {
    if (result) {
        free(result->bboxes);
        free(result);
    }
}
