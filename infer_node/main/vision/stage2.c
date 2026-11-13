/**
 * @file stage2.c
 * @brief Stage2 core：greedy tracking + one-shot candidate（ESP32/PC 共用）。
 *
 * 使用方式（重點摘要，避免之後忘記）：
 * - ESP32（ESP-IDF）建議：
 *   - 只使用 core API：stage2_init() + stage2_update_core()
 *   - 建議定義 STAGE2_DISABLE_HISTORY（hist 目前僅 PC 畫軌跡用；ESP32 先關掉省記憶體/省 stack）
 *   - 不要定義 STAGE2_ENABLE_DEBUG（避免 PC debug wrapper/CSV）
 *
 * - PC（gcc）若要 debug/畫軌跡/寫 CSV：
 *   - 請在編譯參數定義 STAGE2_ENABLE_DEBUG
 */

#include "stage2.h"

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"

// ============================================================================
// Stage2 (C) implementation notes
// ============================================================================
//
// 1) 這個檔案的核心目標是「可直接搬到 ESP32」：
//    - 不依賴 OpenCV / JPEG / PC 畫圖
//    - 不依賴檔案 IO（CSV 輸出以 STAGE2_ENABLE_DEBUG 包住）
//    - 輸入 bbox 只 copy value，不保存外部指標
//
// 2) 關聯策略對齊 Python v4（multi_track_greedy_1130_v4.py）：
//    - predicted center = (cx+vx, cy+vy)
//    - Manhattan distance gate
//    - optional area ratio gate (det.area / track.area)
//    - 收集候選 pair 後依 (dist, track_idx, det_idx) 排序，greedy 一對一配對
//
// 3) age gate：你指定 one-shot
//    - track.age 只在「成功接上 detection」時才 age++（新生 track age=1）
//    - age==age_threshold 當下輸出一次 candidate，並設 cnn_sent=true，避免後續重複
//
// 4) inclusive bbox：
//    - width = x2-x1+1, height = y2-y1+1, area=width*height
//    - cx/cy 用 (x1+x2)/2, (y1+y2)/2
//
// 5) ESP32 stack 注意：
//    - stage2_update_core() 需要一些工作陣列（pairs / used flags）。
//      若放 stack 上，ESP32 task stack 可能不足，因此改用 static storage。
//      這意味著 stage2_update_core() 不是 re-entrant；若未來多 task 並行跑多 tracker，
//      需要在外層加鎖，或把工作緩衝搬到 Stage2_Tracker 裡。

/**
 * @brief 用於儲存一組 (追蹤軌跡, 檢測框) 配對的資訊（greedy 配對算法內部工作結構）。
 *
 * 一個 Stage2_Pair 結構代表一個 "可行的配對候選"：一條追蹤軌跡（track）和一個當前幀的 detection 之間的配對關係，
 * 並附帶本次配對的距離和面積比等運算資訊。這個結構陣列會在匹配過程中集中計算及排序，以決定最佳配對。
 */
typedef struct {
    int dist;          /**< 距離分數：用於配對評估的距離（Manhattan distance，越小越可能配對）。 */
    int track_idx;     /**< 對應到的追蹤軌道在 tracker->tracks 陣列中的 index。 */
    int det_idx;       /**< 對應到的偵測框在 detection 輸入陣列中的 index。 */
    float area_ratio;  /**< 此次配對的面積比例（det_area / track_area），用於過濾不合理的配對。 */
} Stage2_Pair;

// --------------------------------------------------------------------------
// Core working buffers (static storage)
// - For ESP32 stack safety (avoid large stack frames)
// - Not re-entrant / not thread-safe
// --------------------------------------------------------------------------
// stage2_update_core() 使用的工作緩衝區（static storage，避免 ESP32 stack overflow；非 re-entrant，非 thread-safe）

// 儲存所有符合 gate 條件的候選配對 (track, detection, distance, area_ratio)
// did_node：32 x 100 x 16 B = 51 KB，放 .bss 會吃掉大半內部 RAM，改在 stage2_init() 配置到 PSRAM
#define STAGE2_MAX_PAIRS (STAGE2_MAX_TRACKS * STAGE2_MAX_DETECTIONS)
static Stage2_Pair* g_pairs = NULL;

// 紀錄每個 track 是否已經配對到 detection（greedy matching 用）
static bool g_track_used[STAGE2_MAX_TRACKS];

// 紀錄每個 detection 是否已經被配對（greedy matching 用）
static bool g_det_used[STAGE2_MAX_DETECTIONS];

// 每個 track 在本幀成功配對到的 detection index（-1 = 沒有配對成功）
static int g_matched_det_by_track[STAGE2_MAX_TRACKS];

// 每個 track 在本幀配對到的 distance（-1 = 沒有配對成功）
static int g_dist_by_track[STAGE2_MAX_TRACKS];

// 每個 track 在本幀配對到的 area_ratio（-1.0f = 沒有配對成功）
static float g_ar_by_track[STAGE2_MAX_TRACKS];

// 每個 track 本幀是否產生 candidate（one-shot CNN gate 用）
static bool g_candidate_by_track[STAGE2_MAX_TRACKS];

// 本幀實際處理的 detection 數量（>STAGE2_MAX_DETECTIONS 時會截斷）
static int g_last_used_detections = 0;

// 本幀原始輸入的 detection 數量
static int g_last_input_detections = 0;

// 本幀是否有發生 detection 過多被截斷
static bool g_last_detections_truncated = false;

/**
 * @brief 取 int 絕對值（避免引入額外依賴）。
 *
 * @param v [in] 輸入整數
 * @return int |v|
 *
 * @note 用途：Stage2 的距離計算需要 abs，這裡用小 helper 讓行為一致。
 */
static int stage2_abs_int(int v) {
    return (v < 0) ? -v : v;
}

/**
 * @brief 取得 bbox 寬度（inclusive）。
 *
 * @param bbox [in] bbox 指標（可為 NULL）
 * @return int 寬度 = x2-x1+1；若無效回傳 0。
 *
 * @note 這裡的「無效」包含 bbox==NULL 或 x2<x1。
 */
int stage2_bbox_width(const FD_BoundingBox* bbox) {
    // inclusive bbox：合法情況 x2>=x1，寬度 = x2-x1+1
    if (!bbox || bbox->x2 < bbox->x1) return 0;
    return bbox->x2 - bbox->x1 + 1;
}

/**
 * @brief 取得 bbox 高度（inclusive）。
 *
 * @param bbox [in] bbox 指標（可為 NULL）
 * @return int 高度 = y2-y1+1；若無效回傳 0。
 */
int stage2_bbox_height(const FD_BoundingBox* bbox) {
    // inclusive bbox：合法情況 y2>=y1，高度 = y2-y1+1
    if (!bbox || bbox->y2 < bbox->y1) return 0;
    return bbox->y2 - bbox->y1 + 1;
}

/**
 * @brief 取得 bbox 面積（inclusive）。
 *
 * @param bbox [in] bbox 指標（可為 NULL）
 * @return int 面積 = width*height；若無效回傳 0。
 *
 * @note 這裡使用 inclusive 定義的 width/height（都有 +1）。
 */
int stage2_bbox_area(const FD_BoundingBox* bbox) {
    // inclusive bbox 面積：w*h（注意 +1 已包含在 width/height）
    int w = stage2_bbox_width(bbox);
    int h = stage2_bbox_height(bbox);
    return w * h;
}

/**
 * @brief 取得 bbox 中心 x（整數）。
 *
 * @param bbox [in] bbox 指標（可為 NULL）
 * @return int center x = (x1+x2)/2；bbox==NULL 時回傳 0。
 */
int stage2_bbox_cx(const FD_BoundingBox* bbox) {
    // inclusive bbox center：用端點取整（與 Python 的 //2 類似）
    if (!bbox) return 0;
    return (bbox->x1 + bbox->x2) / 2;
}

/**
 * @brief 取得 bbox 中心 y（整數）。
 *
 * @param bbox [in] bbox 指標（可為 NULL）
 * @return int center y = (y1+y2)/2；bbox==NULL 時回傳 0。
 */
int stage2_bbox_cy(const FD_BoundingBox* bbox) {
    if (!bbox) return 0;
    return (bbox->y1 + bbox->y2) / 2;
}

/**
 * @brief Manhattan distance（L1 距離）。
 *
 * @param x1 [in] 點1 x
 * @param y1 [in] 點1 y
 * @param x2 [in] 點2 x
 * @param y2 [in] 點2 y
 * @return int |x1-x2| + |y1-y2|
 *
 * @note 用途：與 Python v4 對齊（greedy matching 以 Manhattan distance 當作關聯距離）。
 */
static int stage2_manhattan(int x1, int y1, int x2, int y2) {
    return stage2_abs_int(x1 - x2) + stage2_abs_int(y1 - y2);
}

/**
 * @brief 面積比 area ratio：ar = new_area / old_area。
 *
 * @param new_area [in] 新偵測 bbox 面積
 * @param old_area [in] track 目前 bbox 面積（match 前）
 * @return float ar 值
 *
 * @note 用途：做關聯 gate（Python v4 的 AREA_RATIO_MIN/MAX）。
 * @note old_area<=0 時：回傳極大值/或 1.0，避免除零並讓 gate 行為可預期。
 */
static float stage2_area_ratio(int new_area, int old_area) {
    // ar = new_area / old_area
    // old_area<=0：避免除 0；此時回傳極大值/或 1.0，讓 gate 行為可預期。
    if (old_area <= 0) {
        return (new_area > 0) ? 1.0e30f : 1.0f;
    }
    return (float)new_area / (float)old_area;
}

/**
 * @brief 取得本 tracker 的「實際允許的最大 active tracks 數」。
 *
 * @param tracker [in] tracker 狀態
 * @return int 最大 tracks 數（>=1）
 *
 * 規則：
 * - 若 params.max_active_tracks <= 0：使用編譯期上限 STAGE2_MAX_TRACKS
 * - 若 params.max_active_tracks 在 (0, STAGE2_MAX_TRACKS) 之間：使用該值
 * - 若 params.max_active_tracks >= STAGE2_MAX_TRACKS：仍使用 STAGE2_MAX_TRACKS
 *
 * 用途：
 * - 限制同畫面同時存在的 tracks 數，避免「每幀偵測很多雜訊」時一直開新 track。
 */
static int stage2_effective_max_tracks(const Stage2_Tracker* tracker) {
    // max_active_tracks<=0 表示使用 STAGE2_MAX_TRACKS
    int limit = STAGE2_MAX_TRACKS;
    if (tracker && tracker->params.max_active_tracks > 0 && tracker->params.max_active_tracks < limit) {
        limit = tracker->params.max_active_tracks;
    }
    return limit;
}

#if defined(STAGE2_ENABLE_DEBUG) && !defined(STAGE2_DISABLE_HISTORY)
/**
 * @brief 清空 track 的 hist ring-buffer。
 *
 * @param track [in,out] track 狀態
 *
 * @note 用途：新生 track 或 reset 時，避免軌跡線使用到舊資料。
 */
static void stage2_hist_clear(Stage2_Track* track) {
    // hist ring buffer 清空
    track->hist_count = 0;
    track->hist_start = 0;
}

/**
 * @brief 將一個中心點 (cx,cy) 推入 track 的 hist ring-buffer。
 *
 * @param track [in,out] track 狀態
 * @param cx    [in]     中心 x
 * @param cy    [in]     中心 y
 *
 * @note 用途：PC debug 畫軌跡線（僅保留最近 STAGE2_HISTORY_LEN 個點）。
 */
static void stage2_hist_push(Stage2_Track* track, int cx, int cy) {
    if (!track) return;
    int pos;
    if (track->hist_count < STAGE2_HISTORY_LEN) {
        pos = (track->hist_start + track->hist_count) % STAGE2_HISTORY_LEN;
        track->hist_count++;
    } else {
        pos = track->hist_start;
        track->hist_start = (track->hist_start + 1) % STAGE2_HISTORY_LEN;
    }
    track->hist[pos].cx = cx;
    track->hist[pos].cy = cy;
}

/**
 * @brief 把 ring-buffer 的 hist 攤平成 0..hist_count-1（方便上層畫圖/輸出）。
 *
 * @param track    [in]  track 狀態
 * @param snapshot [out] 要寫入的 snapshot
 */
static void stage2_copy_history(const Stage2_Track* track, Stage2_TrackSnapshot* snapshot) {
    // 把 ring buffer 攤平成 [0..hist_count-1]，方便 CSV / 畫圖使用
    snapshot->hist_count = track->hist_count;
    for (int i = 0; i < track->hist_count; i++) {
        int src = (track->hist_start + i) % STAGE2_HISTORY_LEN;
        snapshot->hist[i] = track->hist[src];
    }
}
#endif

/**
 * @brief 清空一條 track（將其設為 inactive 並清掉狀態）。
 *
 * @param track [in,out] track 狀態
 *
 * @note 用途：
 * - miss 超過上限時淘汰
 * - reset tracker 時清空
 */
static void stage2_clear_track(Stage2_Track* track) {
    // track 失效時清空狀態（含 bbox/速度/age/miss），避免 reuse 時殘留舊資料
    if (!track) return;
    memset(track, 0, sizeof(*track));
    track->track_id = -1;
}

/**
 * @brief 從一個 detection bbox 建立新 track（age=1, miss=0）。
 *
 * @param track    [out] 要初始化的 track（會被覆寫）
 * @param track_id [in]  新的 track id（單調遞增）
 * @param bbox     [in]  detection bbox（full-res, inclusive）
 *
 * @note 重要：bbox.area 可能未填，這裡會用 inclusive 規則重算 area，確保後續 area_ratio gate 正確。
 */
static void stage2_init_track_from_bbox(Stage2_Track* track, int track_id, const FD_BoundingBox* bbox) {
    // 新建 track：age=1, miss=0, vx/vy=0
    // 注意：bbox.area 可能還沒填，我們會在這裡用 inclusive 規則重算。
    stage2_clear_track(track);
    track->active = true;
    track->track_id = track_id;
    track->bbox = *bbox;
    track->bbox.area = stage2_bbox_area(&track->bbox);
    track->cx = stage2_bbox_cx(&track->bbox);
    track->cy = stage2_bbox_cy(&track->bbox);
    track->vx = 0;
    track->vy = 0;
    track->age = 1;
    track->miss = 0;
    track->cnn_sent = false;
#if defined(STAGE2_ENABLE_DEBUG) && !defined(STAGE2_DISABLE_HISTORY)
    stage2_hist_clear(track);
    stage2_hist_push(track, track->cx, track->cy);
#endif
}

/**
 * @brief 此函式不做 gate、不做選擇；只做更新。配對要不要成立由外部決定。
 *  接受 (track, detection) 的配對，將 detection 併入該 track，更新該track的狀態(位置、速度、age、miss)。
 *
 * 用意：
 * - 這個函式只在「配對已經被決定要成立」時呼叫（例如 greedy/Hungarian 的結果）。
 * - 它負責把 track 的狀態更新到最新：位置/速度/生命週期計數器。
 * - 同時回傳配對品質指標 dist 與 area_ratio，供 debug/CSV 記錄使用。
 *
 * 會做的事（依序）：
 * 1) 以更新前的 track 狀態計算 predicted center = (cx+vx, cy+vy)
 * 2) 計算 dist = manhattan(det.center, predicted center)
 * 3) 計算 area_ratio = det.area / old_track.area
 *    （注意 dist/area_ratio 是「配對品質指標」，刻意用更新前狀態計算）
 * 4) 接受配對：用 detection 覆寫 track->bbox，並重算 track->cx/track->cy/area
 * 5) 速度更新：new_v = curr_center - prev_center，並用 IIR 做平滑更新 vx/vy
 * 6) 生命週期更新：age++（只有 match 成功才加），miss=0
 * 7) （可選）debug 模式下把中心點推入 hist，用於畫軌跡
 *
 * @param track [in,out] 要被更新的 track（配對成立後的狀態會寫回這裡）
 * @param det   [in]     本幀要併入的 detection bbox（full-res, inclusive）
 * @param out_dist [out] 回傳 dist（可為 NULL）
 * @param out_area_ratio [out] 回傳 area_ratio（可為 NULL）
 */
static void stage2_apply_match(Stage2_Track* track, const FD_BoundingBox* det, int* out_dist, float* out_area_ratio) {
    // 接上 detection 的更新規則（對齊 Python v4 apply_match）：
    // - dist / ar：用「配對前」的 predicted center 與舊 bbox area 計算，用於 log/CSV
    // - 更新 bbox/cx/cy
    // - 速度估計：new_v = curr - prev，再做簡單 smoothing：v = (3*v + new_v)/4
    // - age++（只有 match 成功才增加）
    // - miss=0
    //
    // 重要：這裡的 dist/ar 是「配對品質指標」，故刻意用更新前的狀態計算：
    // - dist：det.center 到 pred_center 的距離（pred_center = old_center + old_velocity）
    // - ar  ：det.area / old_track.area
    // 這樣你在 log/CSV 看到的 dist/ar 才能反映「本次配對有多合理」，而不是永遠變成 0 / 1。
    int pred_cx = track->cx + track->vx;
    int pred_cy = track->cy + track->vy;
    int dist = stage2_manhattan(stage2_bbox_cx(det), stage2_bbox_cy(det), pred_cx, pred_cy);
    float ar = stage2_area_ratio(stage2_bbox_area(det), stage2_bbox_area(&track->bbox));

    // 上一幀（更新前）的中心點，用來估計本次位移 -> new_v
    int prev_cx = track->cx;
    int prev_cy = track->cy;

    // 接受配對：用 detection 覆寫 bbox，並重算中心與面積（inclusive bbox）
    track->bbox = *det;
    track->bbox.area = stage2_bbox_area(&track->bbox);
    track->cx = stage2_bbox_cx(&track->bbox);
    track->cy = stage2_bbox_cy(&track->bbox);

    // 速度更新：
    // - new_v = (curr - prev) 是「本次觀測到的速度」
    // - v = (3*v + new_v)/4 是簡單的 IIR 平滑（0.75 舊速度 + 0.25 新速度），可降低抖動
    int new_vx = track->cx - prev_cx;
    int new_vy = track->cy - prev_cy;
    track->vx = (3 * track->vx + new_vx) / 4;
    track->vy = (3 * track->vy + new_vy) / 4;

    // 生命週期：
    // - age：只在成功 match 時才累加（表示「連續被追蹤到的次數」）
    // - miss：成功 match 則歸零；未 match 會在外層流程 miss++
    track->age++;
    track->miss = 0;
#if defined(STAGE2_ENABLE_DEBUG) && !defined(STAGE2_DISABLE_HISTORY)
    // PC debug 用：記錄中心點歷史以畫軌跡；ESP32 可關閉以省記憶體
    stage2_hist_push(track, track->cx, track->cy);
#endif
    // 如果out_dist和out_area_ratio不是NULL，就回傳dist和ar
    if (out_dist) *out_dist = dist;
    if (out_area_ratio) *out_area_ratio = ar;
}

/**
 * @brief qsort 用的候選 pair 排序比較器。
 *
 * @param a [in] Stage2_Pair*
 * @param b [in] Stage2_Pair*
 * @return int 依 (dist, track_idx, det_idx) 做遞增排序
 *
 * @note 用途：
 * - 讓 greedy 配對的 tie-break 固定，避免不同平台或不同編譯器下結果漂移。
 */
static int stage2_pair_compare(const void* a, const void* b) {
    // 排序 key： (dist, track_idx, det_idx)
    // 目的：距離越小越先配對；同距離時固定 tie-break，讓結果穩定可重現。
    const Stage2_Pair* pa = (const Stage2_Pair*)a;
    const Stage2_Pair* pb = (const Stage2_Pair*)b;
    if (pa->dist != pb->dist) return pa->dist - pb->dist;
    if (pa->track_idx != pb->track_idx) return pa->track_idx - pb->track_idx;
    return pa->det_idx - pb->det_idx;
}

/**
 * @brief 在 candidates buffer 裡追加一筆 CNN candidate event（one-shot）。
 *
 * @param out_buf   [in,out] candidates 輸出緩衝
 * @param frame_idx [in]     幀索引（寫入 cand->frame_idx，方便與 PC/ESP32 的 frame_idx 對齊）
 * @param track     [in]     觸發 candidate 的 track（使用其「更新後」的 bbox）
 * @return bool true=已放進緩衝；false=緩衝滿，這條 track 留到下一次配對再送
 *
 * @note Stage2 不會呼叫 CNN，只會輸出「要送 CNN 的資訊」給上層 pipeline。
 */
static bool stage2_candidatebuf_append(Stage2_CandidateBuffer* out_buf, int frame_idx, const Stage2_Track* track) {
    // one-shot candidate event：只做「紀錄」，不做 CNN 呼叫
    if (!out_buf || !out_buf->candidates || !track) return false;
    if (out_buf->num_candidates >= out_buf->max_candidates) return false;
    Stage2_CnnCandidate* cand = &out_buf->candidates[out_buf->num_candidates++];
    cand->valid = true;
    cand->frame_idx = frame_idx;
    cand->track_id = track->track_id;
    cand->bbox = track->bbox;
    return true;
}

#ifdef STAGE2_ENABLE_DEBUG
static void stage2_build_snapshot(
    Stage2_FrameOutput* output,
    const Stage2_Track* track,
    int frame_idx,
    int matched_det_idx,
    bool accepted,
    int dist,
    float area_ratio,
    bool candidate_this_frame
) {
    /**
     * @brief 將 tracker 內部的單一 track 狀態 copy 成「本幀快照」。
     *
     * @param output              [in,out] 本幀輸出（會 append 一筆 tracks[]）
     * @param track               [in]     track 狀態
     * @param frame_idx           [in]     幀索引
     * @param matched_det_idx     [in]     本幀配對到的 det index（-1 表示未配對）
     * @param accepted            [in]     是否配對成功
     * @param dist                [in]     配對距離（未配對則 -1）
     * @param area_ratio          [in]     area ratio（未配對則 -1）
     * @param candidate_this_frame[in]     本幀是否產生 candidate（目前用於擴充；此版未直接寫入 snapshot 欄位）
     *
     * 用途：
     * - 讓上層（PC）可以寫 CSV、畫軌跡線、或 debug 顯示 track 的 age/miss 等狀態。
     */
    if (!output || !track || output->num_tracks >= STAGE2_MAX_TRACKS) return;
    Stage2_TrackSnapshot* snap = &output->tracks[output->num_tracks++];
    memset(snap, 0, sizeof(*snap));
    snap->active = track->active;
    snap->frame_idx = frame_idx;
    snap->track_id = track->track_id;
    snap->matched_det_idx = matched_det_idx;
    snap->accepted = accepted;
    snap->dist = dist;
    snap->area_ratio = area_ratio;
    snap->age = track->age;
    snap->miss = track->miss;
    snap->bbox = track->bbox;
    snap->cx = track->cx;
    snap->cy = track->cy;
    snap->vx = track->vx;
    snap->vy = track->vy;
    snap->cnn_ready = false;
    snap->cnn_sent = track->cnn_sent;
    (void)candidate_this_frame;
#if !defined(STAGE2_DISABLE_HISTORY)
    stage2_copy_history(track, snap);
#endif
}
#endif // STAGE2_ENABLE_DEBUG

/**
 * @brief 填入 Stage2_Params 的預設值（對齊 Python v4 常用設定）。
 *
 * @param params [out] 參數結構（會被覆寫）
 *
 * 用途：
 * - 讓呼叫端能先拿到一組合理可跑的 baseline，再覆寫部分欄位。
 */
void stage2_default_params(Stage2_Params* params) {
    // Python v4 預設值（以你目前使用的為準）
    if (!params) return;
    params->max_match_dist = 60;
    params->max_miss = 2;
    params->age_threshold = 5;
    params->area_ratio_min = 0.30f;
    params->area_ratio_max = 3.00f;
    params->enable_area_ratio_gate = true;
    params->max_active_tracks = STAGE2_MAX_TRACKS;
}

/**
 * @brief 清空 tracker：移除所有 tracks 並重置 track id counter。
 *
 * @param tracker [in,out] tracker 狀態（會被重置）
 *
 * 用途：
 * - 重新開始新的序列（換資料夾/換影片）時使用。
 */
void stage2_reset(Stage2_Tracker* tracker) {
    // 清空所有 track 與 track id counter
    if (!tracker) return;
    for (int i = 0; i < STAGE2_MAX_TRACKS; i++) {
        stage2_clear_track(&tracker->tracks[i]);
    }
    tracker->next_track_id = 0;
    tracker->expired_candidates = 0;
}

/**
 * @brief 初始化 tracker 並帶入參數（含防呆），接著清空內部狀態。
 *
 * @param tracker [out] tracker 結構（會被清空與初始化）
 * @param params  [in]  參數（可為 NULL；NULL 時使用 stage2_default_params）
 *
 * 內部動作：
 * - copy/default params
 * - clamp 不合理數值（負值、max<min 等）
 * - stage2_reset() 清空所有 tracks 與 next_track_id
 */
void stage2_init(Stage2_Tracker* tracker, const Stage2_Params* params) {
    // 初始化 tracker；同時做參數防呆（避免負值或不合理區間）
    if (!tracker) return;
    if (!g_pairs) {
        g_pairs = (Stage2_Pair*)heap_caps_malloc(STAGE2_MAX_PAIRS * sizeof(Stage2_Pair),
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    memset(tracker, 0, sizeof(*tracker));
    if (params) {
        tracker->params = *params;
    } else {
        stage2_default_params(&tracker->params);
    }
    if (tracker->params.max_match_dist < 0) tracker->params.max_match_dist = 0;
    if (tracker->params.max_miss < 0) tracker->params.max_miss = 0;
    if (tracker->params.age_threshold < 1) tracker->params.age_threshold = 1;
    if (tracker->params.area_ratio_min < 0.0f) tracker->params.area_ratio_min = 0.0f;
    if (tracker->params.area_ratio_max < tracker->params.area_ratio_min) {
        tracker->params.area_ratio_max = tracker->params.area_ratio_min;
    }
    stage2_reset(tracker);
}
/**
 * @brief Stage2 core 主流程（每幀呼叫一次）：更新 tracker，並輸出 one-shot CNN candidates。
 *
 * @param tracker        [in,out] tracker 狀態（跨幀持久存在；tracks 會就地更新）
 * @param detections     [in]     Stage1 bbox 陣列（full-res, inclusive）。可為 NULL（代表本幀無偵測）。
 * @param num_detections [in]     bbox 數量
 * @param frame_idx      [in]     幀索引（寫入 Stage2_CnnCandidate.frame_idx，方便後續對照）
 * @param out_buf        [in,out] candidates 輸出緩衝（caller 提供陣列+容量；本函數會填 num_candidates）
 *
 * @return bool true=成功；false=參數錯誤（例如 out_buf 無效）
 *
 * 核心設計（你要求的方案 A）：
 * - 這個 core 只處理「跨幀追蹤」與「candidate 事件輸出」，不做任何 PC 專用的 snapshot/CSV/畫圖。
 * - PC 若要 debug/對照，請使用 `stage2_update_debug()`，它會呼叫 core 並產生 `Stage2_FrameOutput`。
 *
 * 內部做的事（高階，與 Python v4 對齊）：
 * - (0) 將 num_detections 截斷到 STAGE2_MAX_DETECTIONS（避免工作陣列爆掉）
 * - (A) 收集候選 pair：dist gate +（可選）area_ratio gate
 * - (B) 依 (dist, track_idx, det_idx) 排序後 greedy 一對一配對（track/det 各用一次）
 * - (C) unmatched tracks：miss++，超過 max_miss 淘汰
 * - (D) unmatched detections：開新 track（若未超過 max_active_tracks 且有空 slot）
 * - (E) one-shot age gate：某條 track 達到 age_threshold 且還沒送過時輸出 candidate 並設 cnn_sent=true
 *       out_buf 放不下 (max_candidates 可為 0，代表 CNN 正忙) 時不設 cnn_sent，下一次配對成功再送，
 *       用的是當時最新的 bbox；還沒送出就被淘汰的 track 計入 tracker->expired_candidates。
 *
 * @note ESP32 若要和 PC 對照軌跡，建議在每幀 core 更新後，逐條輸出 `tracker->tracks[i]` 的
 *       (frame_idx, track_id, cx, cy, vx, vy, age, miss, bbox) 到 CSV/log；不需要 hist 也能重建軌跡。
 */
bool stage2_update_core(
    Stage2_Tracker* tracker,
    const FD_BoundingBox* detections,
    int num_detections,
    int frame_idx,
    Stage2_CandidateBuffer* out_buf
) {
    if (!tracker || !out_buf || !out_buf->candidates || out_buf->max_candidates < 0 || !g_pairs) return false;

    out_buf->num_candidates = 0;
    g_last_input_detections = (num_detections > 0) ? num_detections : 0;

    if (!detections || num_detections < 0) {
        num_detections = 0;
    }

    int used_detections = num_detections;
    if (used_detections > STAGE2_MAX_DETECTIONS) {
        // 防止 bbox 數量過多導致工作陣列爆掉：超過上限就截斷
        used_detections = STAGE2_MAX_DETECTIONS;
        g_last_detections_truncated = true;
    } else {
        g_last_detections_truncated = false;
    }
    g_last_used_detections = used_detections;

    // 初始化工作緩衝區(清空)
    memset(g_track_used, 0, sizeof(g_track_used));
    memset(g_det_used, 0, sizeof(g_det_used));
    memset(g_candidate_by_track, 0, sizeof(g_candidate_by_track));
    for (int i = 0; i < STAGE2_MAX_TRACKS; i++) {
        g_matched_det_by_track[i] = -1;
        g_dist_by_track[i] = -1;
        g_ar_by_track[i] = -1.0f;
    }

    // ------------------------------------------------------------
    // (A) 收集所有通過 gate 的候選 pair（配對現有追蹤 track 與 detection，並計算距離與面積比例）
    // ------------------------------------------------------------
    // 對於每個 active track，它去跟每個 detection 都算一次距離與面積比例，如果符合gate條件，則收錄為候選pair
    // 這一步只是對每個track先找出所有符合gate條件的bbox(符合就把他們pair起來)，每個track都會有pair_count個候選pair(多個bbox可能符合條件)
    int pair_count = 0;
    for (int track_idx = 0; track_idx < STAGE2_MAX_TRACKS; track_idx++) {
        Stage2_Track* track = &tracker->tracks[track_idx];  
        if (!track->active) continue; // 只考慮有active的 track
        // 用預測位置（center + velocity）計算配對距離
        int pred_cx = track->cx + track->vx;
        int pred_cy = track->cy + track->vy;
        int old_area = stage2_bbox_area(&track->bbox);
        // 對每一個 detection 嘗試配對
        for (int det_idx = 0; det_idx < used_detections; det_idx++) {
            const FD_BoundingBox* det = &detections[det_idx];
            int det_area = stage2_bbox_area(det);
            // 用 detection 的中心點與預測點計算 manhattan 距離  
            int dist = stage2_manhattan(stage2_bbox_cx(det), stage2_bbox_cy(det), pred_cx, pred_cy);
            // 計算 area_ratio（目前 detection 面積/過往 track 的面積）
            float ar = stage2_area_ratio(det_area, old_area);
            // 判斷是否啟用面積比過濾gate，以及 area_ratio 是否落在範圍內
            bool area_ok = (!tracker->params.enable_area_ratio_gate) ||
                           (ar >= tracker->params.area_ratio_min && ar <= tracker->params.area_ratio_max);
            // 該detection與該track同時滿足距離/面積 gate 且配對 buffer 沒溢位，就收錄為候選pair
            if (dist <= tracker->params.max_match_dist && area_ok && pair_count < STAGE2_MAX_PAIRS) {
                g_pairs[pair_count].dist = dist;            // 配對距離
                g_pairs[pair_count].track_idx = track_idx;  // 該配對的 track index
                g_pairs[pair_count].det_idx = det_idx;      // 該配對的 detection index
                g_pairs[pair_count].area_ratio = ar;        // 面積比例
                pair_count++;                               // 候選pair總數+1
            }
        }
    }

    // ------------------------------------------------------------
    // (B) 依距離排序後做 greedy 一對一配對
    // ------------------------------------------------------------
    // 將所有候選 pair 依距離（通常是 dist 由小到大）排序。
    // 這一步很關鍵：後面採用 greedy，等於「先嘗試最接近（最合理）的配對」。
    // 注意：這裡還不是隨機配對；配對順序完全由排序結果決定。
    qsort(g_pairs, (size_t)pair_count, sizeof(g_pairs[0]), stage2_pair_compare);

    // 一個個把已經經過大小排序的候選pair篩選，把已經被用過的track和detection排除掉，剩下的候選pair都加入真正的配對結果中
    for (int pair_idx = 0; pair_idx < pair_count; pair_idx++) {
        int track_idx = g_pairs[pair_idx].track_idx;
        int det_idx = g_pairs[pair_idx].det_idx;
        
        // 一對一限制，排除：
        // - 同一條 track 只能配到一個 detection（track 被用過就跳過）
        // - 同一個 detection 也只能被一條 track 使用（det 被用過就跳過）
        // 因此就算某個 det 同時接近多條 track，也只會被「最先被接受」的那一條拿走。
        if (g_track_used[track_idx] || g_det_used[det_idx]) continue;

        int dist = -1;
        float ar = -1.0f;
        
        // stage2_apply_match 會接受此 pair，正式把 det 併入 track 狀態（更新 bbox/cx/cy/vx/vy/age/miss...）。
        // dist/ar 也會由 stage2_apply_match 回傳(雖然g_pairs已經算過一次了，但為了保險，stage2_apply_match會再算一次)
        stage2_apply_match(&tracker->tracks[track_idx], &detections[det_idx], &dist, &ar);
        
        // 把已經被用過的track和detection標記起來，避免重複配對
        g_track_used[track_idx] = true;
        g_det_used[det_idx] = true;
        g_matched_det_by_track[track_idx] = det_idx;
        g_dist_by_track[track_idx] = dist;
        g_ar_by_track[track_idx] = ar;

        // --------------------------------------------------------
        // one-shot age gate：到達 age_threshold 後輸出一次 candidate；緩衝滿就留到下一次配對
        // --------------------------------------------------------
        if ( !(tracker->tracks[track_idx].cnn_sent) && (tracker->tracks[track_idx].age >= tracker->params.age_threshold) &&
             stage2_candidatebuf_append(out_buf, frame_idx, &tracker->tracks[track_idx]) ) {
            tracker->tracks[track_idx].cnn_sent = true;
            g_candidate_by_track[track_idx] = true;
        }
    }

    // ------------------------------------------------------------
    // (C) unmatched tracks：miss++，超過 max_miss 該track則失效(清空track狀態)
    // ------------------------------------------------------------
    for (int track_idx = 0; track_idx < STAGE2_MAX_TRACKS; track_idx++) {
        Stage2_Track* track = &tracker->tracks[track_idx];
        if (!track->active || g_track_used[track_idx]) continue;
        track->miss++;
        if (track->miss > tracker->params.max_miss) {
            if (track->age >= tracker->params.age_threshold && !track->cnn_sent) {
                tracker->expired_candidates++;
            }
            stage2_clear_track(track);
        }
    }

    // ------------------------------------------------------------
    // (D) unmatched detections：開新 track（若未超過 max_active_tracks 且有空位）
    // ------------------------------------------------------------
    int max_tracks = stage2_effective_max_tracks(tracker);
    for (int det_idx = 0; det_idx < used_detections; det_idx++) {
        // 只處理「本幀尚未被任何 track 配走」的 detection
        if (g_det_used[det_idx]) continue;

        // 找一個空槽 free_idx（inactive 的 track slot），並統計目前 active tracks 數量
        int free_idx = -1; // inactive 的 track 的index
        int active_count = 0;
        for (int ti = 0; ti < STAGE2_MAX_TRACKS; ti++) {
            if (tracker->tracks[ti].active) {
                active_count++;
            } else if (free_idx < 0) {
                free_idx = ti;
            }
        }
        // 資源/雜訊控管：
        // - active_count >= max_tracks：同畫面 track 數已達上限（避免雜訊把 tracks 開爆）
        // - free_idx < 0：沒有空槽可用
        // 兩者任一成立就停止開新 track；剩下的 detections 會被忽略（本幀不追蹤）
        if (active_count >= max_tracks || free_idx < 0) break;

        // 開新 track：用此 detection 初始化（通常 age 會從 1 開始，vx/vy=0，miss=0，cnn_sent=false）
        tracker->next_track_id++;
        stage2_init_track_from_bbox(&tracker->tracks[free_idx], tracker->next_track_id, &detections[det_idx]);

        // 標記此 detection 已被使用（避免同一個 det 同時開多條 track）
        g_det_used[det_idx] = true;

        // debug/snapshot 用：把「新生 track」視為本幀已配對到 det_idx
        g_matched_det_by_track[free_idx] = det_idx;
        // dist/ar 對新生 track 沒有「配對前狀態」可比，這裡用 (0, 1.0) 當作直覺的佔位值
        g_dist_by_track[free_idx] = 0;
        g_ar_by_track[free_idx] = 1.0f;

        // 若 age_threshold==1，新生 track 會在同幀立刻觸發 one-shot candidate（只會送一次）
        if (!tracker->tracks[free_idx].cnn_sent && tracker->tracks[free_idx].age >= tracker->params.age_threshold &&
            stage2_candidatebuf_append(out_buf, frame_idx, &tracker->tracks[free_idx])) {
            tracker->tracks[free_idx].cnn_sent = true;
            g_candidate_by_track[free_idx] = true;
        }
    }
    return true;
}

#ifdef STAGE2_ENABLE_DEBUG
bool stage2_update_debug(
    Stage2_Tracker* tracker,
    const FD_BoundingBox* detections,
    int num_detections,
    int frame_idx,
    Stage2_FrameOutput* output
) {
    /**
     * @brief PC debug wrapper：先呼叫 core 更新 tracker/產生 candidates，再產生本幀 snapshots 供 CSV/畫圖。
     *
     * @param tracker        [in,out] tracker 狀態（跨幀持久存在）
     * @param detections     [in]     Stage1 bbox 陣列（full-res, inclusive）
     * @param num_detections [in]     detections 數量
     * @param frame_idx      [in]     幀索引
     * @param output         [out]    本幀 debug 輸出（tracks snapshots + candidates）
     *
     * @return bool true=成功；false=參數錯誤
     *
     * 設計目的：
     * - 讓 PC 端能把「每幀每條 track 的狀態」寫成 CSV，並且畫在 output_frames 上，方便你目視驗證追蹤是否合理。
     * - 同時輸出 candidates（one-shot 事件），用來對照何時會觸發 Stage3。
     *
     * 注意：
     * - 這個 wrapper 不改變 core 追蹤邏輯；它只是在 core 完成後，把 tracker 內部狀態整理成容易觀察的格式。
     */
    if (!tracker || !output) return false;
    memset(output, 0, sizeof(*output));
    output->frame_idx = frame_idx;
    output->input_detections = (num_detections > 0) ? num_detections : 0;

    Stage2_CandidateBuffer buf;
    buf.candidates = output->candidates;
    buf.max_candidates = STAGE2_MAX_TRACKS;
    buf.num_candidates = 0;

    if (!stage2_update_core(tracker, detections, num_detections, frame_idx, &buf)) {
        return false;
    }
    output->num_candidates = buf.num_candidates;
    output->used_detections = g_last_used_detections;
    output->detections_truncated = g_last_detections_truncated;

    // 產生本幀 snapshots（PC 用：CSV/畫圖）
    for (int ti = 0; ti < STAGE2_MAX_TRACKS; ti++) {
        Stage2_Track* tr = &tracker->tracks[ti];
        if (!tr->active) continue;
        bool accepted = (g_matched_det_by_track[ti] >= 0);
        stage2_build_snapshot(
            output,
            tr,
            frame_idx,
            g_matched_det_by_track[ti],
            accepted,
            g_dist_by_track[ti],
            g_ar_by_track[ti],
            g_candidate_by_track[ti]
        );
        output->tracks[output->num_tracks - 1].cnn_ready = (tr->age >= tracker->params.age_threshold);
        output->tracks[output->num_tracks - 1].cnn_sent = tr->cnn_sent;
    }

    return true;
}
#endif

#ifdef STAGE2_ENABLE_DEBUG
/**
 * @brief 寫 `stage2_tracks.csv` 的 header 列。
 *
 * @param fp [in,out] 已開啟的檔案指標（w 模式）
 *
 * @note 這些欄位對應 `Stage2_TrackSnapshot` 與關聯結果，用於 PC 比對/除錯。
 */
void stage2_debug_write_tracks_csv_header(FILE* fp) {
    if (!fp) return;
    fprintf(fp, "frame_idx,track_id,matched_det_idx,accepted,dist,area_ratio,age,miss,x1,y1,x2,y2,area,cx,cy,vx,vy,cnn_ready,cnn_sent\n");
}

/**
 * @brief 將某一幀的 tracks snapshot 追加寫入 `stage2_tracks.csv`。
 *
 * @param fp     [in,out] 檔案指標（a 或 w 皆可）
 * @param output [in]     stage2_update_debug() 本幀輸出
 *
 * 內部動作：
 * - 逐條 track 輸出一列（CSV）
 *
 * 用途：
 * - 讓你能像 Python 版一樣檢查「每幀的 track_id/age/miss/match」是否合理。
 */
void stage2_debug_write_tracks_csv_frame(FILE* fp, const Stage2_FrameOutput* output) {
    if (!fp || !output) return;
    for (int i = 0; i < output->num_tracks; i++) {
        const Stage2_TrackSnapshot* tr = &output->tracks[i];
        fprintf(
            fp,
            "%d,%d,%d,%d,%d,%.6f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            output->frame_idx,
            tr->track_id,
            tr->matched_det_idx,
            tr->accepted ? 1 : 0,
            tr->dist,
            tr->area_ratio,
            tr->age,
            tr->miss,
            tr->bbox.x1,
            tr->bbox.y1,
            tr->bbox.x2,
            tr->bbox.y2,
            tr->bbox.area,
            tr->cx,
            tr->cy,
            tr->vx,
            tr->vy,
            tr->cnn_ready ? 1 : 0,
            tr->cnn_sent ? 1 : 0
        );
    }
}

/**
 * @brief 寫 `stage2_cnn_candidates.csv` 的 header 列。
 *
 * @param fp [in,out] 已開啟的檔案指標（w 模式）
 */
void stage2_debug_write_candidates_csv_header(FILE* fp) {
    if (!fp) return;
    fprintf(fp, "frame_idx,track_id,x1,y1,x2,y2,area\n");
}

/**
 * @brief 將某一幀的 CNN candidates 追加寫入 `stage2_cnn_candidates.csv`。
 *
 * @param fp     [in,out] 檔案指標（a 或 w 皆可）
 * @param output [in]     stage2_update_debug() 本幀輸出
 *
 * 用途：
 * - 未來串 Stage3 時，這份檔案就等於「何時要送 CNN」的事件列表（one-shot）。
 */
void stage2_debug_write_candidates_csv_frame(FILE* fp, const Stage2_FrameOutput* output) {
    if (!fp || !output) return;
    for (int i = 0; i < output->num_candidates; i++) {
        const Stage2_CnnCandidate* cand = &output->candidates[i];
        if (!cand->valid) continue;
        fprintf(
            fp,
            "%d,%d,%d,%d,%d,%d,%d\n",
            cand->frame_idx,
            cand->track_id,
            cand->bbox.x1,
            cand->bbox.y1,
            cand->bbox.x2,
            cand->bbox.y2,
            cand->bbox.area
        );
    }
}
#endif
