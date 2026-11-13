/**
 * @file stage2.h
 * @brief Stage2：簡易多軌追蹤（greedy 一對一關聯 + age gate），可同時用在 PC 與 ESP32。
 *
 * ============================================================================
 * [Stage2 的定位 / 為什麼要獨立成模組]
 * ----------------------------------------------------------------------------
 * - Stage1（Frame Difference）負責：每幀產生偵測 bbox（可能有抖動、可能每幀數量不同）。
 * - Stage2（本模組）負責：把「跨幀」的偵測 bbox 做關聯，維護 track 的連續性（track_id / age / miss / vx/vy）。
 * - Stage3（CNN）負責：吃 ROI 做分類/辨識（此版 Stage2 不會呼叫 CNN，只會輸出 candidate event）。
 *
 * Stage2 的輸入/輸出設計目標：
 * - 輸入：Stage1 的 bbox（**已經回推到 full-res 座標系**，且符合 inclusive 公約）。
 * - 輸出：
 *   1) 每一幀 active tracks 的狀態（PC 用於 debug / 畫軌跡）。
 *   2) one-shot CNN candidate（當某條 track 第一次達到 age_threshold 時輸出一次）。
 *
 * ============================================================================
 * [COORDINATE_CONVENTION / 座標公約 - 必讀]
 * ----------------------------------------------------------------------------
 * 本專案的 FD_BoundingBox 一律使用 inclusive（四端點都包含）：
 *   - x ∈ [x1..x2], y ∈ [y1..y2]
 *   - width  = x2 - x1 + 1
 *   - height = y2 - y1 + 1
 *   - area   = width * height
 *
 * 注意：Python 版（multi_track_greedy_1130_v4.py）BBox.w/h 寫的是 (x2-x1)/(y2-y1)，
 * 那比較像 half-open/exclusive 的寫法；C 版不能照抄，必須遵守此 inclusive 公約，
 * 否則 area_ratio / cx/cy / ROI crop 的語意會全亂掉。
 *
 * ============================================================================
 * [關聯規則（對齊 Python v4）]
 * ----------------------------------------------------------------------------
 * 1) 對每條 track 計算 predicted center：
 *      pred = (cx + vx, cy + vy)
 * 2) 對每個 detection 計算距離：
 *      dist = |det.cx - pred_cx| + |det.cy - pred_cy|   (Manhattan distance)
 *    並要求 dist <= max_match_dist 才能成為候選 pair。
 * 3) 可選 area ratio gate（建議先保留，與 Python v4 一致）：
 *      ar = det.area / track.area
 *      area_ratio_min <= ar <= area_ratio_max
 * 4) 收集所有候選 pair 後，依 (dist, track_idx, det_idx) 排序，再 greedy 一對一配對。
 *
 * ============================================================================
 * [age gate / CNN candidate（你指定的 one-shot 行為）]
 * ----------------------------------------------------------------------------
 * - track.age 只在「成功配對到 detection」時才會 age++。
 * - 當某條 track 在本幀更新後 age >= age_threshold 且還沒送過：
 *     => 輸出一次 Stage2_CnnCandidate
 *     => 並設 track.cnn_sent=true，避免後續重複送出。
 * - 呼叫端的緩衝放不下 (max_candidates 可為 0) 時不設 cnn_sent，下一次配對成功再送。
 *
 * ============================================================================
 * [PC / ESP32 差異與 macro]
 * ----------------------------------------------------------------------------
 * - STAGE2_DISABLE_HISTORY：
 *     關閉 track 的 hist ring-buffer（ESP32 可以用來省記憶體/省輸出）。
 *
 *     補充（重要）：
 *     - 目前 hist 只用於「PC 端 debug 畫軌跡圖」（把 track 的中心點歷史畫成折線）。
 *     - ESP32 端部分則沒用到
 *       故目前先關掉 hist（建議定義 STAGE2_DISABLE_HISTORY），讓結構更小、stack 壓力更低，
 *     - 但我們刻意保留這個開關的設計：未來若你想在用「短期歷史」來做決策
 *       （例如 track 生死判定、穩定性判斷、速度/方向一致性檢查等），就能再打開或改成更精簡的歷史狀態。
 * - STAGE2_ENABLE_DEBUG：
 *     啟用「PC 專用」debug wrapper 與 CSV 輸出 helper。
 *
 *     為什麼需要這個 macro？
 *     - ESP32 端的核心需求是：每幀更新 tracker + 取得 candidates（送 Stage3）。
 *       這些只需要 `stage2_update_core()` 與 `Stage2_Tracker/Stage2_Track/Stage2_CnnCandidate`。
 *     - PC 端為了驗證追蹤是否正確，會需要：
 *       - 每幀 tracks snapshot（寫 CSV / 畫軌跡）
 *       - candidates 的 CSV
 *       這些屬於「除錯輸出」，不希望變成 ESP32 的編譯負擔（stdio/較大的 struct）。
 *
 *     因此我們只用「一個」macro：STAGE2_ENABLE_DEBUG
 *     - 不定義：只編 core（最適合 ESP32）
 *     - 定義：額外提供 debug wrapper + CSV helpers（最適合 PC）
 *
 * ----------------------------------------------------------------------------
 * [建議用法：macro 要在哪裡 define 才會生效？]
 * ----------------------------------------------------------------------------
 * 重要原則：macro 必須在「編譯 stage2.c 的那個編譯單位」可見，才會真的影響到結構大小/編譯行為。
 *
 * - ESP32（ESP-IDF / 本專案 FD_did_cam_v4）：
 *   - 建議在 component 的 `CMakeLists.txt` 用 `target_compile_definitions(...)` 加：
 *       - STAGE2_DISABLE_HISTORY   （ESP32 先關 hist）
 *     - 不要定義 STAGE2_ENABLE_DEBUG（避免把 PC debug wrapper/CSV 編進 ESP32）
 *
 * - PC（gcc 編譯 c_implementation/tests）：
 *   - 若要輸出 tracks/candidates CSV、畫軌跡、使用 debug wrapper，請在編譯參數加：
 *       - -DSTAGE2_ENABLE_DEBUG
 *
 * @note 不建議把這些 macro 寫在單一 .c 檔的最上方（#define ...）來「期待其他 .c 自動看到」，
 *       因為每個 .c 都是獨立編譯；這正是先前會發生 link error 的根源。
 */

#ifndef STAGE2_H
#define STAGE2_H

#include <stdbool.h>
#include <stdint.h>

// 讓 Stage2 同時能在 ESP32 與 PC 編譯：
// - ESP32：應該會有 fd_mix.h（FD_BoundingBox 定義在此）
// - PC：則可能是 frame_differencing_improved.h
//
// 注意：不要只依賴 ESP_PLATFORM，因為某些工具/IDE 的解析環境可能不會定義它，
// 會導致 Stage2 型別在 C++ 檔中「看起來」未宣告（即使 ESP-IDF 真實 build 沒問題）。
#if defined(__has_include)
#  if __has_include("fd_mix.h")
#    include "fd_mix.h"
#  else
#    include "frame_differencing_improved.h"
#  endif
#else
#  ifdef ESP_PLATFORM
#    include "fd_mix.h"
#  else
#    include "frame_differencing_improved.h"
#  endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Build modes (core vs debug)
// ----------------------------------------------------------------------------
// - ESP32 建議：不要定義 STAGE2_ENABLE_DEBUG，只使用 stage2_update_core() 拿 candidates。
// - PC 建議：定義 STAGE2_ENABLE_DEBUG，才會有 TrackSnapshot/FrameOutput 與 stage2_update_debug()。
// ============================================================================

#ifndef STAGE2_MAX_TRACKS
#define STAGE2_MAX_TRACKS 32
#endif

#ifndef STAGE2_MAX_DETECTIONS
#define STAGE2_MAX_DETECTIONS FD_MAX_BBOXES
#endif

#ifndef STAGE2_HISTORY_LEN
#define STAGE2_HISTORY_LEN 16
#endif

/**
 * @brief Stage2 的參數集合（對齊 Python v4 的 tracking/gate 設定）。
 *
 * 用途：
 * - 控制「哪些 (track, det) 可以成為候選配對」：距離 gate、面積比 gate。
 * - 控制 track 的生命週期：max_miss（連續幾幀沒配到就淘汰）。
 * - 控制何時觸發 one-shot CNN candidate：age_threshold。
 * - 控制同畫面最多 tracks 數：max_active_tracks（避免雜訊開爆）。
 */
typedef struct {
    // Greedy 關聯 gate（對齊 Python v4）
    int max_match_dist;          /**< Manhattan distance gate：dist <= max_match_dist 才允許配對 */
    int max_miss;                /**< miss counter 上限：miss > max_miss 則 track 失效 */

    // age gate（one-shot candidate）：age 只在成功接上 detection 時才會遞增
    int age_threshold;           /**< age == age_threshold 時輸出一次 CNN candidate（之後不重複送） */

    // 面積比 gate（可關閉；建議先保留以對齊 Python v4）
    float area_ratio_min;        /**< ar = det.area / track.area 的下界 */
    float area_ratio_max;        /**< ar = det.area / track.area 的上界 */
    bool enable_area_ratio_gate; /**< true：距離 + 面積比都要過；false：只看距離 */

    // 同畫面最多 track 數（<=0 表示用 STAGE2_MAX_TRACKS）
    int max_active_tracks;
} Stage2_Params;

/**
 * @brief 2D 點（整數座標）。
 *
 * 用途：
 * - 儲存 track 的中心點歷史（hist），用於 PC 畫軌跡線。
 */
typedef struct {
    int cx;
    int cy;
} Stage2_center_Point;

/**
 * @brief Track（軌跡）的「持久狀態」（跨幀累積，會被 Stage2 core 就地更新）。
 *
 * 你可以把它理解成 Python v4 裡 `Track` class 的 C 版等價物。
 *
 * 重要概念：
 * - Stage2_Tracker 會持有一個 `Stage2_Track tracks[STAGE2_MAX_TRACKS]` 陣列。
 * - 每個 Stage2_Track 代表一條 active 軌跡（或 inactive 的空槽）。
 * - Stage2 不會保存 Stage1 result 指標，只會把 bbox 的「值」copy 進 track->bbox。
 *
 * 這個 struct 的生命週期：
 * - 由 `Stage2_Tracker` 持有並跨幀存在。
 * - 每一幀呼叫 `stage2_update_core()`（或 PC 的 `stage2_update_debug()`）後，
 *   tracker->tracks[i] 的內容會被就地更新（match / miss / 淘汰 / 新生 track）。
 *
 * 欄位語意：
 * - cx/cy：目前 bbox 的中心（整數）
 * - vx/vy：速度估計，用來做 predicted center = (cx+vx, cy+vy)
 * - age：成功 match 的次數（新生 track age=1；只有 match 成功才 age++）
 * - miss：連續未 match 的次數（每幀未配到 det 就 miss++，超過 max_miss 淘汰）
 * - cnn_sent：one-shot gate；當 age==age_threshold 觸發 candidate 後設 true，避免後續重複送 CNN
 */
typedef struct {
    bool active;
    int track_id;
    FD_BoundingBox bbox;
    int cx;
    int cy;
    int vx;
    int vy;
    int age;
    int miss;
    bool cnn_sent; // one-shot gate；當 age==age_threshold 觸發 candidate 後設 true，避免後續重複送 CNN
#ifndef STAGE2_DISABLE_HISTORY
    // hist ring-buffer：用於 PC 畫軌跡線（中心點序列）
    Stage2_center_Point hist[STAGE2_HISTORY_LEN]; 
    int hist_count; // 目前有多少有效點（最多 STAGE2_HISTORY_LEN）
    int hist_start; // ring buffer 的「最舊那個點」在陣列中的 index
#endif
} Stage2_Track;

/**
 * @brief one-shot CNN candidate 事件（Stage2 的輸出，不會在 Stage2 內呼叫 CNN）。
 *
 * 用途：
 * - 當某條 track 在本幀更新後「第一次」到達 age_threshold，Stage2 會輸出一筆事件：
 *     (frame_idx, track_id, bbox)
 * - 上層 pipeline（PC/ESP32）收到事件後，才去做 ROI crop/expand/resize 並送 Stage3 CNN。
 */
typedef struct {
    bool valid;
    int frame_idx;
    int track_id;
    FD_BoundingBox bbox;
} Stage2_CnnCandidate;

/**
 * @brief Stage2 core 的 candidates 輸出緩衝（由呼叫端提供）。
 *
 * 用途：
 * - ESP32 端只需要這個：每幀更新後，如果 num_candidates>0，就把 candidates 交給 Stage3。
 *
 * 使用方式（典型）：
 * - 呼叫端先配置一個陣列（通常大小 >= STAGE2_MAX_TRACKS）
 * - 設定 out_buf.candidates / out_buf.max_candidates
 * - 呼叫 `stage2_update_core(..., &out_buf)` 後，讀 out_buf.num_candidates 與 out_buf.candidates[0..n-1]
 */
typedef struct {
    Stage2_CnnCandidate* candidates; /**< [out] caller 提供的陣列 */
    int max_candidates;              /**< [in]  candidates 容量 */
    int num_candidates;              /**< [out] 實際輸出數量 */
} Stage2_CandidateBuffer;

#ifdef STAGE2_ENABLE_DEBUG
/**
 * @brief TrackSnapshot（每幀快照）：把「track 的狀態」+「本幀配對結果」封裝成可讀的輸出。
 *
 * 為什麼需要 Snapshot？
 * - `Stage2_Track` 是跨幀持久狀態，會被下一幀覆蓋更新；如果你想把「每幀狀態」寫成 CSV、
 *   或者在 PC 畫圖顯示「本幀有沒有 match、dist/ar 是多少」，就需要一份「本幀視角」的資料。
 * - Snapshot 也避免 PC debug code 去直接讀/推導 tracker 內部狀態（降低耦合）。
 *
 * 它包含兩類資訊：
 * 1) 本幀配對結果（frame-local）：matched_det_idx/accepted/dist/area_ratio
 * 2) track 更新後狀態（state after update）：bbox/cx/cy/vx/vy/age/miss/cnn_ready/cnn_sent (+hist)
 */
typedef struct {
    bool active;
    int frame_idx;
    int track_id;
    int matched_det_idx; // -1 means unmatched in this frame
    bool accepted;
    int dist;            // -1 means unavailable
    float area_ratio;    // -1.0f means unavailable
    int age;
    int miss;
    FD_BoundingBox bbox;
    int cx;
    int cy;
    int vx;
    int vy;
    bool cnn_ready;
    bool cnn_sent;
#ifndef STAGE2_DISABLE_HISTORY
    Stage2_center_Point hist[STAGE2_HISTORY_LEN];
    int hist_count;
#endif
} Stage2_TrackSnapshot;

/**
 * @brief Stage2_FrameOutput（每幀 debug 輸出）：PC 用的本幀輸出封裝。
 *
 * 用途：
 * - PC：
 *   - 寫 `stage2_tracks.csv`（用 tracks[]）
 *   - 寫 `stage2_cnn_candidates.csv`（用 candidates[]）
 *   - 畫軌跡線/標籤到 output_frames（用 tracks[].hist 與 bbox/cx/cy）
 *
 * 設計重點：
 * - output 只描述「本幀」；不保存跨幀持久狀態（跨幀狀態在 Stage2_Tracker 內）。
 * - detections_truncated：若本幀 detection 太多被截斷，可用來 debug 為何漏追蹤/漏關聯。
 *
 * @note 這個 struct 僅在 `STAGE2_ENABLE_DEBUG` 下可用；ESP32 建議只用 `stage2_update_core()` 取 candidates。
 */
typedef struct {
    int frame_idx;
    int input_detections;
    int used_detections;
    bool detections_truncated;
    int num_tracks;
    Stage2_TrackSnapshot tracks[STAGE2_MAX_TRACKS];
    int num_candidates;
    Stage2_CnnCandidate candidates[STAGE2_MAX_TRACKS];
} Stage2_FrameOutput;
#endif // STAGE2_ENABLE_DEBUG

/**
 * @brief Stage2_Tracker（追蹤器本體）：持有 Stage2 的跨幀持久狀態。
 *
 * Tracker 跟 Track 的關係（你問的重點）：
 * - Tracker 是「容器 / 管理者」：保存參數 + 一組 tracks 槽位 + 下一個要發的 track_id。
 * - Track 是「單一軌跡的狀態」：bbox/速度/age/miss/hist/cnn_sent 等。
 *
 * 一句話：
 * - `Stage2_Tracker` = 整個 Stage2 的狀態（跨幀）。
 * - `Stage2_Track`   = Tracker 裡面的一條軌跡（多條）。
 */
typedef struct {
    Stage2_Params params;
    Stage2_Track tracks[STAGE2_MAX_TRACKS];
    int next_track_id;
    uint32_t expired_candidates; /**< 達到 age_threshold 但等不到 CNN 空位就被淘汰的 track 數 */
} Stage2_Tracker;

/**
 * @brief 填入 Stage2_Params 的預設值（對齊 Python v4 的常用設定）。
 *
 * @param params [out] 參數結構。此函數會覆寫其內容。
 *
 * @note 這個函數的用途是提供「一個合理可用的 baseline」，之後呼叫端可以再覆寫其中部分欄位。
 */
void stage2_default_params(Stage2_Params* params);

/**
 * @brief 初始化 tracker（跨幀持久狀態），並做參數防呆。
 *
 * @param tracker [out] tracker 結構。會被清空並初始化。
 * @param params  [in]  參數（可為 NULL；NULL 時使用 stage2_default_params）。
 *
 * @note 這個函數會呼叫 stage2_reset() 清空所有 tracks 與 track_id counter。
 */
void stage2_init(Stage2_Tracker* tracker, const Stage2_Params* params);

/**
 * @brief 清空 tracker 內的所有 track 狀態與 id counter。
 *
 * @param tracker [in,out] tracker 狀態（會被重置）。
 *
 * @note 用途：
 * - 重新開始一段新的序列（例如換資料夾/換影片）時使用。
 */
void stage2_reset(Stage2_Tracker* tracker);

/**
 * @brief Stage2 core 每幀更新：更新 tracker 並輸出「CNN candidates 清單」。
 *
 * @param tracker        [in,out] tracker 狀態（跨幀持久存在）
 * @param detections     [in]     Stage1 bboxes（full-res, inclusive）；Stage2 只 copy 值、不保存指標
 * @param num_detections [in]     detections 數量
 * @param frame_idx      [in]     幀索引（寫入 candidate.frame_idx）
 * @param out_buf        [in,out] candidates 輸出緩衝（caller 提供陣列 + 容量；函數會填 num_candidates）
 *
 * @return bool true=成功；false=參數錯誤
 *
 * @note 這是 ESP32 端最推薦使用的 API：不需要 TrackSnapshot/FrameOutput，只拿 candidates 去串 Stage3。
 */
bool stage2_update_core(
    Stage2_Tracker* tracker,
    const FD_BoundingBox* detections,
    int num_detections,
    int frame_idx,
    Stage2_CandidateBuffer* out_buf
);

#ifdef STAGE2_ENABLE_DEBUG
/**
 * @brief Stage2 debug wrapper：呼叫 core 更新 tracker，並額外產生每幀可讀的 FrameOutput（含 snapshots）。
 *
 * @param tracker        [in,out] tracker 狀態（跨幀）
 * @param detections     [in]     Stage1 bboxes（full-res, inclusive）
 * @param num_detections [in]     detections 數量
 * @param frame_idx      [in]     幀索引
 * @param output         [out]    debug output（tracks snapshot + candidates）
 * @return bool true=成功
 */
bool stage2_update_debug(
    Stage2_Tracker* tracker,
    const FD_BoundingBox* detections,
    int num_detections,
    int frame_idx,
    Stage2_FrameOutput* output
);
#endif

/**
 * @brief 取得 bbox 寬度（inclusive）。
 *
 * @param bbox [in] bbox 指標
 * @return int 寬度 \(x2-x1+1\)。若 bbox 無效則回傳 0。
 */
int stage2_bbox_width(const FD_BoundingBox* bbox);

/**
 * @brief 取得 bbox 高度（inclusive）。
 *
 * @param bbox [in] bbox 指標
 * @return int 高度 \(y2-y1+1\)。若 bbox 無效則回傳 0。
 */
int stage2_bbox_height(const FD_BoundingBox* bbox);

/**
 * @brief 取得 bbox 面積（inclusive）。
 *
 * @param bbox [in] bbox 指標
 * @return int 面積 \(w*h\)。若 bbox 無效則回傳 0。
 */
int stage2_bbox_area(const FD_BoundingBox* bbox);

/**
 * @brief 取得 bbox 中心 x（整數）。
 *
 * @param bbox [in] bbox 指標
 * @return int center x = (x1+x2)/2
 */
int stage2_bbox_cx(const FD_BoundingBox* bbox);

/**
 * @brief 取得 bbox 中心 y（整數）。
 *
 * @param bbox [in] bbox 指標
 * @return int center y = (y1+y2)/2
 */
int stage2_bbox_cy(const FD_BoundingBox* bbox);

#ifdef STAGE2_ENABLE_DEBUG
#include <stdio.h>

/**
 * @brief 寫出 `stage2_tracks.csv` 的 header 列。
 *
 * @param fp [in,out] 已開啟的檔案指標（w 模式）。
 */
void stage2_debug_write_tracks_csv_header(FILE* fp);

/**
 * @brief 將某一幀的 tracks snapshot 寫入 `stage2_tracks.csv`（一 track 一列）。
 *
 * @param fp     [in,out] 已開啟的檔案指標（a / w 皆可）。
 * @param output [in]     stage2_update 的本幀輸出。
 */
void stage2_debug_write_tracks_csv_frame(FILE* fp, const Stage2_FrameOutput* output);

/**
 * @brief 寫出 `stage2_cnn_candidates.csv` 的 header 列。
 *
 * @param fp [in,out] 已開啟的檔案指標（w 模式）。
 */
void stage2_debug_write_candidates_csv_header(FILE* fp);

/**
 * @brief 將某一幀的 CNN candidates 寫入 `stage2_cnn_candidates.csv`（一 candidate 一列）。
 *
 * @param fp     [in,out] 已開啟的檔案指標（a / w 皆可）。
 * @param output [in]     stage2_update 的本幀輸出。
 */
void stage2_debug_write_candidates_csv_frame(FILE* fp, const Stage2_FrameOutput* output);
#endif

#ifdef __cplusplus
}
#endif

#endif // STAGE2_H
