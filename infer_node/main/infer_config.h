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
#define INFER_NODE_ROLE               INFER_ROLE_CAM

#if INFER_NODE_ROLE != INFER_ROLE_CAM && INFER_NODE_ROLE != INFER_ROLE_MIC
#error "INFER_NODE_ROLE must be INFER_ROLE_CAM or INFER_ROLE_MIC"
#endif

#define INFER_CAM_ENABLE              (INFER_NODE_ROLE == INFER_ROLE_CAM)
#define INFER_MIC_ENABLE              (INFER_NODE_ROLE == INFER_ROLE_MIC)

/* 預測類別的機率低於此值 = 低信賴度，原始資料存 SD。 */
#define INFER_LOW_CONF_THRESHOLD      0.6f

/*
 * 除了低信賴度，預測為「非背景」的也存 SD 並上傳 (audio：drone / helicopter；video：drone / bird)。
 *   0：只存低信賴度 (原本的行為)。
 *   1：低信賴度 或 非背景，任一成立就存。video 只要一幀內有一個 ROI 非背景就存整幀。
 * 偵測多時樣本量會大增 (audio 每視窗 64 KB、video 每幀約 300 KB)；待上傳數達
 * INFER_MAX_PENDING_SAMPLES 後不再存新的。
 * 背景類別在開機時以名稱 INFER_BACKGROUND_LABEL 從模型的類別表找出 (不寫死索引：
 * audio 的 background 是 0，video 是 2)；找不到時這個開關無效，只存低信賴度。
 * 樣本 meta 的 save_reason 記錄為什麼存："low_conf" / "non_background" / "both"。
 * 與 ESP32 版 (ESP32_cam_mic/infer_node) 相同的設計。
 */
#ifndef INFER_SAVE_NON_BACKGROUND
#define INFER_SAVE_NON_BACKGROUND     1
#endif
#define INFER_BACKGROUND_LABEL        "background"

/* 同一種 media 累積這麼多個樣本才上傳 (音訊、影像分開計算)。 */
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
 * 再逐 byte 掃描所有 task 的剩餘 stack (約 1–2 ms)。ESP32 版 (ESP32_cam_mic/infer_node) 因此
 * 漏掉相機 DMA 的 EOF 中斷，約 10–20% 的幀被丟掉 (Camera: incomplete)，關閉後降到 0 (2026-11-16)。
 * S3 的相機 DMA 架構不同，未驗證是否受影響，預設一樣關閉。
 * cam_infer 每次推論的 run / preempt / fd_overlap 不受影響 (只讀自己的計數器，不掃 stack)。
 */
#ifndef INFER_TASK_USAGE_ENABLE
#define INFER_TASK_USAGE_ENABLE       0
#endif

/* 每幀最多送幾個 ROI 進 CNN，避免一張雜訊幀佔住 fb 太久；超過的留到該 track 下一次配對。 */
#define INFER_CAM_MAX_ROI_PER_FRAME   4U

/*
 * 麥克風 PCM 的軟體高通 (mic_infer.c；I2S 與 PDM 都適用，模型輸入合約的一部分)：
 *   0 (預設)：不濾波，與 TTGO_I2S_Record_Test 訓練資料的取法相同。
 *   1：在 × PCM_GAIN (30) 之前加一階高通 (-3 dB @ INFER_MIC_HPF_CUTOFF_HZ)，再 × 30 飽和。
 *      目的：DC 偏移與緩慢漂移 (PDM 給時脈後的數秒特別明顯) 不被 × 30 放大到削波。
 *      做法同 ESP32 版 (ESP32_cam_mic/infer_node 的 MIC_PDM_16BIT_HPF，該版只用於 PDM)。
 * 存下的 WAV 是濾波後的資料 (就是推論用的那份)，SD 重播 (debug_replay.c) 讀 WAV，不會再濾一次。
 * mel 從 250 Hz 起，24 Hz 一階高通在 250 Hz 只衰減約 0.04 dB，對特徵幾乎沒有影響；
 * 但與訓練資料的取法不同，切換後要確認模型輸出是否合理。
 * 也可以不改這裡：idf.py -B build_hpf -DINFER_MIC_HPF_ENABLE=1 build flash
 */
#ifndef INFER_MIC_HPF_ENABLE
#define INFER_MIC_HPF_ENABLE          0
#endif
#define INFER_MIC_HPF_CUTOFF_HZ       24.0f /* 同 ESP32 版，對齊 ICS-43434 內建高通 (DS-000069 p.14) */

/*
 * 音訊除錯 dump (audio/audio_debug.c)：
 *   0：關閉 (預設)。
 *   1：正常推論照常運作；audio_cnn 對開機後前 INFER_AUDIO_DEBUG_DUMP_WINDOWS 個視窗，
 *      在正式推論後把該視窗的 PCM (input.wav)、正式特徵與結果、每個前處理階段、CNN 每一層
 *      以 base64 文字行 (@@DUMP / @@FILE / @@D) 從 console 輸出，不寫 SD。
 *      115200 baud 每個視窗約 40 s (FULL_FRONTEND=1 約 75 s)，期間錄音端會丟掉視窗
 *      ("All 3 windows busy" 警告)，之後恢復正常。
 *      PC 端：TTGO_I2S_Record_Test/tools/receive_infer_node_dump.py 接收，
 *            DID_NODE/tools/compare_debug_dumps.py 比對 (含 Python 訓練前處理；
 *            TTGO_I2S_Record_Test/compare_infer_node_debug_dump.py 是舊版，PC 前處理用 gcc 編的同一份 C)。
 * 也可以不改這裡：idf.py -B build_debug -DINFER_AUDIO_DEBUG_DUMP=1 -DESP_NN_KERNEL_SET=all build flash
 * (ESP_NN_KERNEL_SET 要與現場韌體相同，否則 CNN 用的 kernel 不一樣)。
 */
#ifndef INFER_AUDIO_DEBUG_DUMP
#define INFER_AUDIO_DEBUG_DUMP        0
#endif
#ifndef INFER_AUDIO_DEBUG_DUMP_WINDOWS
#define INFER_AUDIO_DEBUG_DUMP_WINDOWS 1U
#endif
/* 1：另外輸出去 DC 音訊與 power spectrum (多約 255 KB)；0 只輸出 mel 以後的階段 */
#ifndef INFER_AUDIO_DEBUG_DUMP_FULL_FRONTEND
#define INFER_AUDIO_DEBUG_DUMP_FULL_FRONTEND 0
#endif

/*
 * 相機除錯 dump (vision/cam_debug.c)，格式與音訊 dump 相同：
 *   0：關閉 (預設)。
 *   1：正常推論照常運作；cam_infer 對開機後前 INFER_CAM_DEBUG_DUMP_ROIS 個 ROI，在正式推論後
 *      把 ROI (roi.gray，64x64 uint8)、正式結果、正式 interpreter 重跑一次的結果、CNN 每一層
 *      從 console 輸出。115200 baud 每個 ROI 約 20 s，期間 CNN 停住 (FD 與取幀照常)，之後恢復。
 *      要有 FD 候選才會觸發：在鏡頭前揮手或移動物體。
 *      PC 端：TTGO_I2S_Record_Test/tools/receive_infer_node_dump.py -o cam_dump 接收，
 *            DID_NODE/tools/compare_debug_dumps.py --dump-dir cam_dump 比對。
 * 也可以不改這裡：idf.py -B build_cam_debug -DINFER_CAM_DEBUG_DUMP=1 -DESP_NN_KERNEL_SET=all build flash
 * (INFER_NODE_ROLE 要是 INFER_ROLE_CAM；ESP_NN_KERNEL_SET 是要驗證的 kernel 組合)。
 */
#ifndef INFER_CAM_DEBUG_DUMP
#define INFER_CAM_DEBUG_DUMP          0
#endif
#ifndef INFER_CAM_DEBUG_DUMP_ROIS
#define INFER_CAM_DEBUG_DUMP_ROIS     1U
#endif

/*
 * SD 重播除錯 (debug_replay.c)：
 *   0：關閉 (預設)。
 *   1：開機 (Wi-Fi、校時、pipeline init、SD 掛載) 後不啟動即時推論，改讀 SD 的 debug/in/：
 *      相機節點讀 cam_<名稱>.gray (64x64 uint8)，麥克風節點讀 mic_<名稱>.wav (16 kHz mono PCM16, ≥2 s)，
 *      逐一用正式 interpreter 推論後，以與上面相同的格式從 console 輸出每一層；dump 名稱為
 *      <檔名>_<ESP_NN_KERNEL_SET>。重播完板子停住 (不回到即時推論)。
 *      同一組輸入用 ESP_NN_KERNEL_SET=all 與 none 各燒一次、各收一次，放進同一個資料夾後執行
 *      DID_NODE/tools/compare_debug_dumps.py，即可四方比對 (node ESP-NN / node reference / PC reference /
 *      PC Python)。輸入檔可用 DID_NODE/tools/make_debug_inputs.py 產生。
 * 用法 (不用改這裡)：
 *   idf.py -B build_replay_nn  -DINFER_DEBUG_REPLAY=1 -DESP_NN_KERNEL_SET=all  build flash
 *   idf.py -B build_replay_ref -DINFER_DEBUG_REPLAY=1 -DESP_NN_KERNEL_SET=none build flash
 */
#ifndef INFER_DEBUG_REPLAY
#define INFER_DEBUG_REPLAY            0
#endif
