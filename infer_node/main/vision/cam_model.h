#pragma once

/*
 * 影像 ROI CNN (FD_did_cam_v4/main/model.cc)：輸入 64x64 uint8 灰階，輸出 3 類 softmax。
 * interpreter 與 arena 是單例，只由 cam_infer task 呼叫，不可重入。
 */
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_MODEL_ROI_SIZE     64
#define CAM_MODEL_ROI_BYTES    (CAM_MODEL_ROI_SIZE * CAM_MODEL_ROI_SIZE)
#define CAM_MODEL_CLASS_COUNT  3
/* 計時最多記幾層；目前的模型有 23 層 */
#define CAM_MODEL_MAX_OPS      28U

typedef struct {
    const char *name;   /* TFLite op 名稱，常數字串 */
    uint32_t total_us;  /* 上次取走以來，這一層的累計耗時 */
} cam_model_op_time_t;

esp_err_t cam_model_init(void);

/**
 * @param roi 64x64 灰階 (row-major)
 * @param probabilities 反量化後的機率，類別順序 drone / bird / background
 * @param predicted_class 與原專案相同的判定：drone 機率超過門檻就判 drone，否則取最大值
 */
esp_err_t cam_model_predict(const uint8_t *roi, float probabilities[CAM_MODEL_CLASS_COUNT],
                            int *predicted_class);

/** 上一次 cam_model_predict() 裡 Invoke() 本身的時間 (不含複製輸入、反量化)；失敗時是 0。 */
uint32_t cam_model_last_invoke_us(void);

/**
 * @brief 取走每一層的累計耗時並歸零。耗時是實際經過時間，包含被其他 task 或中斷插隊的時間。
 * @param ops 依執行順序填入
 * @param invokes 這段期間推論了幾次
 * @return 層數
 */
size_t cam_model_take_op_times(cam_model_op_time_t ops[CAM_MODEL_MAX_OPS], uint32_t *invokes);

const char *cam_model_class_name(int cls);

#ifdef __cplusplus
}
#endif
