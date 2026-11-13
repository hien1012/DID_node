#pragma once

#include "esp_err.h"

#include "sample_upload.h"

typedef enum {
    INFER_STATE_BOOTING = 0,
    INFER_STATE_RUNNING,
    INFER_STATE_UPLOADING,
} infer_state_t;

/* 每分鐘 heartbeat，兼 Wi-Fi watchdog (與 did_node 的 node_health 相同規則)。 */
esp_err_t infer_health_start(void);
void infer_health_set_state(infer_state_t state);
void infer_health_note_upload(const sample_upload_summary_t *summary);
