#pragma once

#include "esp_err.h"

#include "node_upload.h"
#include "rec_session.h"

typedef enum {
    NODE_STATE_BOOTING = 0,
    NODE_STATE_IDLE,
    NODE_STATE_RECORDING,
    NODE_STATE_UPLOADING,
} node_state_t;

/* Start the 1-minute heartbeat, which also acts as the Wi-Fi watchdog. */
esp_err_t node_health_start(void);
void node_health_set_state(node_state_t state);
void node_health_note_session(const rec_session_result_t *result);
void node_health_note_upload(const node_upload_summary_t *summary);
