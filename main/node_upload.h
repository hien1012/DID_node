#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

typedef struct {
    size_t uploaded_count;
    size_t failed_count;
    size_t rejected_count;      /* server refused for good; moved to broken/ */
    size_t uploaded_bytes;
    bool deferred;              /* stopped early so the next session can start */
    esp_err_t last_error;
} node_upload_summary_t;

/*
 * Upload every file in done/, oldest first, deleting each one after a verified
 * ACK. No new file is started if it would not finish before deadline_epoch
 * (0 = no deadline). Stops at the first transport failure.
 */
esp_err_t node_upload_process_pending(time_t deadline_epoch,
                                      node_upload_summary_t *summary);

/* Remember where the first frame/sample landed for the latest session, so its
 * uploads can carry X-Start-Offset-US. Older backlog files are sent without it. */
void node_upload_note_session_offsets(time_t session_epoch,
                                      int64_t camera_offset_us,
                                      int64_t mic_offset_us);
