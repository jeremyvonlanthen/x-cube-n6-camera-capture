/**
 ******************************************************************************
 * @file    app_detect.h
 * @brief   Statistical + frame-diff motion detector on the two detect-mode
 *          DCMIPP pipes (MVE-accelerated), ported from TB_LDS/Src/app.c
 *          (DETECTION state).
 ******************************************************************************
 */
#ifndef APP_DETECT_H
#define APP_DETECT_H

#include <stdbool.h>

/* Allocates the per-pixel statistics buffers (mean/var/std/... for both
 * pipes) from the shared axisram_alloc pool and resets the detector's
 * frame counter. Call once per dcmipp_apply_detect_config() cycle, right
 * after it (DETECT_MODE_WARMUP) -- reuses the same bump-allocation pass as
 * the pipe capture buffers it allocates. */
void DETECT_Init(void);

/* Blocking calibration pass (31 captures + final mean/variance pass):
 * initializes the running mean/std used by DETECT_ProcessFrame(). Call once
 * per DETECT_MODE_WARMUP entry, right after DETECT_Init(). */
void DETECT_CalibrateStats(void);

/* One detect cycle: captures a pipe1+pipe2 snapshot (capture_detect_frame)
 * and runs the movement/statistical-outlier detector on both pipes,
 * adjusting the running mean/std as it goes. Returns true if movement was
 * detected on either pipe.
 *   pct_pipe1/pct_pipe2 : filled with the percentage (0-100) of each pipe's
 *     pixels flagged by the detector (post neighbour-count filtering) for
 *     this frame -- regardless of whether is_detect ended up true. */
bool DETECT_ProcessFrame(float *pct_pipe1, float *pct_pipe2);

#endif /* APP_DETECT_H */
