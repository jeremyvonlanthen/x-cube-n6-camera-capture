/**
 ******************************************************************************
 * @file    app_record.c
 * @brief   H264 -> MP4 / JPEG snapshot recording to microSD.
 ******************************************************************************
 */
#include "app_record.h"
#include "app_shared.h"
#include "app_config.h"
#include "utils.h"

#include <assert.h>
#include <stdio.h>
#include "app_cam.h"
#include "app_jpg.h"
#include "app_rec.h"
#include "app_enc.h"
#include "cmw_camera.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_dcmipp.h"
#include "stm32n6xx_ll_venc.h"
#include "FreeRTOS.h"
#include "task.h"

/* H264 recording configuration (module-private).
 * The video resolution (4:3) is passed to record_camera_setup()/record_h264_run()
 * as their 'height' argument; the width is derived (height * 4 / 3).  It must stay
 * <= H264_MAX_HEIGHT: the VENC/EWL encoder pools (app_enc.c) and
 * buffer_full_frame (2 capture frames + ring) are sized for that maximum. */
#define H264_FPS              25
#define H264_VENC_OUT_SIZE    (1024 * 1024)  /* 1 MB: holds a full 1080p keyframe */
#define H264_AE_WARMUP_FRAMES 10
#define H264_MAX_HEIGHT       1080     /* do not exceed: pools sized for this */

/* VENC hardware output buffer (module-private) */
static uint8_t h264_venc_out[H264_VENC_OUT_SIZE] ALIGN_32 IN_PSRAM;

/* ==========================================================================
 * H264 -> MP4 / JPEG recording to microSD
 * ========================================================================== */

/* Encodes one frame.  Returns encoded byte count, or 0 if dropped. */
static size_t h264_encode_frame(uint8_t *p_frame, int is_intra_force)
{
  static int enc_call_count = 0;
  size_t res;

  res = ENC_EncodeFrame(p_frame, h264_venc_out, H264_VENC_OUT_SIZE, is_intra_force);
  enc_call_count++;

  if ((int)res <= 0)
    return 0;

  /* VENC hardware wrote to PSRAM — invalidate CPU D-cache before reading */
  SCB_InvalidateDCache_by_Addr((uint32_t *)h264_venc_out, CACHE_ALIGN_SIZE((int)res));

  return res;
}

/* Takes one snapshot (camera is still in the post-warmup configuration),
 * encodes it to JPEG (hardware) and saves it to the SD card as
 * <timestamp>.jpg through the FreeRTOS SD writer task (app_rec.c).
 * Called in RECORD_MODE_WARMUP, right before record_camera_setup().
 *   height : 4:3 photo height (width derived); up to SENSOR_HEIGHT (full res). */
void record_jpeg_sd(const char *timestamp, int height)
{
  int width = height * 4 / 3;      /* 4:3, full-scene downscale from sensor */
  char fname[40];
  int jpeg_len;
  uint32_t start;

  snprintf(fname, sizeof(fname), "%s.jpg", timestamp);

  /* COLOR snapshot while the camera runs in detect (mono, cropped/downsized)
   * mode: reconfigure PIPE1 ONLY to a full-scene width x height YUV422
   * downscale (ROI = full sensor).  The sensor is untouched, so the
   * AE/exposure converged during the detect warmup stay valid -> no delay,
   * color is immediate.  No restore needed: record_camera_setup() reconfigures
   * the camera right after, and DETECT_MODE_WARMUP re-applies the detect setup
   * once the recording is done. */
  CAM_Pipe1_SetFormat(SENSOR_WIDTH, SENSOR_HEIGHT,
                      width, height, DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1);

  /* One snapshot into buffer_full_frame (same flow as capture_yuv) */
  snapshot_in_progress = 1;
  frame_ready = 0;
  CAM_CapturePipe_Start(buffer_full_frame, buffer_warmup, CMW_MODE_SNAPSHOT, 0);

  start = HAL_GetTick();
  while (!frame_ready) {
    if (HAL_GetTick() - start > 5000) {
      snapshot_in_progress = 0;
      printf("[REC] snapshot capture timeout\r\n");
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  snapshot_in_progress = 0;

  {
    int32_t je = 0, jg = 0;
    CMW_CAMERA_GetExposure(&je);
    CMW_CAMERA_GetGain(&jg);
    printf("[REC] jpeg AE: exp=%ld gain=%ld\r\n", (long)je, (long)jg);
  }

  SCB_InvalidateDCache_by_Addr((uint32_t *)buffer_full_frame, CACHE_ALIGN_SIZE(MAX_CAPTURE_FRAME_SIZE));

  /* Hardware JPEG encode: pipe1 was switched to width x height above */
  jpg_conf.width      = width;
  jpg_conf.height     = height;
  jpg_conf.fmt_src    = JPG_SRC_YUV422; //JPG_SRC_GREY;
  jpg_conf.full_width = width;
  JPG_Init(&jpg_conf);
  jpeg_len = JPG_Encode(hires_jpeg_buffer, buffer_full_frame,
                        MAX_JPEG_FRAME_SIZE, MAX_CAPTURE_FRAME_SIZE);
  SCB_CleanDCache_by_Addr((uint32_t *)hires_jpeg_buffer, CACHE_ALIGN_SIZE(jpeg_len));
  JPG_Deinit();

  if (jpeg_len <= 0) {
    printf("[REC] JPG encode failed (%d)\r\n", jpeg_len);
    return;
  }

  /* Written by the SD writer task (FreeRTOS); blocks until file closed */
  if (REC_SaveJpeg(hires_jpeg_buffer, (size_t)jpeg_len, fname) != 0)
    printf("[REC] snapshot save FAILED\r\n");
}

/* Prepares the camera for H264 recording: reconfigures to (height*4/3) x height
 * RGB565 (full-scene downscale), (re)inits the VENC + H264 encoder once, starts
 * the double-buffered capture and lets the AE settle.  Called in
 * RECORD_MODE_WARMUP, right before record_h264_run(); leaves the double-buffered
 * capture running for it.
 *   height : 4:3 video height, must be <= H264_MAX_HEIGHT (width is derived). */
void record_camera_setup(int height)
{
  /* LL_VENC_Init and ENC_Init must each be called exactly once —
   * ENC_DeInit crashes on this target.  Init once on first entry,
   * reuse on every subsequent call (same pattern as the USB phase). */
  static int hw_initialized = 0;

  int width = height * 4 / 3;                                 /* 4:3 */
  uint32_t frame_bytes = (uint32_t)width * (uint32_t)height * 2u; /* RGB565 */
  CAM_conf_t cam_conf = { 0 };
  ENC_Conf_t enc_conf;

  assert(height <= H264_MAX_HEIGHT);  /* encoder pools sized for this max */

  /* Switch camera to width x height RGB565 @ H264_FPS for H264 (full-scene
   * downscale from the sensor).
   * RGB565 (2 B/px) halves PSRAM bandwidth vs ARGB8888: fixes DCMIPP
   * pixel-packer overruns (right-side line artifacts).  Encoder preproc
   * is set to H264ENC_RGB565 accordingly (app_enc.c). */
  /* Reuse the exposure/gain converged before the reconfig as a seed, so the
   * AE warmup below starts near-correct instead of from a dark default. */
  int32_t seed_exp = 0, seed_gain = 0;
  CMW_CAMERA_GetExposure(&seed_exp);
  CMW_CAMERA_GetGain(&seed_gain);

  CAM_Deinit();
  cam_conf.capture_width        = width;
  cam_conf.capture_height       = height;
  cam_conf.fps                  = H264_FPS;
  cam_conf.dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1;
  cam_conf.is_rgb_swap          = 0;
  CAM_Init(&cam_conf, 0);

  /* Seed the freshly-reset AE (CAM_Init resets exposure/gain to defaults). */
  if (seed_exp  > 0) CMW_CAMERA_SetExposure(seed_exp);
  if (seed_gain > 0) CMW_CAMERA_SetGain(seed_gain);

  if (!hw_initialized) {
    /* VENC hardware — assert-fails if called twice */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    LL_VENC_Init();

    /* H264 software encoder */
    ENC_ResetAllocator();
    enc_conf.width  = width;
    enc_conf.height = height;
    enc_conf.fps    = H264_FPS;
    ENC_Init(&enc_conf);

    hw_initialized = 1;
  } else {
    /* Encoder hardware reused: reset pic_cnt so IDR counting restarts.
     * is_sps_pps_done was already cleared by ENC_EndSession() at the
     * end of the previous session, so H264EncStrmStart will fire on the
     * first ENC_EncodeFrame and emit fresh SPS/PPS for the new file. */
    ENC_ResetSession();
  }

  force_intra       = 0;
  h264_frame_ready  = 0;
  h264_ready_buf    = buffer_full_frame;

  /* Start double-buffered continuous capture (two 720p frames inside
   * buffer_full_frame).  Single-buffer capture caused tearing artifacts on
   * the right side of the image: VENC was reading the frame while DCMIPP
   * was still overwriting it. */
  h264_streaming = 1;
  {
    int ret = CMW_CAMERA_DoubleBufferStart(DCMIPP_PIPE1,
                                           buffer_full_frame,
                                           buffer_full_frame + frame_bytes,
                                           CMW_MODE_CONTINUOUS);
    assert(ret == CMW_ERROR_NONE);
  }

  /* Let the auto-exposure/ISP reconverge before recording: skip the first
   * frames (they are under-exposed right after the camera reconfig). */
  {
    uint32_t skipped = 0;
    uint32_t t0 = HAL_GetTick();
    while (skipped < H264_AE_WARMUP_FRAMES && HAL_GetTick() - t0 < 2000) {
      if (h264_frame_ready) {
        h264_frame_ready = 0;
        skipped++;
      } else {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }
  }
  {
    int32_t conv_exp = 0, conv_gain = 0;
    CMW_CAMERA_GetExposure(&conv_exp);
    CMW_CAMERA_GetGain(&conv_gain);
    printf("[REC] video AE: seed exp=%ld gain=%ld -> converged exp=%ld gain=%ld\r\n",
           (long)seed_exp, (long)seed_gain, (long)conv_exp, (long)conv_gain);
  }
}

/* Records rec_duration seconds of H264 video into <timestamp>.mp4 using the
 * double-buffered capture already started by record_camera_setup().  Opens the
 * MP4, runs the capture->encode loop, finalizes the file, and stops the capture.
 * Called in RECORDING, right after RECORD_MODE_WARMUP.
 *   height : must match the value passed to record_camera_setup(). */
void record_h264_run(const char *timestamp, int height, int rec_duration)
{
  int width = height * 4 / 3;                                 /* 4:3 */
  uint32_t frame_bytes = (uint32_t)width * (uint32_t)height * 2u; /* RGB565 */
  char fname[40];

  snprintf(fname, sizeof(fname), "%s.mp4", timestamp);

  /* Open the MP4 file and start the muxer.  Ring buffer lives in the unused
   * part of buffer_full_frame (after the 2 capture frames) -> several seconds
   * of encoded video, absorbing SD latency spikes. */
  if (REC_Start(width, height, H264_FPS,
                buffer_full_frame + 2 * frame_bytes,
                MAX_CAPTURE_FRAME_SIZE - 2 * frame_bytes,
                fname) != 0) {
    printf("[REC] record start failed, recording aborted\r\n");
    /* Stop the capture started by record_camera_setup(). */
    h264_streaming = 0;
    CLEAR_BIT(hcamera_dcmipp.Instance->P1PPCR, DCMIPP_P1PPCR_DBM);
    warmup_done = 0;
    return;
  }

  printf("[REC] recording started (%d sec @ %d fps)...\r\n", rec_duration, H264_FPS);

  /* Record for rec_duration seconds */
  uint32_t start_tick = HAL_GetTick();
  uint32_t last_frame_tick = start_tick;
  uint32_t frame_count = 0;
  uint32_t encode_ok_count = 0;

  printf("[REC] video started %d ms after movement detection\r\n", (int)(start_tick-actual_ticks));

  while (HAL_GetTick() - start_tick < (uint32_t)(rec_duration * 1000)) {
    if (!h264_frame_ready) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    h264_frame_ready = 0;
    frame_count++;

    /* First frame is always IDR+SPS/PPS. */
    {
      int force_idr = (frame_count == 1) || force_intra;
      /* snapshot: the completed buffer (the other one is being written) */
      uint8_t *p_frame = h264_ready_buf;
      size_t len = h264_encode_frame(p_frame, force_idr);
      force_intra = 0;
      if (len > 0) {
        encode_ok_count++;
        /* Real measured frame duration (variable frame rate): keeps the
         * MP4 duration equal to wall-clock time even if the sensor is not
         * exactly at 30 fps or if frames are skipped/dropped. */
        uint32_t now = HAL_GetTick();
        uint32_t dur90k = (now - last_frame_tick) * 90u;
        last_frame_tick = now;
        if (dur90k == 0u || dur90k > 90000u)
          dur90k = 0u; /* aberrant delta -> fall back to nominal 1/fps */
        /* Queue for the SD writer task.  If the ring is full (SD latency
         * spike), the frame is dropped: force an IDR so the decoder can
         * resynchronize on the next frame. */
        if (REC_PushFrame(h264_venc_out, len, dur90k) != 0)
          force_intra = 1;
      }
    }
  }
  printf("[REC] recording states: frames=%lu encOK=%lu dcmippErr=%lu\r\n",
         frame_count, encode_ok_count, (unsigned long)dcmipp_err_count);

  /* Stop the capture->encode pipeline */
  h264_streaming = 0;

  /* Disable hardware double-buffer mode (never cleared by the HAL) so the
   * next single-buffer session (config/detect warmup) starts clean. */
  CLEAR_BIT(hcamera_dcmipp.Instance->P1PPCR, DCMIPP_P1PPCR_DBM);

  /* End encoder session: H264EncStrmEnd transitions ENCODING→INIT so that
   * H264EncStrmStart (called on next session's first encode) emits fresh
   * SPS/PPS for the next file.
   * NOTE: H264EncRelease must NOT be called — it crashes on this target.
   *       H264EncStrmEnd is safe and is the correct way to close a stream. */
  ENC_EndSession(h264_venc_out, H264_VENC_OUT_SIZE);

  /* Flush pending frames and finalize the MP4 (writes the moov index).
   * Blocks until the SD writer task is done. */
  if (REC_Stop() == 0)
    printf("[REC] mp4 file finalized ok\r\n");
  else
    printf("[REC] mp4 finalize failed\r\n");

  /* Re-enter current mode from scratch */
  warmup_done = 0;
}

