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
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "app_cam.h"
#include "app_jpg.h"
#include "app_rec.h"
#include "app_enc.h"
#include "app_uart.h"
#include "app_watchdog.h"
#include "cmw_camera.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_dcmipp.h"
#include "stm32n6xx_ll_venc.h"
#include "FreeRTOS.h"
#include "task.h"

/* H264 recording configuration (module-private).
 * The video resolution (4:3) is passed to setup_record_h264()/
 * record_h264_to_ram() as their 'height' argument; the width is
 * derived (height * 4 / 3).  It must stay
 * <= H264_MAX_HEIGHT: the VENC/EWL encoder pools (app_enc.c) and
 * buffer_full_frame (2 capture frames + ring) are sized for that maximum. */
/* 15 fps @ 720p (not the H264_MAX_HEIGHT of 1080p): a 1440x1080/25fps clip
 * measured ~15.7 Mbit/s on hardware, which didn't fit an 8-second clip in
 * PSRAM (see H264_RAM_STORE_SIZE below). 960x720/15fps measures ~4.15 Mbit/s,
 * comfortably fitting a full 15-second clip with margin to spare. */
#define H264_FPS              20
#define H264_VENC_OUT_SIZE    (1024 * 1024)  /* 1 MB: holds a full 1080p keyframe */
#define H264_AE_WARMUP_FRAMES 10
#define H264_MAX_HEIGHT       1080     /* do not exceed: pools sized for this */

/* VENC hardware output buffer (module-private) */
static uint8_t h264_venc_out[H264_VENC_OUT_SIZE] ALIGN_32 IN_PSRAM;

/* Length of the JPEG in hires_jpeg_buffer, filled by record_snapshot_to_ram()
 * and consumed by record_snapshot_flush_to_sd() -- mirrors the h264_ram_*
 * state below for the video path. */
static int snapshot_jpeg_len;

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

/* Takes one snapshot (camera is still in the post-warmup configuration) and
 * encodes it to JPEG (hardware) into hires_jpeg_buffer -- no SD access here;
 * record_snapshot_flush_to_sd() writes it out once the card is mounted.
 * Called in RECORD_MODE_INIT.
 *   height : 4:3 photo height (width derived); up to SENSOR_HEIGHT (full res).
 *   exposure_us/gain_mdb : forwarded to send_img_uart() as-is (read once by
 *     the caller, avoids a redundant CMW_CAMERA_GetExposure/GetGain here).
 * Returns the encoded length (> 0), or <= 0 on capture/encode failure. */
int record_snapshot_to_ram(int height, int32_t exposure_us, int32_t gain_mdb)
{
  int width = height * 4 / 3;      /* 4:3, full-scene downscale from sensor */
  int jpeg_len;
  uint32_t start;

  /* MONO snapshot while the camera runs in detect (mono, cropped/downsized)
   * mode: reconfigure PIPE1 ONLY to a full-scene width x height MONO
   * downscale (ROI = full sensor).  The sensor is untouched, so the
   * AE/exposure converged during the detect warmup stay valid -> no delay.
   * No restore needed: DETECT_MODE_WARMUP re-applies the detect setup once
   * the record cycle is done (setup_record_h264() reconfigures pipe1 again
   * first if this turns out to be a video). */
  CAM_Pipe1_SetFormat(SENSOR_WIDTH, SENSOR_HEIGHT,
                      width, height, DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1);

  /* One snapshot into buffer_full_frame (same flow as capture_img) */
  snapshot_in_progress = true;
  frame_ready = false;
  CAM_CapturePipe_Start(buffer_full_frame, buffer_warmup, CMW_MODE_SNAPSHOT, 0);

  start = HAL_GetTick();
  while (!frame_ready) {
    if (HAL_GetTick() - start > 5000) {
      snapshot_in_progress = false;
      printf("[REC] snapshot capture timeout\r\n");
      return -1;
    }
    watchdog_kick();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  snapshot_in_progress = false;

  /* Let the CSI/D-PHY link settle after this PIPE1 snapshot before the
   * caller potentially tears the camera down (VIDEO_CAPTURE -> setup_record_h264()
   * -> CAM_Deinit()+CAM_Init() right after this call, with no other delay in
   * between). camera_warmup() always inserts this same 50 ms settle after
   * its own PIPE1 stop; without it here, a video recording that follows a
   * detection cycle (the 2nd+ one in a session) can hit a DCMIPP D-PHY
   * relock error that leaves hcamera_dcmipp.State != READY, failing the
   * "ret == HAL_OK" assert in DCMIPP_PipeInitCapture (app_cam.c). */
  vTaskDelay(pdMS_TO_TICKS(50));

  SCB_InvalidateDCache_by_Addr((uint32_t *)buffer_full_frame, CACHE_ALIGN_SIZE(MAX_CAPTURE_FRAME_SIZE));

  /* Hardware JPEG encode: pipe1 was switched to width x height above */
  jpg_conf.width      = width;
  jpg_conf.height     = height;
  jpg_conf.fmt_src    = JPG_SRC_GREY;
  jpg_conf.full_width = width;
  JPG_Init(&jpg_conf);
  jpeg_len = JPG_Encode(hires_jpeg_buffer, buffer_full_frame,
                        MAX_JPEG_FRAME_SIZE, MAX_CAPTURE_FRAME_SIZE);
  SCB_CleanDCache_by_Addr((uint32_t *)hires_jpeg_buffer, CACHE_ALIGN_SIZE(jpeg_len));
  JPG_Deinit();

  if (jpeg_len <= 0)
    printf("[REC] JPG encode failed (%d)\r\n", jpeg_len);
  else
    /* Unsolicited push to the GUI (same 0xAA-framed protocol as capture_img()
     * / send_img_uart(), just not preceded by a GUI-sent 'S' this time): lets
     * the user see, live, what triggered the movement detection. At 10 MBaud
     * this blocks for well under a second even for a large JPEG -- no
     * meaningful delay to the VIDEO_CAPTURE that follows. */
    send_img_uart(hires_jpeg_buffer, jpeg_len, exposure_us, gain_mdb);

  snapshot_jpeg_len = jpeg_len;
  return jpeg_len;
}

/* Writes the JPEG captured by record_snapshot_to_ram() to fname on the SD
 * card, through REC_SaveFile (FreeRTOS SD writer task). Called in
 * MULTIMEDIA_STORAGE, once SD_CARD_INIT has succeeded.
 * Returns 0 on success. */
int record_snapshot_flush_to_sd(const char *fname)
{
  if (snapshot_jpeg_len <= 0)
    return -1;

  return REC_SaveFile(hires_jpeg_buffer, (size_t)snapshot_jpeg_len, fname);
}

/* Prepares the camera for H264 recording: reconfigures to (height*4/3) x height
 * RGB565 (full-scene downscale), (re)inits the VENC + H264 encoder once, starts
 * the double-buffered capture and lets the AE settle.  Called in
 * VIDEO_CAPTURE, right before record_h264_to_ram(); leaves the
 * double-buffered capture running for it.
 *   height : 4:3 video height, must be <= H264_MAX_HEIGHT (width is derived). */
void setup_record_h264(int height)
{
  /* LL_VENC_Init and ENC_Init must each be called exactly once —
   * ENC_DeInit crashes on this target.  Init once on first entry,
   * reuse on every subsequent call (same pattern as the USB phase). */
  static bool hw_initialized = false;

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

    hw_initialized = true;
  } else {
    /* Encoder hardware reused: reset pic_cnt so IDR counting restarts.
     * is_sps_pps_done was already cleared by ENC_EndSession() at the
     * end of the previous session, so H264EncStrmStart will fire on the
     * first ENC_EncodeFrame and emit fresh SPS/PPS for the new file. */
    ENC_ResetSession();
  }

  force_intra       = false;
  h264_frame_ready  = false;
  h264_ready_buf    = buffer_full_frame;

  /* Start double-buffered continuous capture (two 720p frames inside
   * buffer_full_frame).  Single-buffer capture caused tearing artifacts on
   * the right side of the image: VENC was reading the frame while DCMIPP
   * was still overwriting it. */
  h264_streaming = true;
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
        h264_frame_ready = false;
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
  }
}

/* ==========================================================================
 * H264 RAM store
 *
 * The SD card is kept powered off for the whole capture (VIDEO_CAPTURE runs
 * before SD_CARD_INIT): record_h264_to_ram() cannot call REC_Start/
 * REC_PushFrame directly (they need FatFS/the SD mounted). Instead it
 * accumulates encoded access units into this dedicated PSRAM store; once
 * SD_CARD_INIT has run, record_h264_flush_to_sd() replays them through the
 * existing REC_Start/REC_PushFrame/REC_Stop muxer.
 *
 * Deliberately a SEPARATE buffer from buffer_full_frame's tail (the ring
 * REC_Start uses) rather than reusing it: REC_PushFrame's own ring-offset
 * bookkeeping (during flush) is independent of this store's, so pointing it
 * at the same memory could make the internal memcpy's src/dst ranges
 * overlap without matching exactly -- undefined behavior. Two buffers keeps
 * this trivially safe.
 * ========================================================================== */
#define H264_RAM_STORE_SIZE (12u * 1024u * 1024u) /* 12MB */
#define H264_RAM_MAX_FRAMES 4500u                  /* generous buffer for 120 secondes @ 25fps */

typedef struct {
  uint32_t offset;
  uint32_t len;
  uint32_t duration; /* 1/90000 s, 0 = nominal 1/fps (see REC_PushFrame) */
} h264_frame_desc_t;

static uint8_t h264_ram_store[H264_RAM_STORE_SIZE] ALIGN_32 IN_PSRAM;
/* At 4500 entries (12 bytes each = ~53 KB) this no longer fits the default
 * .bss region (AXISRAM2_P2_S, only 475 KB total, shared with everything
 * else in the app). Place it in AXISRAM3456 instead (1792 KB, ~512 KB still
 * free there) -- same pattern already used for axisram_pool (app.c) and
 * ewl_heap_pool (app_enc.c). */
static h264_frame_desc_t h264_ram_frames[H264_RAM_MAX_FRAMES] __attribute__((section(".axisram_bss")));
static uint32_t h264_ram_frame_count;
static uint32_t h264_ram_used;
static int h264_ram_width, h264_ram_height;

/* Captures+encodes rec_duration seconds of H264 video into the RAM store
 * above (buffer_full_frame + setup_record_h264()'s double-buffered
 * capture; no SD access). Called in VIDEO_CAPTURE, right after
 * setup_record_h264(). Stops early (logged) if the store fills up before
 * rec_duration elapses.
 *   height : must match the value passed to setup_record_h264().
 * Returns the number of frames captured (> 0), or -1 if none were. */
int record_h264_to_ram(int height, int rec_duration)
{
  int width = height * 4 / 3; /* 4:3 */
  uint32_t start_tick, last_frame_tick, frame_count = 0, encode_ok_count = 0;

  h264_ram_frame_count = 0;
  h264_ram_used        = 0;
  h264_ram_width        = width;
  h264_ram_height       = height;

  start_tick = HAL_GetTick();
  last_frame_tick = start_tick;

  printf("[REC] video started %d ms after movement detection\r\n", (int)(start_tick - actual_ticks));
  printf("[REC] capturing %d sec to RAM... (@ %d fps @ %dp @ QF = %d)\r\n", rec_duration, H264_FPS, height, VIDEO_COMPRESSION_FACTOR);

  while (HAL_GetTick() - start_tick < (uint32_t)(rec_duration * 1000)) {
    if (!h264_frame_ready) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    watchdog_kick();
    h264_frame_ready = false;
    frame_count++;

    /* First frame is always IDR+SPS/PPS. */
    {
      bool force_idr = (frame_count == 1) || force_intra;
      /* snapshot: the completed buffer (the other one is being written) */
      uint8_t *p_frame = h264_ready_buf;
      size_t len = h264_encode_frame(p_frame, force_idr);
      force_intra = false;
      if (len > 0) {
        /* Real measured frame duration (variable frame rate): keeps the
         * MP4 duration equal to wall-clock time even if the sensor is not
         * exactly at 30 fps or if frames are skipped/dropped. */
        uint32_t now = HAL_GetTick();
        uint32_t dur90k = (now - last_frame_tick) * 90u;
        last_frame_tick = now;
        if (dur90k == 0u || dur90k > 90000u)
          dur90k = 0u; /* aberrant delta -> fall back to nominal 1/fps */

        if (h264_ram_frame_count >= H264_RAM_MAX_FRAMES || h264_ram_used + len > H264_RAM_STORE_SIZE) {
          printf("[REC] RAM store full (%lu frames, %lu KB), stopping capture early\r\n",
                 (unsigned long)h264_ram_frame_count, (unsigned long)h264_ram_used / 1024);
          break;
        }

        memcpy(&h264_ram_store[h264_ram_used], h264_venc_out, len);
        h264_ram_frames[h264_ram_frame_count].offset   = h264_ram_used;
        h264_ram_frames[h264_ram_frame_count].len      = (uint32_t)len;
        h264_ram_frames[h264_ram_frame_count].duration = dur90k;
        h264_ram_frame_count++;
        h264_ram_used += (uint32_t)len;
        encode_ok_count++;
      }
    }
  }
  unsigned long encoding = 100*encode_ok_count/frame_count;

  /* Encoded-size accounting: actual bitrate and RAM-store fill ratio for
   * this clip. Lets us measure on real footage how much headroom the ISP
   * grayscale neutralization (colorConvStatic, see imx335_isp_param_conf.h)
   * buys in the 12 MB store -- rather than guessing from raw bit-depth
   * ratios (see discussion: chroma_format_idc is hardcoded in the H264
   * encoder, so the gain is whatever fraction of the bitrate chroma used
   * to cost, not a fixed factor). */
  if (h264_ram_frame_count > 0) {
    uint32_t elapsed_ms = last_frame_tick - start_tick;
    float used_mb = (float)h264_ram_used / (1024.0f * 1024.0f);
    float store_total_mb = (float)H264_RAM_STORE_SIZE / (1024.0f * 1024.0f);
    float avg_mbps = elapsed_ms ? (h264_ram_used * 8.0f) / ((float)elapsed_ms * 1000.0f) : 0.0f;
    float store_fill_pct = 100.0f * (float)h264_ram_used / (float)H264_RAM_STORE_SIZE;

    printf("[REC] encoded %lu%% (%lu/%lu frames) | %.1f MB/%.1f MB (RAM store use: %.1f%%) | avg REC bitrate: %.2f Mbit/s\r\n",
           encoding, (unsigned long)encode_ok_count, (unsigned long)frame_count,
           used_mb, store_total_mb, store_fill_pct, avg_mbps);
  }

  /* Stop the capture->encode pipeline started by setup_record_h264(). */
  h264_streaming = false;

  /* Actually stop PIPE1's continuous capture (mirrors camera_warmup()'s
   * post-continuous-phase stop). Without this, PIPE1 is still running when
   * the next camera_warmup() calls CAM_Deinit() to tear it down -- harmless
   * the first time (nothing was running yet to deinit), but the second
   * recording's setup_record_h264() -> CMW_CAMERA_SetPipeConfig(PIPE1, ...)
   * then fails its "ret == HAL_OK" assert in DCMIPP_PipeInitCapture. */
  HAL_DCMIPP_CSI_PIPE_Stop(&hcamera_dcmipp, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  vTaskDelay(pdMS_TO_TICKS(50));

  /* Disable hardware double-buffer mode (never cleared by the HAL) so the
   * next single-buffer session (config/detect warmup) starts clean. */
  CLEAR_BIT(hcamera_dcmipp.Instance->P1PPCR, DCMIPP_P1PPCR_DBM);

  /* End encoder session: H264EncStrmEnd transitions ENCODING→INIT so that
   * H264EncStrmStart (called on next session's first encode) emits fresh
   * SPS/PPS for the next file.
   * NOTE: H264EncRelease must NOT be called — it crashes on this target.
   *       H264EncStrmEnd is safe and is the correct way to close a stream. */
  ENC_EndSession(h264_venc_out, H264_VENC_OUT_SIZE);

  /* Re-enter current mode from scratch */
  warmup_done = false;

  return (h264_ram_frame_count > 0) ? (int)h264_ram_frame_count : -1;
}

/* Muxes the RAM store filled by record_h264_to_ram() into fname on
 * the SD card, through the existing REC_Start/REC_PushFrame/REC_Stop
 * FreeRTOS SD writer task. Called in MULTIMEDIA_STORAGE, once SD_CARD_INIT
 * has succeeded. Unlike the live-recording path, there is no "next real-time
 * frame" to force an IDR onto if the SD writer's ring is briefly full, so
 * REC_PushFrame is retried (bounded) instead of dropping the frame -- a drop
 * here would silently corrupt the saved file.
 * Returns 0 on success. */
int record_h264_flush_to_sd(const char *fname)
{
  uint32_t frame_bytes = (uint32_t)h264_ram_width * (uint32_t)h264_ram_height * 2u; /* RGB565 */
  int ret;

  if (h264_ram_frame_count == 0)
    return -1;

  /* Ring buffer lives in the unused part of buffer_full_frame (after the 2
   * capture frames); capture is long done at this point so it is free. */
  if (REC_Start(h264_ram_width, h264_ram_height, H264_FPS,
                buffer_full_frame + 2 * frame_bytes,
                MAX_CAPTURE_FRAME_SIZE - 2 * frame_bytes,
                fname) != 0) {
    printf("[REC] record start failed, flush aborted\r\n");
    return -1;
  }

  for (uint32_t i = 0; i < h264_ram_frame_count; i++) {
    h264_frame_desc_t *d = &h264_ram_frames[i];
    uint32_t retries = 0;

    watchdog_kick();
    while (REC_PushFrame(&h264_ram_store[d->offset], d->len, d->duration) != 0) {
      if (++retries > 5000u) { /* ~5 s: the SD writer task should never stall this long */
        printf("[REC] flush: ring stayed full, frame %lu dropped\r\n", (unsigned long)i);
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  ret = REC_Stop();
  if (ret != 0)
    printf("[REC] mp4 finalize failed\r\n");

  return ret;
}

/* ==========================================================================
 * Detection JSON export
 * ========================================================================== */

/* snprintf-style append: n tracks the logical write position, safe past the
 * end of buf (writes get truncated/dropped, n keeps growing so callers can
 * detect overflow at the end). */
static int json_append(char *buf, size_t bufsz, int n, const char *fmt, ...)
{
  size_t off = ((size_t)n < bufsz) ? (size_t)n : bufsz;
  va_list ap;
  int written;

  va_start(ap, fmt);
  written = vsnprintf(buf + off, bufsz - off, fmt, ap);
  va_end(ap);

  return n + written;
}

static int json_bbox(char *buf, size_t bufsz, int n, const DETECT_BBox_t *b)
{
  return json_append(buf, bufsz, n, "{\"x_min\":%u,\"y_min\":%u,\"x_max\":%u,\"y_max\":%u}",
                      b->x_min, b->y_min, b->x_max, b->y_max);
}

static int json_bloc(char *buf, size_t bufsz, int n, const DETECT_Bloc_t *b)
{
  n = json_append(buf, bufsz, n, "{\"bbox\":");
  n = json_bbox(buf, bufsz, n, &b->bbox);
  return json_append(buf, bufsz, n,
                      ",\"valeur_moyenne\":%.2f,\"mean_moyen\":%.2f,\"std_moyen\":%.2f,\"nb_voisin_moyen\":%.2f}",
                      (double)b->valeur_moyenne, (double)b->mean_moyen, (double)b->std_moyen, (double)b->nb_voisin_moyen);
}

static int json_pipe_result(char *buf, size_t bufsz, int n, const DETECT_PipeResult_t *p)
{
  const DETECT_DeviationVoisinage_t *dv = &p->deviation_voisinage;

  n = json_append(buf, bufsz, n,
                   "{\"mouvement\":{\"detecte\":%s,\"delta_max_frame_moins_1\":%u,\"delta_max_frame_moins_2\":%u},"
                   "\"deviation_voisinage\":{\"detecte\":%s,\"pct_pipe\":",
                   p->mouvement.detecte ? "true" : "false",
                   p->mouvement.delta_max_frame_moins_1, p->mouvement.delta_max_frame_moins_2,
                   dv->detecte ? "true" : "false");

  /* pct_pipe is NaN (0/0) when the pipe is sized 0x0 (crop/downsize
   * misconfigured too small) -- report null rather than the literal "nan",
   * which isn't valid JSON. */
  n = isnan(dv->pct_pipe) ? json_append(buf, bufsz, n, "null")
                          : json_append(buf, bufsz, n, "%.2f", (double)dv->pct_pipe);

  n = json_append(buf, bufsz, n, ",\"bbox_global\":");
  n = dv->bbox_global_valid ? json_bbox(buf, bufsz, n, &dv->bbox_global) : json_append(buf, bufsz, n, "null");

  n = json_append(buf, bufsz, n, ",\"blocs\":[");
  for (uint8_t i = 0; i < dv->nb_blocs; i++)
  {
    if (i > 0) n = json_append(buf, bufsz, n, ",");
    n = json_bloc(buf, bufsz, n, &dv->blocs[i]);
  }
  return json_append(buf, bufsz, n, "]}}");
}

int record_detection_json_to_sd(const char *fname, const char *det_timestamp,
                                 const DETECT_Result_t *p_result, const Config_t *p_config,
                                 int rec_height, int32_t exposure_us, int32_t gain_mdb)
{
  static char buf[8192];
  int rec_width = rec_height * 4 / 3; /* 4:3, see record_snapshot_to_ram/setup_record_h264 */
  double gain_db = (double)gain_mdb / 1000.0;
  int iso_approx = (int)(100.0 * pow(10.0, gain_db / 20.0)); /* same formula as camera_config_gui.py */
  int n = 0;

  n = json_append(buf, sizeof(buf), n,
    "{\"titre\":\"DIAS - D\xc3\xa9tecteur Intelligent d'Animaux Sauvages\","
    "\"no_device\":\"" DEVICE_NBR "\",\"rev_fw\":\"" FW_REV "\","
    "\"config\":{"
    "\"second_plan\":{\"crop\":{\"v_start\":%u,\"v_size\":%u,\"h_start\":%u,\"h_size\":%u},\"downsize_ratio\":%.2f},"
    "\"premier_plan\":{\"crop\":{\"v_start\":%u,\"v_size\":%u,\"h_start\":%u,\"h_size\":%u},\"downsize_ratio\":%.2f,\"decimation_ratio\":%u},"
    "\"thresholds\":{"
      "\"mouvement\":{\"thresh_mvt\":%u,\"note\":\"diff frame-a-frame vs les 2 frames precedentes, independant du masque deviation/voisinage\"},"
      "\"deviation\":{\"note\":\"pixel hors-bande si valeur < mean-std ou > mean+std ; mean/std sont calcules par pixel, cf. detections\"},"
      "\"voisinage\":{\"dim_carre\":\"3x3\",\"nb_voisin\":{\"second_plan\":%u,\"premier_plan\":%u},\"note\":\"nb minimal de cellules du carre (centre inclus) hors-bande pour confirmer la detection\"},"
      "\"derive_fond\":{\"stat_adjust_ratio\":\"1/12\",\"note\":\"mean/std glissent lentement vers la frame courante sur les pixels non detectes\"}"
    "},"
    "\"proprietes_enregistrement\":{\"width\":%d,\"height\":%d,\"format\":\"4:3\","
      "\"video\":{\"fps\":%d,\"duree_s\":%d,\"facteur_compression\":%d}},"
    "\"resolution_gui\":{\"width\":%d,\"height\":%d,\"note\":\"pleine resolution capteur\"}"
    "},",
    p_config->crop_v_start_pipe1, p_config->crop_v_size_pipe1, p_config->crop_h_start_pipe1, p_config->crop_h_size_pipe1,
    (double)p_config->downsize_ratio_pipe1,
    p_config->crop_v_start_pipe2, p_config->crop_v_size_pipe2, p_config->crop_h_start_pipe2, p_config->crop_h_size_pipe2,
    (double)p_config->downsize_ratio_pipe2, p_config->decimation_ratio_pipe2,
    DETECT_THRESH_MVT, DETECT_NB_VOISIN_PIPE1, DETECT_NB_VOISIN_PIPE2,
    rec_width, rec_height, H264_FPS, VIDEO_DURATION_S, VIDEO_COMPRESSION_FACTOR,
    SENSOR_WIDTH, SENSOR_HEIGHT);

  n = json_append(buf, sizeof(buf), n,
    "\"detections\":{\"note_coordonnees\":\"bbox et blocs sont exprimes en coordonnees locales au pipe (apres crop + downsize), pas en coordonnees capteur brutes\","
    "\"events\":[{\"det_timestamp\":\"%s\","
    "\"camera\":{\"exposure_us\":%ld,\"gain_db\":%.2f,\"iso_approx\":%d},"
    "\"second_plan\":",
    det_timestamp, (long)exposure_us, gain_db, iso_approx);
  n = json_pipe_result(buf, sizeof(buf), n, &p_result->second_plan);
  n = json_append(buf, sizeof(buf), n, ",\"premier_plan\":");
  n = json_pipe_result(buf, sizeof(buf), n, &p_result->premier_plan);
  n = json_append(buf, sizeof(buf), n, "}]}}");

  if (n < 0 || (size_t)n >= sizeof(buf)) {
    printf("[REC] detection json too large (%d bytes, buffer %u)\r\n", n, (unsigned)sizeof(buf));
    return -1;
  }

  return REC_SaveFile((const uint8_t *)buf, (size_t)n, fname);
}

