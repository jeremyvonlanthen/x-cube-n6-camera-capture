/**
 ******************************************************************************
 * @file    app_capture.c
 * @brief   Camera helpers (full-res warmup + MONO snapshot for config mode).
 ******************************************************************************
 */
#include "app_capture.h"
#include "app_shared.h"
#include "app_config.h"

#include "app_cam.h"
#include "app_jpg.h"
#include "cmw_camera.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_dcmipp.h"
#include "FreeRTOS.h"
#include "task.h"

/* ==========================================================================
 * Camera helpers
 * ========================================================================== */

/* (Re)initializes the camera at the requested capture resolution (full-scene
 * downscale from the sensor) and DCMIPP output format, then lets the AE/ISP
 * converge for warmup_frames_target frames before stopping the pipe(s).
 *   cap_w/cap_h : pipe output size (SENSOR_WIDTH x SENSOR_HEIGHT for both the
 *                 config preview and detect warmup). */
void camera_warmup(uint8_t warmup_frames_target, uint8_t warmup_fps, bool two_pipes)
{
	static bool camera_initialized = false;
	int32_t seed_exp = 0, seed_gain = 0;

  CAM_conf_t cam_conf = { 0 };

  if(camera_initialized) {
    CMW_CAMERA_GetExposure(&seed_exp);
    CMW_CAMERA_GetGain(&seed_gain);
    CAM_Deinit();
  }

  cam_conf.capture_width        = SENSOR_WIDTH;
  cam_conf.capture_height       = SENSOR_HEIGHT;
  cam_conf.fps                  = warmup_fps;
  cam_conf.dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1;
  cam_conf.is_rgb_swap          = 0;
  CAM_Init(&cam_conf, (uint8_t)two_pipes);

  /* Seed the freshly-reset AE (CAM_Init resets exposure/gain to defaults) --
   * gives AE a head start, but the full convergence wait below still runs
   * unconditionally: AE keeps drifting for a while after a re-seed (it runs
   * continuously, in the background, not just during this wait), and
   * skipping straight to DETECT_CalibrateStats() risked freezing the
   * background model before AE had actually settled on the detect crop --
   * observed in the field as unreliable detection (real movement missed or
   * masked) from the second detect cycle onward. */
  if (seed_exp  > 0) CMW_CAMERA_SetExposure(seed_exp);
  if (seed_gain > 0) CMW_CAMERA_SetGain(seed_gain);

  warmup_done = false;
  warmup_frames = 0;

  CAM_CapturePipe_Start(buffer_full_frame, buffer_warmup, CMW_MODE_CONTINUOUS, 0);

  while (warmup_frames < warmup_frames_target)
    vTaskDelay(pdMS_TO_TICKS(10));

  HAL_DCMIPP_CSI_PIPE_Stop(&hcamera_dcmipp, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  vTaskDelay(pdMS_TO_TICKS(50));

  snapshot_in_progress = true;
  frame_ready = false;
  CAM_CapturePipe_Start(buffer_full_frame, buffer_warmup, CMW_MODE_SNAPSHOT, 0);
  {
    uint32_t discard_start = HAL_GetTick();
    while (!frame_ready && HAL_GetTick() - discard_start < 1000)
      vTaskDelay(pdMS_TO_TICKS(1));
  }
  snapshot_in_progress = false;

  if(two_pipes){
    HAL_DCMIPP_CSI_PIPE_Stop(&hcamera_dcmipp, DCMIPP_PIPE2, DCMIPP_VIRTUAL_CHANNEL0);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  warmup_done = true;
  camera_initialized = true;
}

/* One full-sensor MONO snapshot (config mode), JPEG-encoded and sent to the
 * GUI over UART (kept at full resolution for accurate crop-region framing) */
int capture_img(void)
{
  snapshot_in_progress = true;
  frame_ready = false;
  CAM_CapturePipe_Start(buffer_full_frame, buffer_warmup, CMW_MODE_SNAPSHOT, 0);

  uint32_t start = HAL_GetTick();
  while (!frame_ready) {
    if (HAL_GetTick() - start > 30000) //FIXME: peut-être remplacer par un watchdog
      break;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  snapshot_in_progress = false;

  SCB_InvalidateDCache_by_Addr((uint32_t *)buffer_full_frame, CACHE_ALIGN_SIZE(MAX_CAPTURE_FRAME_SIZE));

  jpg_conf.width      = SENSOR_WIDTH;
  jpg_conf.height     = SENSOR_HEIGHT;
  jpg_conf.fmt_src    = JPG_SRC_GREY;
  jpg_conf.full_width = SENSOR_WIDTH;
  JPG_Init(&jpg_conf);
  int jpeg_len = JPG_Encode(hires_jpeg_buffer, buffer_full_frame,
                        MAX_JPEG_FRAME_SIZE, MAX_CAPTURE_FRAME_SIZE);
  SCB_CleanDCache_by_Addr((uint32_t *)hires_jpeg_buffer, CACHE_ALIGN_SIZE(jpeg_len));
  JPG_Deinit();

  return jpeg_len;
}

