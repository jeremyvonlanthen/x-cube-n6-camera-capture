/**
 ******************************************************************************
 * @file    app_capture.c
 * @brief   Camera helpers (full-res warmup + YUV snapshot for config mode).
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
 * converge for WARMUP_FRAMES_TARGET frames before stopping the pipe(s).
 *   cap_w/cap_h : pipe output size (SENSOR_WIDTH x SENSOR_HEIGHT for both the
 *                 config preview and detect warmup). */
void camera_warmup(uint32_t cap_w, uint32_t cap_h, uint32_t output_format)
{
	static int camera_initialized = 0;

  CAM_conf_t cam_conf = { 0 };
  uint8_t two_pipes = (output_format == DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1);

  if(camera_initialized) CAM_Deinit();

  cam_conf.capture_width        = cap_w;
  cam_conf.capture_height       = cap_h;
  cam_conf.fps                  = SENSOR_WARMUP_FPS;
  cam_conf.dcmipp_output_format = output_format;
  cam_conf.is_rgb_swap          = 0;
  CAM_Init(&cam_conf, two_pipes);

  /* Required on re-warmup: the frame event callback only increments
   * warmup_frames while warmup_done == 0.  Without this reset, the second
   * warmup (DETECT_MODE_WARMUP) waits forever since warmup_done is still 1
   * from the previous mode. */
  warmup_done = 0;
  warmup_frames = 0;

  CAM_CapturePipe_Start(buffer_full_frame, buffer_warmup, CMW_MODE_CONTINUOUS, 0);

  while (warmup_frames < WARMUP_FRAMES_TARGET)
    vTaskDelay(pdMS_TO_TICKS(10));

  HAL_DCMIPP_CSI_PIPE_Stop(&hcamera_dcmipp, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  vTaskDelay(pdMS_TO_TICKS(50));
  if (two_pipes) {
    HAL_DCMIPP_CSI_PIPE_Stop(&hcamera_dcmipp, DCMIPP_PIPE2, DCMIPP_VIRTUAL_CHANNEL0);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  warmup_done = 1;
  camera_initialized = 1;
}

/* One full-sensor YUV422 snapshot (config mode), JPEG-encoded and sent to the
 * GUI over UART (kept at full resolution for accurate crop-region framing) */
int capture_yuv(void)
{
  snapshot_in_progress = 1;
  frame_ready = 0;
  CAM_CapturePipe_Start(buffer_full_frame, buffer_warmup, CMW_MODE_SNAPSHOT, 0);

  uint32_t start = HAL_GetTick();
  while (!frame_ready) {
    if (HAL_GetTick() - start > 30000) //FIXME: peut-être remplacer par un watchdog
      break;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  snapshot_in_progress = 0;

  SCB_InvalidateDCache_by_Addr((uint32_t *)buffer_full_frame, CACHE_ALIGN_SIZE(MAX_CAPTURE_FRAME_SIZE));

  jpg_conf.width      = SENSOR_WIDTH;
  jpg_conf.height     = SENSOR_HEIGHT;
  jpg_conf.fmt_src    = JPG_SRC_YUV422;
  jpg_conf.full_width = SENSOR_WIDTH;
  JPG_Init(&jpg_conf);
  int jpeg_len = JPG_Encode(hires_jpeg_buffer, buffer_full_frame,
                        MAX_JPEG_FRAME_SIZE, MAX_CAPTURE_FRAME_SIZE);
  SCB_CleanDCache_by_Addr((uint32_t *)hires_jpeg_buffer, CACHE_ALIGN_SIZE(jpeg_len));
  JPG_Deinit();

  return jpeg_len;
}

