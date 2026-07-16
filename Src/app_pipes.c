/**
 ******************************************************************************
 * @file    app_pipes.c
 * @brief   DCMIPP pipes configuration (detect mode).
 ******************************************************************************
 */
#include "app_pipes.h"
#include "app_shared.h"

#include <math.h>
#include "cmw_camera.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_dcmipp.h"

/* Post-processing configuration + detect-mode capture buffers (module-private) */
static DCMIPP_CropConfTypeDef       crop_conf = { 0 };
static DCMIPP_DownsizeTypeDef       downsize_conf_pipe1 = { 0 };
static DCMIPP_DownsizeTypeDef       downsize_conf_pipe2 = { 0 };
static DCMIPP_DecimationConfTypeDef decimation_conf = { 0 };
static uint8_t *buffer_pipe1_capture = NULL;
static uint8_t *buffer_pipe2_capture = NULL;
static uint32_t size_pipe1 = 0;
static uint32_t size_pipe2 = 0;

/* ==========================================================================
 * DCMIPP pipes configuration (detect mode)
 * ========================================================================== */

/* Applies the crop/decimation/downsize configuration received from the GUI
 * (config_py) to DCMIPP pipe1 and pipe2, then allocates the matching
 * capture buffers from the AXISRAM pool.
 * NOTE: axisram_alloc never frees — repeated detect-mode entries consume
 * the pool. */
void dcmipp_apply_detect_config(void)
{
  /* The bump allocator never frees: reset it here, since the two capture
   * buffers below are its only users.  Without this, re-entering this state
   * after each recording would exhaust the 1 MB pool and trap. */
  axisram_reset();

  /* --- Pipe 1: crop + downsize --- */
  crop_conf.VStart   = config_py.crop_v_start_pipe1;
  crop_conf.VSize    = config_py.crop_v_size_pipe1;
  crop_conf.HStart   = config_py.crop_h_start_pipe1;
  crop_conf.HSize    = config_py.crop_h_size_pipe1;
  crop_conf.PipeArea = DCMIPP_POSITIVE_AREA;

  downsize_conf_pipe1.HRatio     = floor(8192 * config_py.downsize_ratio_pipe1);
  downsize_conf_pipe1.HDivFactor = floor((1024 * 8192 - 1) / downsize_conf_pipe1.HRatio);
  downsize_conf_pipe1.HSize      = floor(crop_conf.HSize / config_py.downsize_ratio_pipe1);
  downsize_conf_pipe1.VRatio     = downsize_conf_pipe1.HRatio;
  downsize_conf_pipe1.VDivFactor = downsize_conf_pipe1.HDivFactor;
  downsize_conf_pipe1.VSize      = floor(crop_conf.VSize / config_py.downsize_ratio_pipe1);

  HAL_DCMIPP_PIPE_SetCropConfig(&hcamera_dcmipp, DCMIPP_PIPE1, &crop_conf);
  HAL_DCMIPP_PIPE_EnableCrop(&hcamera_dcmipp, DCMIPP_PIPE1);

  HAL_DCMIPP_PIPE_SetDownsizeConfig(&hcamera_dcmipp, DCMIPP_PIPE1, &downsize_conf_pipe1);
  HAL_DCMIPP_PIPE_EnableDownsize(&hcamera_dcmipp, DCMIPP_PIPE1);

  /* --- Pipe 2: crop + decimation + downsize --- */
  crop_conf.VStart = config_py.crop_v_start_pipe2;
  crop_conf.VSize  = config_py.crop_v_size_pipe2;
  crop_conf.HStart = config_py.crop_h_start_pipe2;
  crop_conf.HSize  = config_py.crop_h_size_pipe2;

  switch (config_py.decimation_ratio_pipe2) {
  case 1:
    decimation_conf.VRatio = DCMIPP_VDEC_ALL;
    decimation_conf.HRatio = DCMIPP_HDEC_ALL;
    break;
  case 2:
    decimation_conf.VRatio = DCMIPP_VDEC_1_OUT_2;
    decimation_conf.HRatio = DCMIPP_HDEC_1_OUT_2;
    break;
  case 4:
    decimation_conf.VRatio = DCMIPP_VDEC_1_OUT_4;
    decimation_conf.HRatio = DCMIPP_HDEC_1_OUT_4;
    break;
  case 8:
  default:
    decimation_conf.VRatio = DCMIPP_VDEC_1_OUT_8;
    decimation_conf.HRatio = DCMIPP_HDEC_1_OUT_8;
    break;
  }

  downsize_conf_pipe2.HRatio     = floor(8192 * config_py.downsize_ratio_pipe2);
  downsize_conf_pipe2.HDivFactor = floor((1024 * 8192 - 1) / downsize_conf_pipe2.HRatio);
  downsize_conf_pipe2.HSize      = floor(crop_conf.HSize / (config_py.downsize_ratio_pipe2 * config_py.decimation_ratio_pipe2));
  downsize_conf_pipe2.VRatio     = downsize_conf_pipe2.HRatio;
  downsize_conf_pipe2.VDivFactor = downsize_conf_pipe2.HDivFactor;
  downsize_conf_pipe2.VSize      = floor(crop_conf.VSize / (config_py.downsize_ratio_pipe2 * config_py.decimation_ratio_pipe2));

  HAL_DCMIPP_PIPE_SetCropConfig(&hcamera_dcmipp, DCMIPP_PIPE2, &crop_conf);
  HAL_DCMIPP_PIPE_EnableCrop(&hcamera_dcmipp, DCMIPP_PIPE2);

  HAL_DCMIPP_PIPE_SetDecimationConfig(&hcamera_dcmipp, DCMIPP_PIPE2, &decimation_conf);
  HAL_DCMIPP_PIPE_EnableDecimation(&hcamera_dcmipp, DCMIPP_PIPE2);

  HAL_DCMIPP_PIPE_SetDownsizeConfig(&hcamera_dcmipp, DCMIPP_PIPE2, &downsize_conf_pipe2);
  HAL_DCMIPP_PIPE_EnableDownsize(&hcamera_dcmipp, DCMIPP_PIPE2);

  /* --- Capture buffers --- */
  size_pipe1 = SENSOR_WIDTH * downsize_conf_pipe1.VSize;
  size_pipe2 = SENSOR_WIDTH * downsize_conf_pipe2.VSize;
  buffer_pipe1_capture = (uint8_t *)axisram_alloc(size_pipe1);
  buffer_pipe2_capture = (uint8_t *)axisram_alloc(size_pipe2);
}

