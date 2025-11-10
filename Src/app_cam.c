 /**
 ******************************************************************************
 * @file    app_cam.c
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <assert.h>
#include "cmw_camera.h"
#include "app_cam.h"
#include "app_config.h"
#include "stm32n6xx.h"
#include "utils.h"

/* Define sensor orientation */
#if CAMERA_SELFY == 1
#define SENSOR_IMX335_FLIP CMW_MIRRORFLIP_MIRROR
#define SENSOR_VD66GY_FLIP CMW_MIRRORFLIP_FLIP
#define SENSOR_VD55G1_FLIP CMW_MIRRORFLIP_FLIP
#define SENSOR_VD1943_FLIP CMW_MIRRORFLIP_MIRROR
#else
#define SENSOR_IMX335_FLIP CMW_MIRRORFLIP_NONE
#define SENSOR_VD66GY_FLIP CMW_MIRRORFLIP_FLIP_MIRROR
#define SENSOR_VD55G1_FLIP CMW_MIRRORFLIP_FLIP_MIRROR
#define SENSOR_VD1943_FLIP CMW_MIRRORFLIP_NONE
#endif

/* Define sensor width x height size. 0x0 means full frame */
#define SENSOR_WIDTH     0
#define SENSOR_HEIGHT    0

static const char *sensor_names[] = {
  "CMW_UNKNOWN",
  "CMW_VD66GY",
  "CMW_IMX335",
  "CMW_VD55G1",
  "CMW_VD1943",
};

static CMW_Sensor_Name_t sensor;
static int is_sensor_valid = 0;

static int CAM_getFlipMode(CMW_Sensor_Name_t sensor)
{
  int sensor_mirror_flip = CMW_MIRRORFLIP_NONE;
  int sensor_name_idx = 0;

  switch (sensor) {
  case CMW_VD66GY_Sensor:
    sensor_mirror_flip = SENSOR_VD66GY_FLIP;
    sensor_name_idx = 1;
    break;
  case CMW_IMX335_Sensor:
    sensor_mirror_flip = SENSOR_IMX335_FLIP;
    sensor_name_idx = 2;
    break;
  case CMW_VD55G1_Sensor:
    sensor_mirror_flip = SENSOR_VD55G1_FLIP;
    sensor_name_idx = 3;
    break;
  case CMW_VD1943_Sensor:
    sensor_mirror_flip = SENSOR_VD1943_FLIP;
    sensor_name_idx = 4;
    break;
  default:
    assert(0);
  }
  printf("Detected %s\n", sensor_names[sensor_name_idx]);

  return sensor_mirror_flip;
}

static int CAM_FormatToBpp(int dcmipp_output_format)
{
  int bpp = 0;

  switch (dcmipp_output_format)
  {
  case DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1:
    bpp = 1;
    break;
  case DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1:
  case DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1:
    bpp = 2;
    break;
  case DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1:
    bpp = 3;
    break;
  default:
    assert(0);
  }

  return bpp;
}

/* Keep display output aspect ratio using crop area */
static void CAM_InitCropConfig(CMW_Manual_roi_area_t *roi, int sensor_width, int sensor_height, CAM_conf_t *conf)
{
  const float ratiox = (float)sensor_width / conf->capture_width;
  const float ratioy = (float)sensor_height / conf->capture_height;
  const float ratio = MIN(ratiox, ratioy);

  assert(ratio >= 1);
  assert(ratio < 64);

  roi->width = (uint32_t) MIN(conf->capture_width * ratio, sensor_width);
  roi->height = (uint32_t) MIN(conf->capture_height * ratio, sensor_height);
  roi->offset_x = (sensor_width - roi->width + 1) / 2;
  roi->offset_y = (sensor_height - roi->height + 1) / 2;
}

static void CAM_EnableYuv(uint32_t Pipe)
{
  DCMIPP_ColorConversionConfTypeDef color_conf = {
    .ClampOutputSamples = ENABLE,
    .OutputSamplesType = DCMIPP_CLAMP_YUV,
    .RR = 131, .RG = -119, .RB = -12, .RA = 128,
    .GR =  55, .GG =  183, .GB =  18, .GA =   0,
    .BR = -30, .BG = -101, .BB = 131, .BA = 128,
  };
  int ret;

  /* only pipe1 can do yuv */
  assert(Pipe == DCMIPP_PIPE1);
  ret = HAL_DCMIPP_PIPE_SetYUVConversionConfig(CMW_CAMERA_GetDCMIPPHandle(), Pipe, &color_conf);
  assert(ret == HAL_OK);
  ret = HAL_DCMIPP_PIPE_EnableYUVConversion(CMW_CAMERA_GetDCMIPPHandle(), Pipe);
  assert(ret == HAL_OK);
}

static void DCMIPP_PipeInitCapture(CAM_conf_t *cam_conf, int sensor_width, int sensor_height, CAM_conf_t *conf)
{
  CMW_DCMIPP_Conf_t dcmipp_conf;
  uint32_t hw_pitch;
  int ret;

  assert(conf->capture_width >= conf->capture_height);

  dcmipp_conf.output_width = conf->capture_width;
  dcmipp_conf.output_height = conf->capture_height;
  dcmipp_conf.output_format = cam_conf->dcmipp_output_format;
  dcmipp_conf.output_bpp = CAM_FormatToBpp(cam_conf->dcmipp_output_format);
  dcmipp_conf.mode = CMW_Aspect_ratio_manual_roi;
  dcmipp_conf.enable_swap = cam_conf->is_rgb_swap;
  dcmipp_conf.enable_gamma_conversion = 0;
  CAM_InitCropConfig(&dcmipp_conf.manual_conf, sensor_width, sensor_height, conf);
  ret = CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &dcmipp_conf, &hw_pitch);
  assert(ret == HAL_OK);
  assert(hw_pitch == dcmipp_conf.output_width * dcmipp_conf.output_bpp);

  if (cam_conf->dcmipp_output_format == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1)
    CAM_EnableYuv(DCMIPP_PIPE1);
}

void CAM_Init(CAM_conf_t *conf)
{
  CMW_CameraInit_t cam_conf;
  int ret;

  if (!is_sensor_valid) {
    is_sensor_valid = 1;
    ret = CMW_CAMERA_GetSensorName(&sensor);
    assert(ret == CMW_ERROR_NONE);
  }

  cam_conf.width = SENSOR_WIDTH;
  cam_conf.height = SENSOR_HEIGHT;
  cam_conf.fps = conf->fps;
  cam_conf.mirror_flip = CAM_getFlipMode(sensor);

  ret = CMW_CAMERA_Init(&cam_conf, NULL);
  assert(ret == CMW_ERROR_NONE);

  /* CMW_CAMERA_Init update width height */
  assert(cam_conf.width);
  assert(cam_conf.height);
  DCMIPP_PipeInitCapture(conf, cam_conf.width, cam_conf.height, conf);
}

void CAM_CapturePipe_Start(uint8_t *capture_pipe_dst, uint32_t cam_mode)
{
  int ret;

  ret = CMW_CAMERA_Start(DCMIPP_PIPE1, capture_pipe_dst, cam_mode);
  assert(ret == CMW_ERROR_NONE);
}

void CAM_IspUpdate(void)
{
  int ret;

  ret = CMW_CAMERA_Run();
  assert(ret == CMW_ERROR_NONE);
}

void CAM_Deinit()
{
  int ret;

  ret = CMW_CAMERA_DeInit();
  assert(ret == CMW_ERROR_NONE);
}

void CMW_CAMERA_PIPE_ErrorCallback(uint32_t pipe)
{
  /* FIXME : Need to tune sensor/ipplug so we can remove this implementation */
}
