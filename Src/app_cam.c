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
#include <stdbool.h>
#include <stdio.h>
#include "cmw_camera.h"
#include "app_cam.h"
#include "app_config.h"
#include "stm32n6xx.h"
#include "stm32n6xx_hal.h"
#include "utils.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* CAM_Init() retry budget: see the comment above its DCMIPP_PipeInitCapture()
 * retry loop. */
#define CAM_INIT_MAX_ATTEMPTS 5

/* ISP update task: see CAM_IspUpdate_SignalFromISR()/isp_update_task_fct()
 * below for why CAM_IspUpdate() must not run directly from the VSYNC IRQ. */
#define ISP_UPDATE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define ISP_UPDATE_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)

/* Define sensor orientation. DIAS is a stationary wildlife camera, not a
 * selfie application: IMX335 (the sensor actually mounted, see README) is
 * fixed to CMW_MIRRORFLIP_NONE regardless of CAMERA_SELFY, which was
 * mirroring every capture left/right. */
#define SENSOR_IMX335_FLIP CMW_MIRRORFLIP_NONE
#if CAMERA_SELFY == 1
#define SENSOR_VD66GY_FLIP CMW_MIRRORFLIP_FLIP
#define SENSOR_VD55G1_FLIP CMW_MIRRORFLIP_FLIP
#define SENSOR_VD1943_FLIP CMW_MIRRORFLIP_MIRROR
#else
#define SENSOR_VD66GY_FLIP CMW_MIRRORFLIP_FLIP_MIRROR
#define SENSOR_VD55G1_FLIP CMW_MIRRORFLIP_FLIP_MIRROR
#define SENSOR_VD1943_FLIP CMW_MIRRORFLIP_NONE
#endif

/* Define sensor width x height size. 0x0 means full frame */
#define SENSOR_WIDTH     0
#define SENSOR_HEIGHT    0

static CMW_Sensor_Name_t sensor;
static bool is_sensor_valid = false;

static int CAM_getFlipMode(CMW_Sensor_Name_t sensor)
{
  int sensor_mirror_flip = CMW_MIRRORFLIP_NONE;

  switch (sensor) {
  case CMW_VD66GY_Sensor:
    sensor_mirror_flip = SENSOR_VD66GY_FLIP;
    break;
  case CMW_IMX335_Sensor:
    sensor_mirror_flip = SENSOR_IMX335_FLIP;
    break;
  case CMW_VD55G1_Sensor:
    sensor_mirror_flip = SENSOR_VD55G1_FLIP;
    break;
  case CMW_VD1943_Sensor:
    sensor_mirror_flip = SENSOR_VD1943_FLIP;
    break;
  default:
    assert(0);
  }

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
  case DCMIPP_PIXEL_PACKER_FORMAT_ARGB8888:
    bpp = 4;
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

/* Returns 0 on success, -1 if either pipe's HAL config call failed (a rare
 * DCMIPP/CSI D-PHY relock race can leave the peripheral in a state where
 * HAL_DCMIPP_PIPE_SetConfig legitimately refuses the request -- see
 * CAM_Init()'s retry loop, the caller that actually needs to recover from
 * this; CAM_Pipe1_SetFormat()'s live-reconfigure callers just log it). The
 * hw_pitch mismatch asserts are left as hard asserts: unlike a HAL call
 * failing, a pitch mismatch here means the app miscomputed its own request,
 * which is a real bug to fix, not a transient condition to retry. */
static int DCMIPP_PipeInitCapture(CAM_conf_t *cam_conf, int sensor_width, int sensor_height, CAM_conf_t *conf, uint8_t two_pipes)
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
  dcmipp_conf.enable_gamma_conversion = 1;  /* sRGB-ish gamma: lifts midtones so JPEG & video look bright/pleasant (0 = linear, darker) */
  CAM_InitCropConfig(&dcmipp_conf.manual_conf, sensor_width, sensor_height, conf);

  /*Init Pipe1*/
  ret = CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &dcmipp_conf, &hw_pitch);
  if (ret != HAL_OK)
    return -1;
  assert(hw_pitch == dcmipp_conf.output_width * dcmipp_conf.output_bpp);

  if(two_pipes)
  {
	  dcmipp_conf.output_format = DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1;
	  dcmipp_conf.output_bpp = 1;

	  /*Init Pipe2*/
	  ret = CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE2, &dcmipp_conf, &hw_pitch);
	  if (ret != HAL_OK)
	    return -1;
	  assert(hw_pitch == dcmipp_conf.output_width * dcmipp_conf.output_bpp);
  }

  if (cam_conf->dcmipp_output_format == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1)
    CAM_EnableYuv(DCMIPP_PIPE1);

  return 0;
}

/* Reconfigures PIPE1 ONLY (pixel packer format + full-scene downscale),
 * without touching the sensor: exposure/gain/ISP state are preserved, so no
 * AE warmup is needed afterwards.  Used to grab a COLOR snapshot while the
 * camera runs in detect (mono, cropped/downsized) mode.
 *   sensor_width/height : the ROI source (full sensor) -> keeps full field of view
 *   out_width/height    : the pipe output size (<= sensor) -> DCMIPP downscales
 * Passing out == sensor gives a 1:1 full-sensor grab.
 * The pipe must be stopped (snapshot mode) when calling this. */
void CAM_Pipe1_SetFormat(int sensor_width, int sensor_height,
                         int out_width, int out_height, int output_format)
{
  CAM_conf_t conf;

  conf.capture_width        = out_width;
  conf.capture_height       = out_height;
  conf.fps                  = 0; /* unused by the pipe config */
  conf.dcmipp_output_format = output_format;
  conf.is_rgb_swap          = 0;

  /* ROI = full sensor, output = out_width x out_height: CAM_InitCropConfig
   * keeps the aspect ratio and DCMIPP downscales the whole scene (not a crop).
   * Re-enables YUV conversion when needed; overrides any detect crop/downsize. */
  if (DCMIPP_PipeInitCapture(&conf, sensor_width, sensor_height, &conf, 0) != 0)
    printf("[CAM] pipe1 format reconfigure failed\r\n");
}

/* ISP_BackgroundProcess() (called through CMW_CAMERA_Run() -> the sensor
 * driver's Run()) writes exposure/gain to the sensor over I2C -- a blocking
 * transaction. Running it straight from the DCMIPP VSYNC IRQ used to freeze
 * the whole system intermittently: main.c's boot sequence sets every
 * peripheral IRQ (DCMIPP included) to the exact same priority as SysTick, so
 * a same-priority IRQ cannot be preempted by it; any I2C wait inside the ISR
 * then stalls SysTick, freezing HAL_GetTick() and, with it, every
 * timeout-based wait in the system (observed as camera_warmup() hanging
 * silently forever, with no diagnostic ever printing). Deferring the actual
 * work to this task keeps the blocking I2C call at task level, where a
 * same-priority IRQ conflict can no longer stall the tick. */
static StaticSemaphore_t isp_update_sem_storage;
static SemaphoreHandle_t isp_update_sem;
static StaticTask_t isp_update_task_tcb;
static StackType_t isp_update_task_stack[ISP_UPDATE_TASK_STACK_SIZE];
static bool isp_update_task_started = false;

static void isp_update_task_fct(void *arg)
{
  (void)arg;
  for (;;) {
    xSemaphoreTake(isp_update_sem, portMAX_DELAY);
    CAM_IspUpdate();
  }
}

static void isp_update_task_ensure_started(void)
{
  if (isp_update_task_started)
    return;

  isp_update_sem = xSemaphoreCreateBinaryStatic(&isp_update_sem_storage);
  xTaskCreateStatic(isp_update_task_fct, "isp_upd", ISP_UPDATE_TASK_STACK_SIZE, NULL,
                    ISP_UPDATE_TASK_PRIORITY, isp_update_task_stack, &isp_update_task_tcb);
  isp_update_task_started = true;
}

void CAM_IspUpdate_SignalFromISR(void)
{
  BaseType_t higher_priority_task_woken = pdFALSE;

  if (isp_update_sem == NULL)
    return;

  xSemaphoreGiveFromISR(isp_update_sem, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void CAM_Init(CAM_conf_t *conf, uint8_t two_pipes)
{
  CMW_CameraInit_t cam_conf;
  int ret;
  int attempt;

  isp_update_task_ensure_started();

  if (!is_sensor_valid) {
    is_sensor_valid = true;
    ret = CMW_CAMERA_GetSensorName(&sensor);
    assert(ret == CMW_ERROR_NONE);
  }

  /* DCMIPP_PipeInitCapture() can fail on a rare DCMIPP/CSI D-PHY relock
   * race: a link-error IRQ flips hcamera_dcmipp.State to ERROR in the
   * narrow window between the sensor probe (inside CMW_CAMERA_Init) and
   * the SetPipeConfig call right after, so HAL_DCMIPP_PIPE_SetConfig
   * legitimately refuses the request. A fresh CMW_CAMERA_DeInit()+Init()
   * cycle forces the state back to READY (the sensor probe's own
   * CSI_PIPE_SetConfig call does this unconditionally), so retrying from
   * scratch self-heals almost always. Bounded so a genuine, persistent
   * hardware fault still surfaces (assert) instead of retrying forever on
   * an unattended device. */
  for (attempt = 1; ; attempt++) {
    cam_conf.width = SENSOR_WIDTH;
    cam_conf.height = SENSOR_HEIGHT;
    cam_conf.fps = conf->fps;
    cam_conf.mirror_flip = CAM_getFlipMode(sensor);

    ret = CMW_CAMERA_Init(&cam_conf, NULL);
    assert(ret == CMW_ERROR_NONE);

    /* CMW_CAMERA_Init update width height */
    assert(cam_conf.width);
    assert(cam_conf.height);

    if (DCMIPP_PipeInitCapture(conf, cam_conf.width, cam_conf.height, conf, two_pipes) == 0)
      return;

    printf("[CAM] pipe config failed (attempt %d/%d), reinitializing...\r\n",
           attempt, CAM_INIT_MAX_ATTEMPTS);
    assert(attempt < CAM_INIT_MAX_ATTEMPTS);
    CMW_CAMERA_DeInit();
    HAL_Delay(50);
  }
}

void CAM_CapturePipe_Start(uint8_t *capture_pipe_dst_pipe1, uint8_t *capture_pipe_dst_pipe2, uint32_t cam_mode, uint8_t two_pipes)
{
  int ret;

  ret = CMW_CAMERA_Start(DCMIPP_PIPE1, capture_pipe_dst_pipe1, cam_mode);
  assert(ret == CMW_ERROR_NONE);

  if(two_pipes)
  {
	  ret = CMW_CAMERA_Start(DCMIPP_PIPE2, capture_pipe_dst_pipe2, cam_mode);
	  assert(ret == CMW_ERROR_NONE);
  }

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

/* DCMIPP pipe error counter (overrun = corrupted line endings on the right
 * side of the image).  Displayed in the [REC] periodic report (app.c). */
volatile uint32_t dcmipp_err_count = 0;

void CMW_CAMERA_PIPE_ErrorCallback(uint32_t pipe)
{
  /* FIXME : Need to tune sensor/ipplug so we can remove this implementation */
  if (pipe == DCMIPP_PIPE1)
    dcmipp_err_count++;
}
