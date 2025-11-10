 /**
 ******************************************************************************
 * @file    app.c
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "app.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "app_cam.h"
#include "app_config.h"
#include "app_jpg.h"
#include "cmw_camera.h"
#include "stm32n6xx_hal.h"
#include "ulist.h"
#ifdef STM32N6570_DK_REV
#include "stm32n6570_discovery.h"
#else
#include "stm32n6xx_nucleo.h"
#endif
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "utils.h"
#include "uvcl.h"

/* Disable jpeg support for nucleo due to lack of memory */
#ifdef STM32N6570_NUCLEO_REV
#define DISABLE_JPEG 1
#endif

#ifndef APP_VERSION_STRING
#define APP_VERSION_STRING "dev"
#endif

#define CAPT_BUFFER_NB                2
#define JPEG_BUFFER_NB                CAPT_BUFFER_NB

#define FREERTOS_PRIORITY(p) ((UBaseType_t)((int)tskIDLE_PRIORITY + configMAX_PRIORITIES / 2 + (p)))

struct list_protected {
  struct ulist head;
  SemaphoreHandle_t lock;
  StaticSemaphore_t lock_buffer;
  SemaphoreHandle_t sem;
  StaticSemaphore_t sem_buffer;
  int level_dbg;
};

struct buffer {
  struct ulist list;
  uint8_t *buffer;
  int size;
  int len;
  int is_jpeg;
};

struct streaming_ctx {
  int counter;
  int is_streaming;
  UVCL_StreamConf_t stream;
};

struct streaming_req {
  struct streaming_ctx ctx;
  SemaphoreHandle_t lock;
  StaticSemaphore_t lock_buffer;
};

/* threads */
 /* uvc thread */
static StaticTask_t uvc_thread;
static StackType_t uvc_thread_stack[2 * configMINIMAL_STACK_SIZE];
  /* isp thread */
static StaticTask_t isp_thread;
static StackType_t isp_thread_stack[2 * configMINIMAL_STACK_SIZE];
static SemaphoreHandle_t isp_sem;
static StaticSemaphore_t isp_sem_buffer;
/* capture thread */
static StaticTask_t capture_thread;
static StackType_t capture_thread_stack[configMINIMAL_STACK_SIZE];
static SemaphoreHandle_t capture_sem;
static StaticSemaphore_t capture_sem_buffer;

/* capture buffers */
static uint8_t capture_buffer[CAPT_BUFFER_NB][MAX_IMG_FRAME_SIZE] ALIGN_32 IN_PSRAM;
static struct buffer capture[CAPT_BUFFER_NB];
#ifndef DISABLE_JPEG
/* jpg */
static uint8_t jpeg_buffer[JPEG_BUFFER_NB][MAX_IMG_FRAME_SIZE] ALIGN_32 IN_PSRAM;
static struct buffer jpeg[JPEG_BUFFER_NB];
#endif

/* uvc */
static struct uvcl_callbacks uvcl_cbs;

/* list heads */
struct list_protected capt_free_buffers;
struct list_protected capt_capturing_buffers;
struct list_protected capt_ready_buffers;
struct list_protected uvc_in_use_buffers;
struct list_protected jpeg_free_buffers;

static struct streaming_req streaming_req;

static int sr_init(struct streaming_req *sr)
{
  sr->ctx.counter = 0;
  sr->ctx.is_streaming = 0;
  sr->lock = xSemaphoreCreateMutexStatic(&sr->lock_buffer);
  assert(sr->lock);

  return 0;
}

static void sr_lock(struct streaming_req *sr)
{
  int ret;

  ret = xSemaphoreTake(sr->lock, portMAX_DELAY);
  assert(ret == pdTRUE);
}

static void sr_unlock(struct streaming_req *sr)
{
  int ret;

  ret = xSemaphoreGive(sr->lock);
  assert(ret == pdTRUE);
}

static void sr_set_streaming_active(struct streaming_req *sr, UVCL_StreamConf_t *stream)
{
  sr_lock(sr);
  sr->ctx.counter++;
  sr->ctx.is_streaming = 1;
  sr->ctx.stream = *stream;
  sr_unlock(sr);
}

static void sr_set_streaming_inactive(struct streaming_req *sr)
{
  sr_lock(sr);
  sr->ctx.counter++;
  sr->ctx.is_streaming = 0;
  sr_unlock(sr);
}

static int sr_is_streaming(struct streaming_req *sr, struct streaming_ctx *ctx)
{
  int res;

  sr_lock(sr);
  res = sr->ctx.is_streaming;
  if (res)
    *ctx = sr->ctx;
  sr_unlock(sr);

  return res;
}

static int sr_is_valid(struct streaming_req *sr, struct streaming_ctx *ctx)
{
  return sr->ctx.counter == ctx->counter;
}

static int lp_init(struct list_protected *lp)
{
  ulist_init_head(&lp->head);
  lp->lock = xSemaphoreCreateMutexStatic(&lp->lock_buffer);
  assert(lp->lock);
  lp->sem = xSemaphoreCreateCountingStatic(CAPT_BUFFER_NB, 0, &lp->sem_buffer);
  assert(lp->sem);
  lp->level_dbg = 0;

  return 0;
}

static void lp_lock(struct list_protected *lp)
{
  int ret;

  ret = xSemaphoreTake(lp->lock, portMAX_DELAY);
  assert(ret == pdTRUE);
}

static void lp_unlock(struct list_protected *lp)
{
  int ret;

  ret = xSemaphoreGive(lp->lock);
  assert(ret == pdTRUE);
}

static int lp_push(struct list_protected *lp, struct buffer *buffer)
{
  int ret;

  assert(!xPortIsInsideInterrupt());

  lp_lock(lp);
  ulist_add_tail(&buffer->list, &lp->head);
  lp->level_dbg++;
  lp_unlock(lp);

  ret = xSemaphoreGive(lp->sem);
  assert(ret == pdTRUE);

  return 0;
}

static struct buffer *lp_pop(struct list_protected *lp, int is_blocking)
{
  struct buffer *res;
  int ret;

  assert(!xPortIsInsideInterrupt());

  ret = xSemaphoreTake(lp->sem, is_blocking ? portMAX_DELAY : 0);
  if (ret == pdFALSE)
    return NULL;

  lp_lock(lp);
  res = ulist_entry(lp->head.next, struct buffer, list);
  lp->level_dbg--;
  assert(res);
  ulist_del(&res->list);
  lp_unlock(lp);

  return res;
}

static struct buffer *lp_remove_buffer(struct list_protected *lp, uint8_t *buffer)
{
  struct buffer *res = NULL;
  struct buffer *current;
  struct buffer *tmp;
  int ret;

  assert(!xPortIsInsideInterrupt());

  ret = xSemaphoreTake(lp->sem, 0);
  assert(ret == pdTRUE);

  lp_lock(lp);
  ulist_for_each_entry_safe(current, tmp, &lp->head, list) {
    if (current->buffer != buffer)
      continue;
    res = current;
    assert(res);
    ulist_del(&res->list);
    break;
  }
  lp->level_dbg--;
  lp_unlock(lp);

  return res;
}

static int uvcl_payload_to_dcmipp_type(int payload_type)
{
  switch (payload_type) {
  case UVCL_PAYLOAD_UNCOMPRESSED_YUY2:
    return DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
    break;
  case UVCL_PAYLOAD_FB_JPEG:
  case UVCL_PAYLOAD_JPEG:
    return DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
    break;
  case UVCL_PAYLOAD_FB_RGB565:
    return DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1;
    break;
  case UVCL_PAYLOAD_FB_BGR3:
    return DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1;
    break;
  case UVCL_PAYLOAD_FB_GREY:
    return DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1;
    break;
  case UVCL_PAYLOAD_FB_GREY_D3DFMT_L8:
    return DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1;
    break;
  default:
    assert(0);
  }

  return DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
}

static int uvcl_payload_to_bpp(int payload_type)
{
  switch (payload_type) {
  case UVCL_PAYLOAD_UNCOMPRESSED_YUY2:
    return 2;
    break;
  case UVCL_PAYLOAD_FB_JPEG:
  case UVCL_PAYLOAD_JPEG:
    return 1;
    break;
  case UVCL_PAYLOAD_FB_RGB565:
    return 2;
    break;
  case UVCL_PAYLOAD_FB_BGR3:
    return 3;
    break;
  case UVCL_PAYLOAD_FB_GREY:
    return 1;
    break;
  case UVCL_PAYLOAD_FB_GREY_D3DFMT_L8:
    return 1;
    break;
  default:
    assert(0);
  }

  return DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
}

static const char *uvcl_payload_to_name(int payload_type)
{
  switch (payload_type) {
  case UVCL_PAYLOAD_UNCOMPRESSED_YUY2:
    return "YUV2";
    break;
  case UVCL_PAYLOAD_FB_JPEG:
    return "JPEG_FB";
    break;
  case UVCL_PAYLOAD_JPEG:
    return "JPEG";
    break;
  case UVCL_PAYLOAD_FB_RGB565:
    return "RGB565";
    break;
  case UVCL_PAYLOAD_FB_BGR3:
    return "BGR3";
    break;
  case UVCL_PAYLOAD_FB_GREY:
    return "GREY";
    break;
  case UVCL_PAYLOAD_FB_GREY_D3DFMT_L8:
    return "GREY_L8";
    break;
  default:
    assert(0);
  }

  return "UNKNOWN";
}

static void app_main_pipe_vsync_event()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int ret;

  ret = xSemaphoreGiveFromISR(isp_sem, &xHigherPriorityTaskWoken);
  if (ret == pdTRUE)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void isp_thread_fct(void *arg)
{
  int ret;

  while (1) {
    ret = xSemaphoreTake(isp_sem, portMAX_DELAY);
    assert(ret == pdTRUE);

    CAM_IspUpdate();
  }
}

static void app_main_pipe_frame_event()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int ret;

  ret = xSemaphoreGiveFromISR(capture_sem, &xHigherPriorityTaskWoken);
  if (ret == pdTRUE)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void capture_thread_fct(void *arg)
{
  struct buffer *current_buffer;
  struct buffer *next_buffer;
  int ret;

  while (1) {
    ret = xSemaphoreTake(capture_sem, portMAX_DELAY);
    assert(ret == pdTRUE);

    next_buffer = lp_pop(&capt_free_buffers, 0);
    if (next_buffer) {
      ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE1,
                                             DCMIPP_MEMORY_ADDRESS_0, (uint32_t) next_buffer->buffer);
      assert(ret == HAL_OK);
      current_buffer = lp_pop(&capt_capturing_buffers, 0);
      assert(current_buffer);
      lp_push(&capt_capturing_buffers, next_buffer);
      lp_push(&capt_ready_buffers, current_buffer);
    }
  }
}

static void app_uvc_streaming_active(struct uvcl_callbacks *cbs, UVCL_StreamConf_t stream)
{
  sr_set_streaming_active(&streaming_req, &stream);

  BSP_LED_On(LED_RED);
}

static void app_uvc_streaming_inactive(struct uvcl_callbacks *cbs)
{
  sr_set_streaming_inactive(&streaming_req);

  BSP_LED_Off(LED_RED);
}

static void app_uvc_frame_release(struct uvcl_callbacks *cbs, void *frame)
{
  struct buffer *buffer;

  buffer = lp_remove_buffer(&uvc_in_use_buffers, frame);
  assert(buffer);

  if (buffer->is_jpeg)
    lp_push(&jpeg_free_buffers, buffer);
  else
    lp_push(&capt_free_buffers, buffer);
}

static void UVC_Init()
{
  const UVCL_StreamConf_t streams[] = IMG_STREAMS;
  UVCL_Conf_t uvcl_conf = { 0 };
  int ret;
  int i;

  for (i = 0; i < ARRAY_NB(streams); i++)
    uvcl_conf.streams[i] = streams[i];
  uvcl_conf.streams_nb = ARRAY_NB(streams);
  uvcl_conf.is_immediate_mode = 1;
  uvcl_cbs.streaming_active = app_uvc_streaming_active;
  uvcl_cbs.streaming_inactive = app_uvc_streaming_inactive;
  uvcl_cbs.frame_release = app_uvc_frame_release;
  ret = UVCL_Init(USB1_OTG_HS, &uvcl_conf, &uvcl_cbs);
  assert(ret == 0);
}

static void send_raw_frame(struct buffer *buffer)
{
  int ret;

  ret = lp_push(&uvc_in_use_buffers, buffer);
  assert(ret == 0);
  ret = UVCL_ShowFrame(buffer->buffer, buffer->len);
  if (ret) {
    buffer = lp_remove_buffer(&uvc_in_use_buffers, buffer->buffer);
    assert(buffer);
    lp_push(&capt_free_buffers, buffer);
  }
}

static void send_jpg_frame(struct buffer *buffer)
{
#ifndef DISABLE_JPEG
  struct buffer *jpeg_buffer;
  int ret;

  jpeg_buffer = lp_pop(&jpeg_free_buffers, 0);
  if (!jpeg_buffer) {
    ret = lp_push(&capt_free_buffers, buffer);
    assert(ret == 0);
    return ;
  }

  ret = JPG_Encode(jpeg_buffer->buffer, buffer->buffer, jpeg_buffer->size, buffer->len);
  assert(ret > 0);
  jpeg_buffer->len = ret;

  ret = lp_push(&capt_free_buffers, buffer);
  assert(ret == 0);
  ret = lp_push(&uvc_in_use_buffers, jpeg_buffer);
  assert(ret == 0);

  ret = UVCL_ShowFrame(jpeg_buffer->buffer, jpeg_buffer->len);
  if (ret) {
    jpeg_buffer = lp_remove_buffer(&uvc_in_use_buffers, jpeg_buffer->buffer);
    assert(jpeg_buffer);
    lp_push(&jpeg_free_buffers, jpeg_buffer);
  }
#else
  assert(0);
#endif
}

static void JPEG_Init(UVCL_StreamConf_t *stream)
{
#ifndef DISABLE_JPEG
  JPG_conf_t jpg_conf = { 0 };
  int ret;

  jpg_conf.width = stream->width;
  jpg_conf.height = stream->height;
  jpg_conf.fmt_src = JPG_SRC_YUV422;

  ret = JPG_Init(&jpg_conf);
  assert(ret == 0);
#else
  assert(0);
#endif
}

static void capture_init(UVCL_StreamConf_t *current, int is_jpeg)
{
  CAM_conf_t cam_conf = { 0 };
  struct buffer *buffer;
  int bpp;
  int ret;
  int i;

  if (is_jpeg)
    JPEG_Init(current);

  /* start camera pipeline */
   /* init */
  cam_conf.capture_width = current->width;
  cam_conf.capture_height = current->height;
  cam_conf.fps = current->fps;
  cam_conf.dcmipp_output_format = uvcl_payload_to_dcmipp_type(current->payload_type);
  cam_conf.is_rgb_swap = 0;
  CAM_Init(&cam_conf);

  bpp = uvcl_payload_to_bpp(current->payload_type);
  for (i = 0; i < CAPT_BUFFER_NB; i++)
    capture[i].len = current->width * current->height * bpp;
  buffer = lp_pop(&capt_free_buffers, 1);
  assert(buffer);
  ret = lp_push(&capt_capturing_buffers, buffer);
  assert(ret == 0);

  CAM_CapturePipe_Start(buffer->buffer, CMW_MODE_CONTINUOUS);
}

static void capture_deinit(int is_jpeg)
{
  struct buffer *buffer;

  CAM_Deinit();
  if (is_jpeg)
    JPG_Deinit();

  do {
    buffer = lp_pop(&capt_capturing_buffers, 0);
    if (buffer) {
      lp_push(&capt_free_buffers, buffer);
    }
  } while (buffer);

  do {
    buffer = lp_pop(&capt_ready_buffers, 0);
    if (buffer) {
      lp_push(&capt_free_buffers, buffer);
    }
  } while (buffer);
}

static void uvc_thread_fct(void *arg)
{
  struct streaming_ctx current = { 0 };
  struct buffer *buffer;
  int is_jpeg;

  while (1) {
    /* wait for stream request */
    while (!sr_is_streaming(&streaming_req, &current)) {
      HAL_Delay(1);
    }

    /* copy request */
    is_jpeg = current.stream.payload_type == UVCL_PAYLOAD_JPEG || current.stream.payload_type == UVCL_PAYLOAD_FB_JPEG;
    printf("Start streaming %s | %dx%d@%dfps\n", uvcl_payload_to_name(current.stream.payload_type), current.stream.width,
           current.stream.height, current.stream.fps);

    capture_init(&current.stream, is_jpeg);
    /* looping until no more streaming */
    while (1) {
      do {
        buffer = lp_pop(&capt_ready_buffers, 0);
        if (!buffer)
          HAL_Delay(1);
      } while (!buffer && sr_is_valid(&streaming_req, &current));

      if (!buffer)
        break;

      if (is_jpeg)
        send_jpg_frame(buffer);
      else
        send_raw_frame(buffer);
    }

    printf("End streaming\n");
    capture_deinit(is_jpeg);
  }
}

static void LIST_Init()
{
  int ret;
  int i;

  ret = lp_init(&capt_free_buffers);
  assert(ret == 0);
  ret = lp_init(&capt_capturing_buffers);
  assert(ret == 0);
  ret = lp_init(&capt_ready_buffers);
  assert(ret == 0);
  ret = lp_init(&uvc_in_use_buffers);
  assert(ret == 0);
  ret = lp_init(&jpeg_free_buffers);
  assert(ret == 0);

  for (i = 0; i < CAPT_BUFFER_NB; i++) {
    capture[i].buffer = capture_buffer[i];
    capture[i].size = MAX_IMG_FRAME_SIZE;
    capture[i].is_jpeg = 0;
    ret = lp_push(&capt_free_buffers, &capture[i]);
    assert(ret == 0);
  }

#ifndef DISABLE_JPEG
  for (i = 0; i < JPEG_BUFFER_NB; i++) {
    jpeg[i].buffer = jpeg_buffer[i];
    jpeg[i].size = MAX_IMG_FRAME_SIZE;
    jpeg[i].is_jpeg = 1;
    ret = lp_push(&jpeg_free_buffers, &jpeg[i]);
    assert(ret == 0);
  }
#endif
}

static void app_display_info_header()
{
  printf("========================================\n");
  printf("x-cube-n6-camera-capture v2.0.0 (%s)\n", APP_VERSION_STRING);
  printf("Build date & time: %s %s\n", __DATE__, __TIME__);
#if defined(__GNUC__)
  printf("Compiler: GCC %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#elif defined(__ICCARM__)
  printf("Compiler: IAR EWARM %d.%d.%d\n", __VER__ / 1000000, (__VER__ / 1000) % 1000 ,__VER__ % 1000);
#else
  printf("Compiler: Unknown\n");
#endif
  printf("HAL: %lu.%lu.%lu\n", __STM32N6xx_HAL_VERSION_MAIN, __STM32N6xx_HAL_VERSION_SUB1, __STM32N6xx_HAL_VERSION_SUB2);
  printf("========================================\n");
}

void app_run()
{
  UBaseType_t capture_priority = FREERTOS_PRIORITY(3);
  UBaseType_t isp_priority = FREERTOS_PRIORITY(2);
  UBaseType_t uvc_priority = FREERTOS_PRIORITY(0);
  TaskHandle_t hdl;

  app_display_info_header();
  /* Enable DWT so DWT_CYCCNT works when debugger not attached */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  LIST_Init();
  sr_init(&streaming_req);

  /* UVC ss */
  UVC_Init();

  /* sems + mutex init */
  isp_sem = xSemaphoreCreateCountingStatic(1, 0, &isp_sem_buffer);
  assert(isp_sem);
  capture_sem = xSemaphoreCreateCountingStatic(1, 0, &capture_sem_buffer);
  assert(capture_sem);

  /* threads init */
  hdl = xTaskCreateStatic(uvc_thread_fct, "uvc", configMINIMAL_STACK_SIZE * 2, NULL, uvc_priority, uvc_thread_stack,
                          &uvc_thread);
  assert(hdl != NULL);
  hdl = xTaskCreateStatic(isp_thread_fct, "isp", configMINIMAL_STACK_SIZE * 2, NULL, isp_priority, isp_thread_stack,
                          &isp_thread);
  assert(hdl != NULL);
  hdl = xTaskCreateStatic(capture_thread_fct, "capture", configMINIMAL_STACK_SIZE, NULL, capture_priority, capture_thread_stack,
                          &capture_thread);
  assert(hdl != NULL);

  BSP_LED_On(LED_GREEN);
}

int CMW_CAMERA_PIPE_FrameEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    app_main_pipe_frame_event();

  return HAL_OK;
}

int CMW_CAMERA_PIPE_VsyncEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    app_main_pipe_vsync_event();

  return HAL_OK;
}
