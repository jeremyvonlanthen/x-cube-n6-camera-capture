/**
 ******************************************************************************
 * @file    app.c
 * @brief   DIAS application core.
 *
 * Building blocks available in this module:
 *   - Camera capture through DCMIPP (pipe1 full-res, pipe2 downsized)
 *   - Hardware JPEG encoding (JPG_*) and transfer over UART to the Python GUI
 *   - H264 encoding (VENC hardware, ENC_*) of a 1280x720@30 RGB565 stream
 *   - MP4 (video) and JPEG (snapshot) recording to microSD (REC_*, FreeRTOS)
 *   - Runtime DCMIPP crop/decimation/downsize configuration received from
 *     the Python GUI over UART (Config_t)
 *
 * app_run() runs the DIAS state machine, on top of the helpers
 * (check_SD / app_mode_config_run).
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
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_cam.h"
#include "app_config.h"
#include "app_enc.h"
#include "app_jpg.h"
#include "app_rec.h"
#include "cmw_camera.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_dcmipp.h"
#include "stm32n6xx_ll_venc.h"
#ifdef STM32N6570_DK_REV
#include "stm32n6570_discovery.h"
#else
#include "stm32n6xx_nucleo.h"
#endif
#include "FreeRTOS.h"
#include "task.h"
#include "utils.h"

/* ==========================================================================
 * Constants & macros
 * ========================================================================== */

#define CACHE_ALIGN_SIZE(s)   (((s) + 31) & ~31)

/* Full-resolution sensor frame (IMX335) */
#define SENSOR_WIDTH          2592
#define SENSOR_HEIGHT         1944
#define SENSOR_WARMUP_FPS     10
#define WARMUP_FRAMES_TARGET  60      /* frames skipped so the AE/ISP converge */

/* Config protocol with the Python GUI (UART) */
#define CONFIG_MAGIC          0x12345678u
#define UART_TX_CHUNK_SIZE    32768

/* H264 recording configuration */
#define H264_WIDTH            1280
#define H264_HEIGHT           720
#define H264_FPS              30
#define H264_RECORD_SECONDS   10
#define H264_VENC_OUT_SIZE    (255 * 1024)
/* One RGB565 capture frame; two of them fit inside buffer_full_frame
 * (2 x 1.84 MB <= 9.7 MB) for double-buffered capture during recording. */
#define H264_FRAME_BYTES      (H264_WIDTH * H264_HEIGHT * 2)
/* Frames skipped before recording starts, to let the auto-exposure/ISP
 * reconverge after the switch to 720p30 (avoids a dark start of video). */
#define H264_AE_WARMUP_FRAMES 20

/* ==========================================================================
 * Types
 * ========================================================================== */

/* DCMIPP pipes configuration received from the Python GUI over UART */
typedef struct __attribute__((packed))
{
  uint32_t magic;               /* CONFIG_MAGIC */

  uint16_t crop_v_start_pipe1;
  uint16_t crop_v_size_pipe1;
  uint16_t crop_h_start_pipe1;
  uint16_t crop_h_size_pipe1;

  uint16_t crop_v_start_pipe2;
  uint16_t crop_v_size_pipe2;
  uint16_t crop_h_start_pipe2;
  uint16_t crop_h_size_pipe2;

  uint8_t decimation_ratio_pipe2;
  float downsize_ratio_pipe1;
  float downsize_ratio_pipe2;
} Config_t;

typedef enum
{
	SD_CARD,
	CONFIG_MODE_WARMUP,
	SEND_YUV_FRAME,
	RECEIVE_PIPES_CONFIG,
	DETECT_MODE_WARMUP,
	PIPES_CONFIGURATION,
	MOVEMENT_DETECTION,
	RECORDING
} state_t;

/* ==========================================================================
 * Module state
 * ========================================================================== */

/* External handles */
extern UART_HandleTypeDef huart1;                 /* main.c */
extern DCMIPP_HandleTypeDef hcamera_dcmipp;       /* app_cam.c */
/* DCMIPP pipe error counter (overruns = corrupted line endings),
 * incremented by CMW_CAMERA_PIPE_ErrorCallback in app_cam.c */
extern volatile uint32_t dcmipp_err_count;

/* Configuration received from the Python GUI */
static Config_t config_py = { 0 };

/* DIAS state machine (local: not shared with any ISR/callback) */
state_t state = SD_CARD;

/* Capture buffers (PSRAM) */
static uint8_t buffer_full_frame[MAX_CAPTURE_FRAME_SIZE] ALIGN_32 IN_PSRAM;
static uint8_t hires_jpeg_buffer[MAX_JPEG_FRAME_SIZE] ALIGN_32 IN_PSRAM;
/* H264 encoder output buffer (VENC hardware writes here) */
static uint8_t h264_venc_out[H264_VENC_OUT_SIZE] ALIGN_32 IN_PSRAM;

/* Detect-mode capture buffers, allocated from the AXISRAM pool once the
 * pipe configuration (hence the frame size) is known */
static uint8_t *buffer_pipe1_capture = NULL;
static uint8_t *buffer_pipe2_capture = NULL;
static uint8_t *buffer_warmup = NULL;
static uint32_t size_pipe1 = 0;
static uint32_t size_pipe2 = 0;

/* DCMIPP post-processing configuration */
static DCMIPP_CropConfTypeDef crop_conf = { 0 };
static DCMIPP_DownsizeTypeDef downsize_conf_pipe1 = { 0 };
static DCMIPP_DownsizeTypeDef downsize_conf_pipe2 = { 0 };
static DCMIPP_DecimationConfTypeDef decimation_conf = { 0 };
static JPG_conf_t jpg_conf = { 0 };

/* Capture/mode flags */
static volatile int snapshot_in_progress = 0;
static volatile int frame_ready = 0;
static volatile int warmup_frames = 0;
static volatile int warmup_done = 0;
static volatile int uart_busy = 0;        /* 1 = UART used for binary data, printf muted */

/* H264 recording state */
static volatile int h264_streaming = 0;   /* capture->encode pipeline active */
static volatile int h264_frame_ready = 0;
static volatile int force_intra = 0;
/* Double-buffered capture: address of the buffer just completed by DCMIPP
 * (read from the P1STM0AR status register in the frame event callback,
 * same pattern as the ST VENC_SDCard example). */
static uint8_t * volatile h264_ready_buf = NULL;

/* RTC (clocked on the internal LSI) used to build timestamped file names.
 * The wall-clock value is set by the Python GUI over UART (command 'T'). */
static RTC_HandleTypeDef hrtc;
static volatile int rtc_ready = 0;   /* 1 once HAL_RTC_Init succeeded */

/* ==========================================================================
 * Console & memory helpers
 * ========================================================================== */

int __io_putchar(int ch)
{
  if (!uart_busy)
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
  return ch;
}

/* Simple bump allocator in AXISRAM (allocations are never freed) */
__attribute__((section(".axisram_bss"))) static uint8_t axisram_pool[1 * 1024 * 1024];
static uint32_t axisram_offset = 0;

static void *axisram_alloc(uint32_t size)
{
  void *ptr;

  taskENTER_CRITICAL();

  /* Keep every allocation 32-byte aligned (cache line) */
  axisram_offset = (axisram_offset + 31) & ~31;

  if (axisram_offset + size > sizeof(axisram_pool)) {
    taskEXIT_CRITICAL();
    while (1) {} /* pool exhausted: trap */
  }
  ptr = &axisram_pool[axisram_offset];
  axisram_offset += size;

  taskEXIT_CRITICAL();

  return ptr;
}

/* ==========================================================================
 * RTC (timestamped file names)
 * ========================================================================== */

/* Initializes the RTC on the internal LSI (~32 kHz: no external crystal
 * required).  Non-fatal: on any failure rtc_ready stays 0 and the timestamp
 * helper falls back to a HAL_GetTick-based name.  The actual date/time must be
 * pushed by the GUI (command 'T'); until then the calendar counts from its
 * power-on default. */
static void rtc_init(void)
{
  RCC_OscInitTypeDef osc = { 0 };
  RCC_PeriphCLKInitTypeDef pclk = { 0 };

  HAL_PWR_EnableBkUpAccess();

  /* Enable LSI (leave the PLLs untouched) */
  osc.OscillatorType = RCC_OSCILLATORTYPE_LSI;
  osc.LSIState       = RCC_LSI_ON;
  osc.PLL1.PLLState  = RCC_PLL_NONE;
  osc.PLL2.PLLState  = RCC_PLL_NONE;
  osc.PLL3.PLLState  = RCC_PLL_NONE;
  osc.PLL4.PLLState  = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    printf("[RTC] LSI enable failed\r\n");
    return;
  }

  /* Route LSI to the RTC */
  pclk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  pclk.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) {
    printf("[RTC] clock select failed\r\n");
    return;
  }

  __HAL_RCC_RTC_ENABLE();
  __HAL_RCC_RTCAPB_CLK_ENABLE();

  /* LSI ~32 kHz -> (127+1) * (249+1) = 32000 for a 1 Hz calendar tick */
  hrtc.Instance            = RTC;
  hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv   = 127;
  hrtc.Init.SynchPrediv    = 249;
  hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp   = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode        = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK) {
    printf("[RTC] HAL_RTC_Init failed\r\n");
    return;
  }

  rtc_ready = 1;
  printf("[RTC] initialized on LSI\r\n");
}

/* Sets the RTC calendar from a 6-byte payload (year-2000, month, day, hour,
 * minute, second) received from the GUI. */
static void rtc_set_datetime(const uint8_t dt[6])
{
  RTC_TimeTypeDef t = { 0 };
  RTC_DateTypeDef d = { 0 };

  if (!rtc_ready)
    return;

  d.Year    = dt[0];             /* years since 2000 */
  d.Month   = dt[1];             /* 1..12 */
  d.Date    = dt[2];             /* 1..31 */
  d.WeekDay = RTC_WEEKDAY_MONDAY; /* unused for naming */
  t.Hours   = dt[3];
  t.Minutes = dt[4];
  t.Seconds = dt[5];

  HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN);
  HAL_RTC_SetDate(&hrtc, &d, RTC_FORMAT_BIN);
  printf("[RTC] set to 20%02u-%02u-%02u %02u:%02u:%02u\r\n",
         dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]);
}

/* Writes a file-name-safe timestamp "AAAA-MM-JJ_HH-MM-SS" into buf (needs >= 20
 * bytes).  ':' is illegal in FAT names so it is replaced by '-', and the
 * space by '_'.  Falls back to a tick-based name if the RTC is not ready. */
static void rtc_make_timestamp(char *buf, size_t n)
{
  RTC_TimeTypeDef t = { 0 };
  RTC_DateTypeDef d = { 0 };

  if (!rtc_ready) {
    snprintf(buf, n, "REC_%08lu", (unsigned long)HAL_GetTick());
    return;
  }

  /* GetTime MUST be called before GetDate to unlock the shadow registers */
  HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);

  snprintf(buf, n, "%04u-%02u-%02u_%02u-%02u-%02u",
           2000 + d.Year, d.Month, d.Date, t.Hours, t.Minutes, t.Seconds);
}

/* ==========================================================================
 * UART transfer helpers (protocol with the Python GUI)
 * ========================================================================== */

/* Sends one encoded JPEG over UART:
 *   0xAA | length (4 B, little endian) | JPEG data | exposure (4 B) | gain (4 B) */
static void send_jpeg_uart(const uint8_t *jpeg, int jpeg_len)
{
	uart_busy = 1;

  uint8_t sync = 0xAA;
  uint8_t size_buf[4];
  const uint8_t *src = jpeg;
  int remaining = jpeg_len;
  int32_t exposure = 0;
  int32_t gain = 0;

  if (jpeg_len <= 0)
    return;

  HAL_UART_Transmit(&huart1, &sync, 1, HAL_MAX_DELAY);

  size_buf[0] = (jpeg_len >> 0) & 0xFF;
  size_buf[1] = (jpeg_len >> 8) & 0xFF;
  size_buf[2] = (jpeg_len >> 16) & 0xFF;
  size_buf[3] = (jpeg_len >> 24) & 0xFF;
  HAL_UART_Transmit(&huart1, size_buf, 4, HAL_MAX_DELAY);

  while (remaining > 0) {
    int chunk = remaining > UART_TX_CHUNK_SIZE ? UART_TX_CHUNK_SIZE : remaining;
    HAL_UART_Transmit(&huart1, (uint8_t *)src, chunk, HAL_MAX_DELAY);
    src += chunk;
    remaining -= chunk;
  }

  CMW_CAMERA_GetExposure(&exposure);
  HAL_UART_Transmit(&huart1, (uint8_t *)&exposure, sizeof(exposure), HAL_MAX_DELAY);

  CMW_CAMERA_GetGain(&gain);
  HAL_UART_Transmit(&huart1, (uint8_t *)&gain, sizeof(gain), HAL_MAX_DELAY);

  uart_busy = 0;
}

/* ==========================================================================
 * Camera helpers
 * ========================================================================== */

/* (Re)initializes the camera at full resolution with the requested DCMIPP
 * output format, then lets the AE/ISP converge for WARMUP_FRAMES_TARGET
 * frames before stopping the pipe(s). */
static void camera_warmup(uint32_t output_format)
{
	static int camera_initialized = 0;

  CAM_conf_t cam_conf = { 0 };
  uint8_t two_pipes = (output_format == DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1);

  if(camera_initialized) CAM_Deinit();

  cam_conf.capture_width        = SENSOR_WIDTH;
  cam_conf.capture_height       = SENSOR_HEIGHT;
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

/* One full-resolution YUV422 snapshot (config mode), JPEG-encoded and sent
 * to the GUI over UART */
static int capture_yuv(void)
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

/* ==========================================================================
 * DCMIPP pipes configuration (detect mode)
 * ========================================================================== */

/* Applies the crop/decimation/downsize configuration received from the GUI
 * (config_py) to DCMIPP pipe1 and pipe2, then allocates the matching
 * capture buffers from the AXISRAM pool.
 * NOTE: axisram_alloc never frees — repeated detect-mode entries consume
 * the pool. */
static void dcmipp_apply_detect_config(void)
{
  /* The bump allocator never frees: reset it here, since the two capture
   * buffers below are its only users.  Without this, re-entering this state
   * after each recording would exhaust the 1 MB pool and trap. */
  axisram_offset = 0;

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

/* Takes one full-resolution snapshot (camera is still in the post-warmup
 * configuration), encodes it to JPEG (hardware) and saves it to the SD
 * card as IMG_xxxx.JPG through the FreeRTOS SD writer task (app_rec.c).
 * Called on the TAMP button, right before record_h264_sd(). */
static void record_jpeg_sd(const char *timestamp)
{
  char fname[40];
  int jpeg_len;
  uint32_t start;

  snprintf(fname, sizeof(fname), "%s.jpg", timestamp);

  /* Full-sensor COLOR snapshot while the camera runs in detect (mono,
   * cropped/downsized) mode: reconfigure PIPE1 ONLY to full-frame YUV422.
   * The sensor is untouched, so the AE/exposure already converged during
   * the detect warmup stay valid -> no delay, color is immediate.
   * No restore needed: record_h264_sd() reconfigures the camera right
   * after, and DETECT_MODE_WARMUP + PIPES_CONFIGURATION re-apply the
   * detect setup once the recording is done. */
  CAM_Pipe1_SetFormat(SENSOR_WIDTH, SENSOR_HEIGHT, DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1);

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

  SCB_InvalidateDCache_by_Addr((uint32_t *)buffer_full_frame, CACHE_ALIGN_SIZE(MAX_CAPTURE_FRAME_SIZE));

  /* Hardware JPEG encode: pipe1 was switched to full-frame YUV422 above */
  jpg_conf.width      = SENSOR_WIDTH;
  jpg_conf.height     = SENSOR_HEIGHT;
  jpg_conf.fmt_src    = JPG_SRC_YUV422; //JPG_SRC_GREY;
  jpg_conf.full_width = SENSOR_WIDTH;
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

/* Records H264_RECORD_SECONDS seconds of 1280x720@30 H264 video into a new
 * VID_xxxx.MP4 on the SD card, then restores the camera state (clears
 * warmup_done so the current mode re-enters from scratch). */
static void record_h264_sd(const char *timestamp)
{
  /* LL_VENC_Init and ENC_Init must each be called exactly once —
   * ENC_DeInit crashes on this target.  Init once on first entry,
   * reuse on every subsequent call (same pattern as the USB phase). */
  static int hw_initialized = 0;

  CAM_conf_t cam_conf = { 0 };
  ENC_Conf_t enc_conf;
  char fname[40];

  snprintf(fname, sizeof(fname), "%s.mp4", timestamp);

  /* Switch camera to 1280x720 RGB565 @ 30 fps for H264.
   * RGB565 (2 B/px) halves PSRAM bandwidth vs ARGB8888: fixes DCMIPP
   * pixel-packer overruns (right-side line artifacts).  Encoder preproc
   * is set to H264ENC_RGB565 accordingly (app_enc.c). */
  CAM_Deinit();
  cam_conf.capture_width        = H264_WIDTH;
  cam_conf.capture_height       = H264_HEIGHT;
  cam_conf.fps                  = H264_FPS;
  cam_conf.dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1;
  cam_conf.is_rgb_swap          = 0;
  CAM_Init(&cam_conf, 0);

  if (!hw_initialized) {
    /* VENC hardware — assert-fails if called twice */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    LL_VENC_Init();

    /* H264 software encoder */
    ENC_ResetAllocator();
    enc_conf.width  = H264_WIDTH;
    enc_conf.height = H264_HEIGHT;
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

  /* Open the MP4 file and start the muxer.
   * Ring buffer: the recording capture only uses the first 2 RGB565
   * frames (2 x 1.84 MB) of buffer_full_frame (sized 9.7 MB for the
   * full-res config mode) — the remaining ~6 MB buffer several seconds
   * of encoded video, absorbing long SD card pauses without dropping. */
  if (REC_Start(H264_WIDTH, H264_HEIGHT, H264_FPS,
                buffer_full_frame + 2 * H264_FRAME_BYTES,
                MAX_CAPTURE_FRAME_SIZE - 2 * H264_FRAME_BYTES,
                fname) != 0) {
    printf("[REC] record start failed, recording aborted\r\n");
    warmup_done = 0;
    return;
  }

  /* Start double-buffered continuous capture (two 720p frames inside
   * buffer_full_frame).  Single-buffer capture caused tearing artifacts on
   * the right side of the image: VENC was reading the frame while DCMIPP
   * was still overwriting it. */
  h264_streaming = 1;
  {
    int ret = CMW_CAMERA_DoubleBufferStart(DCMIPP_PIPE1,
                                           buffer_full_frame,
                                           buffer_full_frame + H264_FRAME_BYTES,
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
  printf("[REC] recording started (%d s)...\r\n", H264_RECORD_SECONDS);

  /* Record for H264_RECORD_SECONDS seconds */
  uint32_t start_tick = HAL_GetTick();
  uint32_t last_frame_tick = start_tick;
  uint32_t frame_count = 0;
  uint32_t encode_ok_count = 0;

  while (HAL_GetTick() - start_tick < (uint32_t)(H264_RECORD_SECONDS * 1000)) {
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
  printf("[REC] recording ended: frames=%lu encOK=%lu dcmippErr=%lu\r\n",
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
    printf("[REC] MP4 file finalized OK\r\n");
  else
    printf("[REC] MP4 finalize FAILED\r\n");

  /* Re-enter current mode from scratch */
  warmup_done = 0;
}

/* ==========================================================================
 * Camera & button callbacks
 * ========================================================================== */

int CMW_CAMERA_PIPE_FrameEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1) {
    if (h264_streaming) {
      /* Double-buffer mode: P1STM0AR (status reg) holds the address of the
       * buffer the hardware just completed (VENC_SDCard example pattern). */
      h264_ready_buf = (uint8_t *)hcamera_dcmipp.Instance->P1STM0AR;
      h264_frame_ready = 1;
    }
    else if (!warmup_done)
      warmup_frames++;
    else if (snapshot_in_progress)
      frame_ready = 1;
  }
  return HAL_OK;
}

int CMW_CAMERA_PIPE_VsyncEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    CAM_IspUpdate();
  return HAL_OK;
}

void BSP_PB_Callback(Button_TypeDef Button)
{
  if(Button == BUTTON_USER1) state = CONFIG_MODE_WARMUP;

}

/* ==========================================================================
 * Public API (building blocks for the state machine)
 * ========================================================================== */

/* One-time peripheral init: LEDs, TAMP button (polling) and SD recorder
 * (SD card + FAT32 mount + FreeRTOS SD writer task). */
int check_SD(void)
{
	int rec_ready = REC_Init();
	switch(rec_ready)
	{
	case 0:
		printf("[uSD] uSD init successful\r\n");
		break;
	case -1:
		printf("[uSD] f_mount failed - card FAT32-formatted?\r\n");
		break;
	case -2:
		printf("[uSD] uSD init failed - card inserted?\r\n");
		break;
	case -3:
		printf("[uSD] SDMMC2 clock config failed\r\n");
		break;
	default:
		break;
	}

  return (rec_ready == 0);
}

void LED_mode(void)
{
	if(state < MOVEMENT_DETECTION){ // configuration process
		BSP_LED_Off(LED_GREEN);
		BSP_LED_On(LED_RED);
	}
	else{ // detection phase
		BSP_LED_On(LED_GREEN);
		BSP_LED_Off(LED_RED);
	}
}

/* ==========================================================================
 * Application entry point
 * ========================================================================== */

void app_run(void)
{
	printf("[FSM] please press USER1 to restart from config mode warmup\r\n");

	/* TAMP button read by polling in MOVEMENT_DETECTION */
	BSP_PB_Init(BUTTON_TAMP, BUTTON_MODE_GPIO);

	/* RTC for timestamped file names (non-fatal if it fails) */
	rtc_init();

	while(1)
	{
		LED_mode();

		switch(state)
		{
		case SD_CARD:
			if(check_SD()){
				state = CONFIG_MODE_WARMUP;
				break;
			}
			HAL_Delay(2000);
			break;

		case CONFIG_MODE_WARMUP:
			printf("[FSM] config mode warmup\r\n");
			camera_warmup(DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1);
			printf("[FSM] warmup ended\r\n");

			state = SEND_YUV_FRAME;
			printf("[FSM] wait for send yuv frame...\r\n");
			break;

		case SEND_YUV_FRAME:
			uint8_t cmd = 0;
			HAL_UART_Receive(&huart1, &cmd, 1, 100);

			if (cmd == 'S'){
				int jpeg_len = capture_yuv();
				send_jpeg_uart(hires_jpeg_buffer, jpeg_len);
			}
			else if (cmd == 'T'){
				/* Set RTC date/time: 6 bytes (year-2000, month, day,
				 * hour, minute, second) sent right after the 'T'. */
				uint8_t dt[6] = { 0 };
				if (HAL_UART_Receive(&huart1, dt, sizeof(dt), 1000) == HAL_OK)
					rtc_set_datetime(dt);
				printf("[FSM] RTC set\r\n");
			}
			else if(cmd == 'V'){
				state = RECEIVE_PIPES_CONFIG;
			}
			break;

		case RECEIVE_PIPES_CONFIG:
			uint8_t buffer[sizeof(Config_t)];
			uint8_t answer = 'F'; //F = Fail

			HAL_UART_Receive(&huart1, buffer, sizeof(Config_t), 100);
			memcpy(&config_py, buffer, sizeof(Config_t));
			if (config_py.magic == CONFIG_MAGIC){
				answer = 'V';
				HAL_UART_Transmit(&huart1, &answer, 1, 100);

				state = DETECT_MODE_WARMUP;
				break;
			}
			HAL_UART_Transmit(&huart1, &answer, 1, 100);
			break;

		case DETECT_MODE_WARMUP:
			printf("[FSM] start detection mode warmup\r\n");
			camera_warmup(DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1);
			printf("[FSM] warmup ended\r\n");

			state = PIPES_CONFIGURATION;
			break;

		case PIPES_CONFIGURATION:
			printf("[FSM] pipes configuration\r\n");
			dcmipp_apply_detect_config();

			state = MOVEMENT_DETECTION;
			printf("[FSM] start movement detection... (TAMP button)\r\n");
			break;

		case MOVEMENT_DETECTION:
			if(BSP_PB_GetState(BUTTON_TAMP) == GPIO_PIN_SET){
				BSP_LED_Off(LED_GREEN);
				printf("[FSM] movement detected!\r\n");
				state = RECORDING;
				break;
			}
			HAL_Delay(1000);
			break;

		case RECORDING:
			/* One timestamp for this event, shared by the JPEG and the MP4 so
			 * both files carry the same "AAAA-MM-JJ_HH-MM-SS" base name. */
			char timestamp[20];
			rtc_make_timestamp(timestamp, sizeof(timestamp));

			record_jpeg_sd(timestamp);
			record_h264_sd(timestamp);
			/* record_h264_sd() left the camera in 720p RGB565 and cleared
			 * warmup_done: go through a full detect warmup again (the pipes
			 * config is re-applied after it). */
			BSP_LED_On(LED_GREEN);
			state = DETECT_MODE_WARMUP;
			break;

		default:
			break;
		}
	}
}
