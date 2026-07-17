/**
 ******************************************************************************
 * @file    app_shared.h
 * @brief   Etat, types et constantes partages entre les modules DIAS.
 *          Les definitions de l'etat partage sont dans app.c.
 ******************************************************************************
 */
#ifndef APP_SHARED_H
#define APP_SHARED_H

#include <stdint.h>
#include "stm32n6xx_hal.h"
#include "app_jpg.h"        /* JPG_conf_t */

/* --- Constantes partagees --- */
#define SENSOR_WIDTH          2592
#define SENSOR_HEIGHT         1944
#define SENSOR_WARMUP_FPS     10
#define WARMUP_FRAMES_TARGET  60      /* frames skipped so the AE/ISP converge */

#define CONFIG_MAGIC          0x12345678u
#define CACHE_ALIGN_SIZE(s)   (((s) + 31) & ~31)

/* --- Types partages --- */
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

/* --- Handles communs (definis ailleurs) --- */
extern UART_HandleTypeDef huart1;              /* main.c */
extern DCMIPP_HandleTypeDef hcamera_dcmipp;    /* app_cam.c */
extern volatile uint32_t dcmipp_err_count;     /* app_cam.c */

/* --- Etat partage (defini dans app.c) --- */
extern Config_t config_py;
extern state_t  state;

extern uint8_t  buffer_full_frame[];
extern uint8_t  hires_jpeg_buffer[];
extern uint8_t *buffer_warmup;
extern JPG_conf_t jpg_conf;

extern volatile int snapshot_in_progress;
extern volatile int frame_ready;
extern volatile int warmup_frames;
extern volatile int warmup_done;
extern volatile int uart_busy;

extern volatile int h264_streaming;
extern volatile int h264_frame_ready;
extern volatile int force_intra;
extern uint8_t * volatile h264_ready_buf;
extern uint32_t actual_ticks;

/* --- Allocateur AXISRAM (defini dans app.c) --- */
void *axisram_alloc(uint32_t size);
void  axisram_reset(void);

#endif /* APP_SHARED_H */
