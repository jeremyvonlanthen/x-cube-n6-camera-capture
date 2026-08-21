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
#include "app_jpg.h"

/* --- Constantes partagees --- */
#define SENSOR_WIDTH          2592
#define SENSOR_HEIGHT         1944
#define SENSOR_WARMUP_FPS     5
#define WARMUP_FRAMES_TARGET  10      /* frames skipped so the AE/ISP converge */

#define CONFIG_MAGIC          0x12345678u
#define CACHE_ALIGN_SIZE(s)   (((s) + 31) & ~31)

/* DEBUG_MODE
 * 0: measurement mode: play with `RUN` and disconnect ST-Link USB
 * 1: debugging mode: play with `Debug` but assume extra consumption current */
#define DEBUG_MODE 0

/* SD_LOW_POWER_TEST
 * Isolated one-shot SD power-state test: initializes the SD card once
 * (SD_CARD state), puts it in the selected state below, then parks in
 * low-power SLEEP forever -- no camera/DCMIPP/detection activity at all, so
 * the SD's own contribution to board current (a couple mA at most) isn't
 * buried under the ~200 mA MOVEMENT_DETECTION load. The FSM never proceeds
 * past SD_CARD. Flash, disconnect ST-Link USB (see DEBUG_MODE above), and
 * measure board current; repeat for each value to compare:
 *   0: normal operation (test disabled)
 *   1: SD left active/awake (baseline)
 *   2: SD asleep  -- REC_SleepSD(): SDMMC2 clock gated only, card selected
 *   3: SD powered down -- REC_PowerDownSD(): clock + GPIOC/E off, unmounted
 * For a 4th "card physically removed" data point, pull the microSD card
 * while parked in mode 1 or 2 (nothing in this test reacts to it). */
#define SD_LOW_POWER_TEST 3

/* LPTIM1 wake-up timer clock frequency, in Hz, used to convert a requested
 * sleep duration (ms) into an auto-reload tick count. The LSI is an
 * uncalibrated RC oscillator: its datasheet-nominal ~32kHz can be off by
 * 30-40% in practice.
 *
 * How to recalibrate this constant (needed again if you change board, unit,
 * or run at a very different temperature -- LSI drifts with all three):
 *   1. Pick a strategy that sleeps for a known, fixed duration each cycle
 *      (e.g. force sleep_duration_ms to a constant like 1000 for the test).
 *   2. Measure the ACTUAL elapsed sleep time with something independent of
 *      this firmware's own clock (oscilloscope/logic analyzer on the
 *      LED_GREEN pin, or just time the blink period with a stopwatch over
 *      many cycles for a rough number).
 *   3. new_value = LPTIM_LSI_FREQ_HZ * (requested_ms / measured_ms)
 *   4. Update the constant, reflash, remeasure -- repeat once more if the
 *      new measurement is still off by more than your tolerance.
 * A rigorous fix would measure LSI at runtime against a known reference
 * clock instead of trusting a fixed constant recalibrated by hand. */
#define LPTIM_LSI_FREQ_HZ 42350u

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
  CONFIG_MODE_WARMUP,
  SEND_YUV_FRAME,
  RECEIVE_PIPES_CONFIG,
  DETECT_MODE_WARMUP,
  SD_CARD,
  OP_WINDOW_CHECK,
  MOVEMENT_DETECTION,
  RECORD_MODE_INIT,
  VIDEO_RECORDING
} state_t;

/* --- Handles communs (definis ailleurs) --- */
extern UART_HandleTypeDef huart1;              /* main.c */
extern DCMIPP_HandleTypeDef hcamera_dcmipp;    /* app_cam.c */
extern volatile uint32_t dcmipp_err_count;     /* app_cam.c */

void SystemClock_Config(void);                 /* main.c */

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
extern volatile int restart_requested;  /* set by BSP_PB_Callback (ISR), consumed by app_run() */

extern volatile int h264_streaming;
extern volatile int h264_frame_ready;
extern volatile int force_intra;
extern uint8_t * volatile h264_ready_buf;
extern uint32_t actual_ticks;

/* --- Allocateur AXISRAM (defini dans app.c) --- */
void *axisram_alloc(uint32_t size);
void  axisram_reset(void);

#endif /* APP_SHARED_H */
