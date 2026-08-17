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
#define SENSOR_WARMUP_FPS     5
#define WARMUP_FRAMES_TARGET  10      /* frames skipped so the AE/ISP converge */

/* MOVEMENT_DETECTION idle-wait strategy under test (see app.c):
 *   1 = reference busy-wait (HAL_Delay, Run mode, no low power at all)
 *   2 = SLEEP mode   (CSLEEP: only the Cortex-M55 core clock is gated)
 *   3 = STOP mode    (CSTOP: CPU + bus clocks + PLL1..4 gated, SRAM/state kept)
 *       ON HOLD: HAL_PWR_EnterSTOPMode() never wakes on this board with LPTIM1
 *       (confirmed with SVOS3/5, WFI/WFE, clean power cycle -- see
 *       ST_ticket_stop_mode.txt). Kept in the code for reference/ticket
 *       reproduction only; don't use for real measurements right now.
 *   4 = SLEEP mode + PLL1..4 manually switched off beforehand (CPUCLK/SYSCLK
 *       moved to HSI first), restored via SystemClock_Config() on wake.
 *       Practical replacement for 3: same reliable SLEEP wake path, but
 *       attacks the PLLs directly instead of relying on STOP.
 * STANDBY is deliberately not offered here: it wipes SRAM and restarts from
 * the reset vector, which is incompatible with resuming this FSM every
 * ~1s (see comments in app.c). */
#define SLEEP_STRATEGY 4

/* Strategy 3 only: narrow the "peripheral clock kept alive in low-power
 * mode" bits (set wide-open at boot in main.c) down to just LPTIM1 for the
 * duration of the STOP sleep window. DEFAULT OFF: the AHB4 group we blanket
 * -disable also carries PWR's own low-power clock-keep bit (per the HAL
 * header), and PWR is what drives the wake-up sequencing -- gating it may be
 * what causes the SWD/debugger lockups seen when this was enabled. Only
 * flip to 1 to deliberately re-test this hypothesis, ideally without a
 * debugger attached. */
#define STOP_MODE_NARROW_CLOCKS 0

/* Keep DBGMCU clocked through SLEEP/STOP/STANDBY so ST-LINK/SWD stays
 * connected and breakpoints work while developing. This ADDS consumption
 * and defeats what strategies 2/3 are trying to save: set to 0 (and detach
 * the debugger) before taking any real current measurement. */
#define DEBUG_KEEP_SWD_ALIVE_IN_LOWPOWER 1

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
extern LPTIM_HandleTypeDef hlptim1;            /* main.c */
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

extern volatile int h264_streaming;
extern volatile int h264_frame_ready;
extern volatile int force_intra;
extern uint8_t * volatile h264_ready_buf;
extern uint32_t actual_ticks;

/* --- Allocateur AXISRAM (defini dans app.c) --- */
void *axisram_alloc(uint32_t size);
void  axisram_reset(void);

#endif /* APP_SHARED_H */
