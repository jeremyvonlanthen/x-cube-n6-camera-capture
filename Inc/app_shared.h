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
 *      e.g. requested 900ms, measured 788ms -> 42350 * (900/788) ~= 48350
 *      (the value below was already produced by one such measurement).
 *   4. Update the constant, reflash, remeasure -- repeat once more if the
 *      new measurement is still off by more than your tolerance.
 * A rigorous fix would measure LSI at runtime against a known reference
 * clock instead of trusting a fixed constant recalibrated by hand. */
#define LPTIM_LSI_FREQ_HZ 42350u

/* MOVEMENT_DETECTION idle-wait strategy under test (see app.c):
 *   1 = reference busy-wait (HAL_Delay, Run mode, no low power at all)
 *   2 = STOP mode (CSTOP: CPU + bus clocks + PLL1..4 gated, SRAM/state kept)
 *       ON HOLD: HAL_PWR_EnterSTOPMode() never wakes on this board with LPTIM1
 *       (confirmed with SVOS3/5, WFI/WFE, clean power cycle -- see
 *       ST_ticket_stop_mode.txt). Kept in the code for reference/ticket
 *       reproduction only; don't use for real measurements right now.
 *   3 = SLEEP mode + PLL1..4 manually switched off beforehand (CPUCLK/SYSCLK
 *       moved to MSI@4MHz first) + unused-peripheral clock-gating, restored
 *       via SystemClock_Config() on wake. The reliable, working strategy
 *       (~1.4 mA measured) -- practical replacement for 2 since STOP is
 *       broken on this board.
 * (The former "explicit SLEEP mode, no PLL shutdown" strategy 2 was removed:
 * it's electrically identical to strategy 1, since HAL_Delay() is remapped
 * to vTaskDelay() and FreeRTOS's tickless idle already does a single WFI for
 * the whole delay -- there was nothing left to compare.) */
#define SLEEP_STRATEGY 1

/* Strategy 2 only: narrow the "peripheral clock kept alive in low-power
 * mode" bits (set wide-open at boot in main.c) down to just LPTIM1 for the
 * duration of the STOP sleep window. DEFAULT OFF: the AHB4 group we blanket
 * -disable also carries PWR's own low-power clock-keep bit (per the HAL
 * header), and PWR is what drives the wake-up sequencing -- gating it may be
 * what causes the SWD/debugger lockups seen when this was enabled. Only
 * flip to 1 to deliberately re-test this hypothesis, ideally without a
 * debugger attached. */
#define STOP_MODE_NARROW_CLOCKS 0

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
