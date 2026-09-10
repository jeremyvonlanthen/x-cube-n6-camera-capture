/**
 ******************************************************************************
 * @file    app_watchdog.c
 * @brief   Independent watchdog (IWDG): resets the board if the FSM and the
 *          SD writer task both stop making progress.
 *
 * Nothing else in this project can recover from a hang on its own: a failed
 * HAL_RCC_OscConfig()/HAL_RCC_ClockConfig()/HAL_UART_Init() traps in
 * while(1) (main.c), and any blocking call that never returns (e.g. the
 * unbounded UART transmit send_img_uart() used to have) freezes app_run()'s
 * FSM permanently -- observed in the field as acquisitions/recording
 * stopping for good after unplugging the debug/USB link mid-run, with no
 * recovery short of a power cycle. IWDG turns any such stall into an
 * automatic reboot instead.
 ******************************************************************************
 */
#include "app_watchdog.h"
#include "app_shared.h"

#include "stm32n6xx_hal.h"

/* Comfortably above every bounded blocking call in the FSM (longest is the
 * VIDEO_DURATION_S H264 capture loop and the MP4/SD flush, both of which
 * call watchdog_kick() internally -- see app_record.c/app_rec.c) so only a
 * genuine stall (nothing left calling watchdog_kick()) trips it. */
#define WATCHDOG_TIMEOUT_MS   30000u

static IWDG_HandleTypeDef hiwdg;

void watchdog_init(void)
{
#if DEBUG_MODE
  /* Freeze the downcounter while halted at a breakpoint, so an interactive
   * debug session doesn't get reset out from under you. */
  __HAL_DBGMCU_FREEZE_IWDG();
#endif

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_512;

  /* LSI is the same uncalibrated RC oscillator used for LPTIM1 (app_sleep.c)
   * -- reuse its empirically-measured frequency (LPTIM_LSI_FREQ_HZ,
   * app_shared.h) rather than the nominal 32 kHz for a timeout close to the
   * one actually configured. */
  {
    uint32_t reload = (uint32_t)(((uint64_t)WATCHDOG_TIMEOUT_MS * LPTIM_LSI_FREQ_HZ)
                                  / (1000ull * 512u));
    hiwdg.Init.Reload = (reload > 0x0FFFu) ? 0x0FFFu : reload;
  }

  hiwdg.Init.Window = IWDG_WINDOW_DISABLE; /* refresh allowed at any time */
  hiwdg.Init.EWI    = IWDG_EWI_DISABLE;

  HAL_IWDG_Init(&hiwdg);
}

void watchdog_kick(void)
{
  HAL_IWDG_Refresh(&hiwdg);
}
