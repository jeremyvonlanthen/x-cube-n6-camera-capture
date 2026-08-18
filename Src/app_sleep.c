/**
 ******************************************************************************
 * @file    app_sleep.c
 * @brief   Low-power helpers for the DIAS state machine's idle windows.
 *          See app_sleep.h for the rationale (STOP mode is broken on this
 *          board, see ST_ticket_stop_mode.txt).
 ******************************************************************************
 */
#include "app_sleep.h"
#include "app_shared.h"

#include <stdio.h>

#include "stm32n6xx_hal.h"

/* LPTIM1 wakes the CPU from SLEEP mode; owned entirely by this module. */
static LPTIM_HandleTypeDef hlptim1;

void app_sleep_init(void)
{
  RCC_PeriphCLKInitTypeDef pclk = {0};

  pclk.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
  pclk.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) {
    printf("[app_sleep] LPTIM1 clock select failed\r\n");
    return;
  }

  __HAL_RCC_LPTIM1_CLK_ENABLE();

  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source      = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler   = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source    = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.UpdateMode        = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource     = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Period            = 0xFFFF;
  hlptim1.Init.RepetitionCounter = 0;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK) {
    printf("[app_sleep] LPTIM1 init failed\r\n");
    return;
  }

  HAL_NVIC_SetPriority(LPTIM1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
}

void APP_SLEEP_LPTIM_IRQHandler(void)
{
  HAL_LPTIM_IRQHandler(&hlptim1);
}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  HAL_LPTIM_Counter_Stop_IT(hlptim);
}

void sleep_short_period(uint32_t time_ms)
{
  if (time_ms <= 0)
    return;

  /* LSI is uncalibrated: use the empirically-measured frequency, not the
   * nominal 32kHz, or the sleep duration drifts. See LPTIM_LSI_FREQ_HZ in
   * app_shared.h for how to recalibrate. */
  static const uint32_t prescalers[] = {
    LPTIM_PRESCALER_DIV1,  LPTIM_PRESCALER_DIV2,  LPTIM_PRESCALER_DIV4,
    LPTIM_PRESCALER_DIV8,  LPTIM_PRESCALER_DIV16, LPTIM_PRESCALER_DIV32,
    LPTIM_PRESCALER_DIV64, LPTIM_PRESCALER_DIV128,
  };
  /* LPTIM1's auto-reload is 16-bit: at DIV1 that caps a single sleep at
   * ~1.5s (0xFFFF * 1000 / LPTIM_LSI_FREQ_HZ). Rather than clamp (and
   * silently sleep less than asked), widen the LPTIM1 prescaler until the
   * request fits -- each step halves resolution but doubles reach, up to
   * ~198s at DIV128. Still hard-clamped at DIV128 for anything longer than
   * that (shouldn't happen for this FSM's ~1-2s waits). */
  uint32_t divider = 1;
  uint32_t presc_idx = 0;
  uint32_t period_ticks;
  do {
    period_ticks = (uint32_t)(((uint64_t)time_ms * LPTIM_LSI_FREQ_HZ) / (1000ull * divider));
    if (period_ticks <= 0xFFFF || presc_idx == 7)
      break;
    divider <<= 1;
    presc_idx++;
  } while (1);
  if (period_ticks > 0xFFFF) period_ticks = 0xFFFF;

  hlptim1.Init.Clock.Prescaler = prescalers[presc_idx];
  hlptim1.Init.Period = period_ticks;
  HAL_LPTIM_Init(&hlptim1);
  HAL_LPTIM_Counter_Start_IT(&hlptim1);

  /* Still in Run mode / tick running here: safe to use HAL_Delay indirectly
   * (HAL_RCC_OscConfig polls PLL flags, not the tick). */
  RCC_ClkInitTypeDef clk_lp = {0};
  RCC_OscInitTypeDef osc_lp = {0};

  /* MSI@4MHz instead of HSI (~64MHz): lowest-frequency oscillator this
   * family offers for CPUCLK/SYSCLK. Must be turned on and stable BEFORE
   * it's selected as a clock source below. */
  osc_lp.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  osc_lp.MSIState       = RCC_MSI_ON;
  osc_lp.MSIFrequency   = RCC_MSI_FREQ_4MHZ;
  HAL_RCC_OscConfig(&osc_lp);

  clk_lp.ClockType    = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK;
  clk_lp.CPUCLKSource = RCC_CPUCLKSOURCE_MSI;
  clk_lp.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  HAL_RCC_ClockConfig(&clk_lp);

  /* Now safe to switch PLL1..4 off (no longer selected as source). */
  osc_lp.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  osc_lp.PLL1.PLLState = RCC_PLL_OFF;
  osc_lp.PLL2.PLLState = RCC_PLL_OFF;
  osc_lp.PLL3.PLLState = RCC_PLL_OFF;
  osc_lp.PLL4.PLLState = RCC_PLL_OFF;
  HAL_RCC_OscConfig(&osc_lp);

  /* main.c's boot policy keeps every peripheral bus clock alive during
   * CSLEEP ("keep all IPs enabled to wake up the CPU" --
   * LL_xxx_EnableClockLowPower(~0)). Narrow this down to just LPTIM1 + PWR
   * for the sleep window (AHB4 also carries PWR's own bit -- gating it
   * broke the wake-up when this was first tried). SLEEP wakes via plain
   * NVIC, not the PWR/EXTI deep-sleep circuit, so narrowing here is safe
   * (unlike for STOP mode, where the same narrowing broke the wake-up). */
  LL_BUS_DisableClockLowPower(~0);
  LL_MEM_DisableClockLowPower(~0);
  LL_AHB1_GRP1_DisableClockLowPower(~0);
  LL_AHB2_GRP1_DisableClockLowPower(~0);
  LL_AHB3_GRP1_DisableClockLowPower(~0);
  LL_AHB4_GRP1_DisableClockLowPower(~0);
  LL_AHB4_GRP1_EnableClockLowPower(LL_AHB4_GRP1_PERIPH_PWR);
  LL_AHB5_GRP1_DisableClockLowPower(~0);
  LL_APB1_GRP1_DisableClockLowPower(~0);
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_LPTIM1);
  LL_APB1_GRP2_DisableClockLowPower(~0);
  LL_APB2_GRP1_DisableClockLowPower(~0);
  LL_APB4_GRP1_DisableClockLowPower(~0);
  LL_APB4_GRP2_DisableClockLowPower(~0);
  LL_APB5_GRP1_DisableClockLowPower(~0);
  LL_MISC_DisableClockLowPower(~0);

  HAL_SuspendTick();
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  /* Resume tick before any call that (indirectly) uses HAL_Delay()/
   * vTaskDelay() -- SystemClock_Config() below needs it. */
  HAL_ResumeTick();

  /* Restore the boot-time "keep everything alive" policy before the rest
   * of the FSM resumes normal operation. */
  LL_BUS_EnableClockLowPower(~0);
  LL_MEM_EnableClockLowPower(~0);
  LL_AHB1_GRP1_EnableClockLowPower(~0);
  LL_AHB2_GRP1_EnableClockLowPower(~0);
  LL_AHB3_GRP1_EnableClockLowPower(~0);
  LL_AHB4_GRP1_EnableClockLowPower(~0);
  LL_AHB5_GRP1_EnableClockLowPower(~0);
  LL_APB1_GRP1_EnableClockLowPower(~0);
  LL_APB1_GRP2_EnableClockLowPower(~0);
  LL_APB2_GRP1_EnableClockLowPower(~0);
  LL_APB4_GRP1_EnableClockLowPower(~0);
  LL_APB4_GRP2_EnableClockLowPower(~0);
  LL_APB5_GRP1_EnableClockLowPower(~0);
  LL_MISC_EnableClockLowPower(~0);

  SystemClock_Config(); /* PLLs back ON, full speed restored */

  /* MSI is no longer selected as CPUCLK/SYSCLK source at this point
   * (SystemClock_Config moved it to the PLL/IC tree) -- turn it back off
   * so it isn't left running uselessly until the next sleep window. */
  RCC_OscInitTypeDef osc_msi_off = {0};
  osc_msi_off.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  osc_msi_off.MSIState       = RCC_MSI_OFF;
  HAL_RCC_OscConfig(&osc_msi_off);
}
