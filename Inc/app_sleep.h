/**
 ******************************************************************************
 * @file    app_sleep.h
 * @brief   Low-power helpers for the DIAS state machine's idle windows.
 ******************************************************************************
 */
#ifndef APP_SLEEP_H
#define APP_SLEEP_H

#include <stdint.h>

/* One-time init: must be called once at boot, after the LSI oscillator is
 * already enabled (RTC_Config() in main.c does this) */
void app_sleep_init(void);
void sleep_short_period(uint32_t time_ms);

/* Wired to LPTIM1_IRQHandler() in stm32n6xx_it.c -- do not call directly. */
void APP_SLEEP_LPTIM_IRQHandler(void);

#endif /* APP_SLEEP_H */
