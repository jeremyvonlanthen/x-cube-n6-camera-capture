/**
 ******************************************************************************
 * @file    app_rtc.c
 * @brief   RTC runtime helpers (timestamped file names).
 ******************************************************************************
 */
#include "app_rtc.h"

#include <stdio.h>

#include "stm32n6xx_hal.h"

/* Defined in main.c (RTC_Config) */
extern RTC_HandleTypeDef hrtc;
extern volatile int rtc_ready;

void rtc_set_datetime(const uint8_t dt[6])
{
  RTC_TimeTypeDef t = { 0 };
  RTC_DateTypeDef d = { 0 };

  if (!rtc_ready)
    return;

  d.Year    = dt[0];
  d.Month   = dt[1];
  d.Date    = dt[2];
  d.WeekDay = RTC_WEEKDAY_MONDAY;
  t.Hours   = dt[3];
  t.Minutes = dt[4];
  t.Seconds = dt[5];

  HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN);
  HAL_RTC_SetDate(&hrtc, &d, RTC_FORMAT_BIN);
  printf("[RTC] date and time set to 20%02u-%02u-%02u %02u:%02u:%02u\r\n",
         dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]);
}

void rtc_make_timestamp(char *buf, size_t n)
{
  RTC_TimeTypeDef t = { 0 };
  RTC_DateTypeDef d = { 0 };

  if (!rtc_ready) {
    snprintf(buf, n, "REC_%08lu", (unsigned long)HAL_GetTick());
    return;
  }

  HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);

  snprintf(buf, n, "%04u-%02u-%02u_%02u-%02u-%02u",
           2000 + d.Year, d.Month, d.Date, t.Hours, t.Minutes, t.Seconds);
}
