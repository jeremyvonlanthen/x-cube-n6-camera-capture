/**
 ******************************************************************************
 * @file    app_rtc.c
 * @brief   RTC runtime helpers (timestamped file names).
 *
 * Driven by an external DS3231 RTC module sharing I2C1 with the camera
 * sensor (SCL = PH9, SDA = PC1 on the STM32N6570-DK). I2C1 is broken out,
 * unlike I2C2, on the Arduino-compatible header CN12 as pins D15 (SCL) /
 * D14 (SDA) -- no soldering to the MCU needed. I2C2 (PD14/PD4) was NOT used
 * because it is already wired on-board to the touch panel controller and
 * the USB-C PD chip, with no accessible header exposing it.
 *
 * No address collision with the camera sensors sharing the bus: DS3231 is
 * 0x68 (7-bit), none of the sensor addresses in cmw_io.h (0x20/0x34/0x78,
 * 8-bit) match it. BSP_I2C1_Init() is reference-counted (I2c1InitCounter in
 * stm32n6570_discovery_bus.c), so calling it here and again from the camera
 * driver is safe.
 *
 * BCD calendar registers 0x00-0x06, status register 0x0F with an
 * Oscillator Stop Flag (OSF) that latches if the backup battery ever let
 * the calendar die.
 *
 * NUCLEO-N657X0-Q note: that board's BSP only wraps I2C2 (see
 * stm32n6xx_nucleo_bus.h), not I2C1, so this driver won't even compile
 * there as-is. Not addressed here since the project's default target is
 * the STM32N6570-DK.
 ******************************************************************************
 */
#include "app_rtc.h"

#include <stdbool.h>
#include <stdio.h>

#include "stm32n6xx_hal.h"
#ifdef STM32N6570_DK_REV
#include "stm32n6570_discovery_bus.h"
#else
#include "stm32n6xx_nucleo_bus.h"
#endif

/* 7-bit I2C address 0x68, shifted for the BSP's 8-bit DevAddr convention
 * (same convention as the camera sensor addresses in cmw_io.h). */
#define DS3231_I2C_ADDR       (0x68U << 1)

#define DS3231_REG_SECONDS    0x00U
#define DS3231_REG_STATUS     0x0FU
#define DS3231_STATUS_OSF     0x80U /* Oscillator Stop Flag: calendar was lost */

static volatile bool rtc_ready = false;
/* False from boot until either the OSF check at init found the calendar
 * already valid, or rtc_set_datetime() has written a fresh time. Gates
 * rtc_make_timestamp()'s fallback -- without this, a recording triggered
 * before the GUI sends 'T' (calendar lost: dead battery, first power-up)
 * would silently timestamp itself from the stale/garbage registers instead
 * of falling back to the tick-based name. */
static volatile bool rtc_time_valid = false;

static uint8_t bin2bcd(uint8_t v)
{
  return (uint8_t)(((v / 10U) << 4) | (v % 10U));
}

static uint8_t bcd2bin(uint8_t v)
{
  return (uint8_t)(((v >> 4) * 10U) + (v & 0x0FU));
}

void rtc_init(void)
{
  uint8_t status;

  if (BSP_I2C1_Init() != BSP_ERROR_NONE) {
    printf("[RTC] I2C1 init failed\r\n");
    return;
  }

  if (BSP_I2C1_IsReady(DS3231_I2C_ADDR, 3) != BSP_ERROR_NONE) {
    printf("[RTC] DS3231 not responding on I2C1\r\n");
    return;
  }

  rtc_ready = true;

  if (BSP_I2C1_ReadReg(DS3231_I2C_ADDR, DS3231_REG_STATUS, &status, 1) == BSP_ERROR_NONE
      && !(status & DS3231_STATUS_OSF)) {
    rtc_time_valid = true;
  }

  printf("[RTC] ready: %s/ time valid: %s\r\n", rtc_ready ? "true":"false", rtc_time_valid ? "true":"false");
}

bool check_rtc_validity(void)
{
  return rtc_time_valid;
}

void rtc_set_datetime(const uint8_t dt[6])
{
  uint8_t reg[7];
  uint8_t status;

  if (!rtc_ready)
    return;

  reg[0] = bin2bcd(dt[5]);  /* seconds */
  reg[1] = bin2bcd(dt[4]);  /* minutes */
  reg[2] = bin2bcd(dt[3]);  /* hours, 24h mode (bit 6 = 0) */
  reg[3] = bin2bcd(1);      /* day-of-week: unused, kept valid (1-7) */
  reg[4] = bin2bcd(dt[2]);  /* date */
  reg[5] = bin2bcd(dt[1]);  /* month, century bit left at 0 */
  reg[6] = bin2bcd(dt[0]);  /* year - 2000 */

  if (BSP_I2C1_WriteReg(DS3231_I2C_ADDR, DS3231_REG_SECONDS, reg, sizeof(reg)) != BSP_ERROR_NONE) {
    printf("[RTC] failed to write date/time\r\n");
    return;
  }

  /* Clear OSF: the calendar is now known-valid until the next power loss
   * without a working backup battery. */
  if (BSP_I2C1_ReadReg(DS3231_I2C_ADDR, DS3231_REG_STATUS, &status, 1) == BSP_ERROR_NONE) {
    status &= (uint8_t)~DS3231_STATUS_OSF;
    BSP_I2C1_WriteReg(DS3231_I2C_ADDR, DS3231_REG_STATUS, &status, 1);
  }
  rtc_time_valid = true;

  printf("[RTC] date and time set to 20%02u-%02u-%02u %02u:%02u:%02u\r\n",
         dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]);
}

void rtc_make_timestamp(char *buf, size_t n)
{
  uint8_t reg[7];

  if (!rtc_ready || !rtc_time_valid
      || BSP_I2C1_ReadReg(DS3231_I2C_ADDR, DS3231_REG_SECONDS, reg, sizeof(reg)) != BSP_ERROR_NONE) {
    snprintf(buf, n, "REC_%08lu", (unsigned long)HAL_GetTick());
    return;
  }

  uint8_t sec   = bcd2bin(reg[0] & 0x7FU);
  uint8_t min   = bcd2bin(reg[1] & 0x7FU);
  uint8_t hour  = bcd2bin(reg[2] & 0x3FU); /* 24h mode */
  uint8_t date  = bcd2bin(reg[4] & 0x3FU);
  uint8_t month = bcd2bin(reg[5] & 0x1FU);
  uint8_t year  = bcd2bin(reg[6]);

  snprintf(buf, n, "%04u-%02u-%02u_%02u-%02u-%02u",
           2000 + year, month, date, hour, min, sec);
}
