/**
 ******************************************************************************
 * @file    app_rtc.h
 * @brief   RTC runtime helpers (timestamped file names).
 *
 * Backed by an external DS3231 I2C RTC module (battery-backed, keeps time
 * across power loss), driven over I2C1 (shared with the camera sensor) --
 * see rtc_init() in app_rtc.c.
 ******************************************************************************
 */
#ifndef APP_RTC_H
#define APP_RTC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Probes the external RTC module on I2C1 and brings the bus up. Non-fatal on
 * failure: rtc_ready stays false and rtc_make_timestamp falls back to a
 * HAL_GetTick-based name. Call once at startup, before rtc_set_datetime /
 * rtc_make_timestamp. */
void rtc_init(void);

/* Sets the RTC calendar from a 6-byte payload (year-2000, month, day, hour,
 * minute, second) received from the GUI. */
void rtc_set_datetime(const uint8_t dt[6]);

/* Writes a file-name-safe timestamp "AAAA-MM-JJ_HH-MM-SS" into buf (>= 20 B).
 * Falls back to a tick-based name if the RTC is not ready. */
void rtc_make_timestamp(char *buf, size_t n);

/* True once the calendar is known-valid (OSF was clear at rtc_init(), or
 * rtc_set_datetime() has since been called). False means the calendar was
 * lost (dead/missing backup battery, or first power-up). */
bool check_rtc_validity(void);

#endif /* APP_RTC_H */
