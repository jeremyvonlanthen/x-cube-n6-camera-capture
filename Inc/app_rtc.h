/**
 ******************************************************************************
 * @file    app_rtc.h
 * @brief   RTC runtime helpers (timestamped file names).
 *
 * The RTC peripheral itself is configured in main.c (RTC_Config); the handle
 * `hrtc` and the `rtc_ready` flag are defined there.  These helpers only read
 * and set the calendar.
 ******************************************************************************
 */
#ifndef APP_RTC_H
#define APP_RTC_H

#include <stddef.h>
#include <stdint.h>

/* Sets the RTC calendar from a 6-byte payload (year-2000, month, day, hour,
 * minute, second) received from the GUI. */
void rtc_set_datetime(const uint8_t dt[6]);

/* Writes a file-name-safe timestamp "AAAA-MM-JJ_HH-MM-SS" into buf (>= 20 B).
 * Falls back to a tick-based name if the RTC is not ready. */
void rtc_make_timestamp(char *buf, size_t n);

#endif /* APP_RTC_H */
