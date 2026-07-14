 /**
 ******************************************************************************
 * @file    app.h
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef APP_H
#define APP_H

#define USE_DCACHE

/* Application entry point (called from main.c once FreeRTOS is running).
 * Currently empty: the application state machine will be implemented here. */
void app_run(void);

/* --- Building blocks for the state machine (implemented in app.c) --- */

/* One-time init: LEDs, TAMP button, SD card + FAT32 + SD writer task */
void app_recorder_init(void);

/* Config mode: full-res YUV422 captures served to the Python GUI over UART,
 * then reception of the DCMIPP pipes configuration.  TAMP = record to SD. */
void app_mode_config_run(void);

/* Detect mode: greyscale dual-pipe captures (crop/decimation/downsize from
 * the received configuration) served over UART.  TAMP = record to SD. */
void app_mode_detect_run(void);

#endif