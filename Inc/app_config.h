/**
 ******************************************************************************
 * @file    app_config.h
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#ifndef APP_CONFIG
#define APP_CONFIG

/* Full-resolution capture buffer: one 2592x1944 frame, 2 bytes/pixel
 * (YUV422 in config mode; also holds the 2 RGB565 720p frames + the
 * encoded-video ring buffer during H264 recording — see app.c) */
#define MAX_CAPTURE_FRAME_SIZE (2592 * 1944 * 2)

/* Hardware JPEG encoder output buffer */
#define MAX_JPEG_FRAME_SIZE    (1 * 1024 * 1024)

#endif
