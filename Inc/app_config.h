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

/* ifndef so you can change below without code modification */
#ifndef IMG_STREAMS
#ifdef STM32N6570_DK_REV
#define IMG_STREAMS {{UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 224, 224, 30}, \
                     {UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 256, 256, 30}, \
                     {UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 480, 480, 30}, \
                     {UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 640, 480, 30}, \
                     {UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 800, 480, 30}};
#define MAX_IMG_FRAME_SIZE (800 * 480 * 2)
#else
#define IMG_STREAMS {{UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 224, 224, 30}, \
                     {UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 256, 256, 30}, \
                     {UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 480, 480, 30}, \
                     {UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 640, 480, 30}, \
                     {UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 800, 480, 30}};
#define MAX_IMG_FRAME_SIZE (800 * 480 * 2)
#endif
#else
#ifndef MAX_IMG_FRAME_SIZE
#error MAX_IMG_FRAME_SIZE must be define
#endif
#endif

#endif
