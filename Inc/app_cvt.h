 /**
 ******************************************************************************
 * @file    app_cvt.h
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

#ifndef CVT_FORMAT
#define CVT_FORMAT 1

#include <stdint.h>

void CVT_FormatInit(void);
void CVT_FormatGreyToYuv422(uint8_t *p_dst, uint8_t *p_src, int width, int height);
void CVT_FormatArgbToYuv422(uint8_t *p_dst, uint8_t *p_src, int width, int height);
void CVT_FormatRgb565ToYuv422(uint8_t *p_dst, uint8_t *p_src, int width, int height);
void CVT_FormatGreyToYuv422Jpeg(uint8_t *p_dst, uint8_t *p_src, int width, int height);
void CVT_FormatRgbArgbToYuv422Jpeg(uint8_t *p_dst, uint8_t *p_src, int width, int height);
void CVT_FormatRgb888ToYuv422Jpeg(uint8_t *p_dst, uint8_t *p_src, int width, int height);
void CVT_FormatRgb565ToYuv422Jpeg(uint8_t *p_dst, uint8_t *p_src, int width, int height);
void CVT_FormatYuv422ToYuv422Jpeg(uint8_t *p_dst, uint8_t *p_src, int width, int height);

#endif
