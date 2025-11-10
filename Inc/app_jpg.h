 /**
 ******************************************************************************
 * @file    app_jpg.h
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

#ifndef APP_JPG_H
#define APP_JPG_H

#include "stdint.h"

#define JPG_SRC_YUV422 0
#define JPG_SRC_RGB888 1

typedef struct {
  int width;
  int height;
  int fmt_src;
} JPG_conf_t;

int JPG_Init(JPG_conf_t *conf);
void JPG_Deinit(void);
/* return encoded bytes or negative in case of error */
int JPG_Encode(uint8_t *p_dst, uint8_t *p_src, int dst_size, int src_size);

#endif