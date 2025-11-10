 /**
 ******************************************************************************
 * @file    app_jpg.c
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

#include "app_jpg.h"

#include <assert.h>
#include <stdio.h>

#include "app_config.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_def.h"
#include "app_cvt.h"
#include "utils.h"

typedef struct {
  JPEG_HandleTypeDef hjpeg;
  JPG_conf_t conf;
  int jpg_encode_len;
} jpg_ctx_t;

static jpg_ctx_t jpg_ctx;
static uint8_t mcu_buffer[MAX_IMG_FRAME_SIZE] ALIGN_32 IN_PSRAM;


/* Public API */
int JPG_Init(JPG_conf_t *conf)
{
  JPEG_ConfTypeDef jpeg_conf = { 0 };
  jpg_ctx_t *ctx = &jpg_ctx;
  int ret;

  CVT_FormatInit();

  ctx->hjpeg.Instance = JPEG;
  ret = HAL_JPEG_Init(&ctx->hjpeg);
  if (ret)
    return ret;

  jpeg_conf.ColorSpace        = JPEG_YCBCR_COLORSPACE;
  jpeg_conf.ChromaSubsampling = JPEG_422_SUBSAMPLING;
  jpeg_conf.ImageWidth        = conf->width;
  jpeg_conf.ImageHeight       = conf->height;
  jpeg_conf.ImageQuality      = 90;
  ret = HAL_JPEG_ConfigEncoding(&ctx->hjpeg, &jpeg_conf);
  if (ret)
    return ret;

  ctx->conf = *conf;

  return 0;
}

void JPG_Deinit()
{
  jpg_ctx_t *ctx = &jpg_ctx;
  int ret;

  ret = HAL_JPEG_DeInit(&ctx->hjpeg);
  assert(ret == HAL_OK);
}

int JPG_Encode(uint8_t *p_dst, uint8_t *p_src, int dst_size, int src_size)
{
  jpg_ctx_t *ctx = &jpg_ctx;
  int ret;

  switch (ctx->conf.fmt_src) {
  case JPG_SRC_YUV422:
    CVT_FormatYuv422ToYuv422Jpeg(mcu_buffer, p_src, ctx->conf.width, ctx->conf.height);
    break;
  case JPG_SRC_RGB888:
    CVT_FormatRgb888ToYuv422Jpeg(mcu_buffer, p_src, ctx->conf.width, ctx->conf.height);
    break;
  default:
    assert(0);
  }

  ret = HAL_JPEG_Encode(&ctx->hjpeg, mcu_buffer, sizeof(mcu_buffer), p_dst, dst_size, 1000);
  assert(ret == HAL_OK);
  if (ret)
    return -1;

  return ctx->jpg_encode_len;
}

void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{
  jpg_ctx_t *ctx = &jpg_ctx;

  /* Uncomment below if you want to avoid dummy 0 bytes due to 4 bytes output fifo */
  /*while (pDataOut[OutDataLength - 1] == 0)
    OutDataLength--;*/

  ctx->jpg_encode_len = OutDataLength;
}

void HAL_JPEG_MspInit(JPEG_HandleTypeDef *hjpeg)
{
  __HAL_RCC_JPEG_CLK_ENABLE();
}

void HAL_JPEG_MspDeInit(JPEG_HandleTypeDef *hjpeg)
{
  __HAL_RCC_JPEG_CLK_DISABLE();
}
