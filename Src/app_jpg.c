#include "app_jpg.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_def.h"
#include "app_cvt.h"
#include "utils.h"

#define MCU_ROW_MCU_NB      ((2592 + 15) / 16)
#define MCU_ROW_BYTES_YUV   (MCU_ROW_MCU_NB * 256)
#define MCU_ROW_BYTES_GREY  (((2592 + 7) / 8) * 64)
#define MCU_ROW_BYTES       MCU_ROW_BYTES_YUV  // allouer le max
/* Macro */
#define CACHE_ALIGN_SIZE(s)  (((s) + 31) & ~31)

typedef struct {
  JPEG_HandleTypeDef hjpeg;
  JPG_conf_t conf;
  int jpg_encode_len;
  uint8_t *p_src;
  int current_row;
  int total_rows;
  int src_pitch;
} jpg_ctx_t;

static jpg_ctx_t jpg_ctx;
static uint8_t mcu_row_buffer[MCU_ROW_BYTES] ALIGN_32;

int JPG_Init(JPG_conf_t *conf)
{
  JPEG_ConfTypeDef jpeg_conf = { 0 };
  jpg_ctx_t *ctx = &jpg_ctx;
  int ret;

  CVT_FormatInit();

  ctx->hjpeg.Instance = JPEG;
  ret = HAL_JPEG_Init(&ctx->hjpeg);
  if (ret) return ret;

  if (conf->fmt_src == JPG_SRC_GREY) {
      jpeg_conf.ColorSpace        = JPEG_GRAYSCALE_COLORSPACE;
      jpeg_conf.ChromaSubsampling = JPEG_444_SUBSAMPLING;
  } else {
      jpeg_conf.ColorSpace        = JPEG_YCBCR_COLORSPACE;
      jpeg_conf.ChromaSubsampling = JPEG_422_SUBSAMPLING;
  }

  jpeg_conf.ImageWidth        = conf->width;
  jpeg_conf.ImageHeight       = conf->height;
  jpeg_conf.ImageQuality      = 75;
  ret = HAL_JPEG_ConfigEncoding(&ctx->hjpeg, &jpeg_conf);
  if (ret) return ret;

  ctx->conf = *conf;
  return 0;
}

void JPG_Deinit()
{
  jpg_ctx_t *ctx = &jpg_ctx;
  int ret = HAL_JPEG_DeInit(&ctx->hjpeg);
  assert(ret == HAL_OK);
}

int JPG_Encode(uint8_t *p_dst, uint8_t *p_src, int dst_size, int src_size)
{
  jpg_ctx_t *ctx = &jpg_ctx;
  int ret;

  ctx->p_src       = p_src;
  ctx->current_row = 0;
  ctx->total_rows  = (ctx->conf.height + 7) / 8;
  ctx->jpg_encode_len = 0;

  // Première ligne MCU
  if (ctx->conf.fmt_src == JPG_SRC_GREY) {
      uint32_t real_mcu_bytes = ((ctx->conf.width + 7) / 8) * 64;
      CVT_FormatGreyToGreyJpeg_Row(mcu_row_buffer, p_src,
                                    ctx->conf.width, ctx->conf.height,
                                    0, MIN(8, ctx->conf.height), ctx->conf.full_width);
      ret = HAL_JPEG_Encode_IT(&ctx->hjpeg,
                                mcu_row_buffer, real_mcu_bytes,  // ← taille réelle
                                p_dst, dst_size);
  } else {
      uint32_t real_mcu_bytes = ((ctx->conf.width + 15) / 16) * 256;
      CVT_FormatYuv422ToYuv422Jpeg_Row(mcu_row_buffer, p_src,
                                        ctx->conf.width, ctx->conf.height,
                                        0, MIN(8, ctx->conf.height));
      ret = HAL_JPEG_Encode_IT(&ctx->hjpeg,
                                mcu_row_buffer, real_mcu_bytes,  // ← taille réelle par largeur
                                p_dst, dst_size);
  }

  if (ret != HAL_OK) return -1;

  uint32_t tickstart = HAL_GetTick();
  while (ctx->hjpeg.State != HAL_JPEG_STATE_READY) {
    if (HAL_GetTick() - tickstart > 10000)
      return -1;
  }

  return ctx->jpg_encode_len;
}

void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbDecodedData)
{
  jpg_ctx_t *ctx = &jpg_ctx;

  ctx->current_row++;

  if (ctx->current_row >= ctx->total_rows) {
    HAL_JPEG_ConfigInputBuffer(hjpeg, mcu_row_buffer, 0);
    return;
  }

  int remain_height = ctx->conf.height - ctx->current_row * 8;
  if (remain_height > 8) remain_height = 8;

  if (ctx->conf.fmt_src == JPG_SRC_GREY) {
      uint32_t real_mcu_bytes = ((ctx->conf.width + 7) / 8) * 64;
      CVT_FormatGreyToGreyJpeg_Row(mcu_row_buffer, ctx->p_src,
                                    ctx->conf.width, ctx->conf.height,
                                    ctx->current_row, remain_height, ctx->conf.full_width);
      HAL_JPEG_ConfigInputBuffer(hjpeg, mcu_row_buffer, real_mcu_bytes);  // ← taille réelle
  } else {
      uint32_t real_mcu_bytes = ((ctx->conf.width + 15) / 16) * 256;
      CVT_FormatYuv422ToYuv422Jpeg_Row(mcu_row_buffer, ctx->p_src,
                                        ctx->conf.width, ctx->conf.height,
                                        ctx->current_row, remain_height);
      HAL_JPEG_ConfigInputBuffer(hjpeg, mcu_row_buffer, real_mcu_bytes);
  }
}

void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg,
                                 uint8_t *pDataOut, uint32_t OutDataLength)
{
  jpg_ctx_t *ctx = &jpg_ctx;
  ctx->jpg_encode_len = OutDataLength;
}

void HAL_JPEG_MspInit(JPEG_HandleTypeDef *hjpeg)
{
  __HAL_RCC_JPEG_CLK_ENABLE();
  HAL_NVIC_SetPriority(JPEG_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(JPEG_IRQn);
}

void HAL_JPEG_MspDeInit(JPEG_HandleTypeDef *hjpeg)
{
  HAL_NVIC_DisableIRQ(JPEG_IRQn);
  __HAL_RCC_JPEG_CLK_DISABLE();
}

void JPG_IRQHandler(void)
{
  HAL_JPEG_IRQHandler(&jpg_ctx.hjpeg);
}
