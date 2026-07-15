 /**
 ******************************************************************************
 * @file    enc.c
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
#include "app_enc.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "h264encapi.h"
#include "jpegencapi.h"
#include "stm32n6xx_hal.h"
#include "utils.h"
#include "ewl.h"

#define VENC_ALLOCATOR_SIZE (4 * 1024 * 1024)
#define EWL_HEAP_POOL_SIZE  (64 * 1024)
#define RATE_CTRL_QP 25

enum {
  VENC_RATE_CTRL_QP_CONSTANT,
  VENC_RATE_CTRL_VBR,
};

static uint8_t venc_hw_allocator_buffer[VENC_ALLOCATOR_SIZE] ALIGN_32 IN_PSRAM;
static uint8_t *venc_hw_allocator_pos = venc_hw_allocator_buffer;

/* EWL heap bump-allocator pool — in AXISRAM3456, safe from heap/USB overlap */
static uint8_t ewl_heap_pool[EWL_HEAP_POOL_SIZE]
    __attribute__((section(".axisram_bss"), aligned(32)));
static size_t ewl_heap_used = 0;
static struct VENC_Context {
  H264EncInst hdl;
  int is_sps_pps_done;
  uint64_t pic_cnt;
  int gop_len;
} VENC_Instance;

static void VENC_SetupConstantQp(H264EncRateCtrl *rate, int qp)
{
  rate->pictureRc = 0;
  rate->mbRc = 0;
  rate->pictureSkip = 0;
  rate->hrd = 0;
  rate->qpHdr = qp;
  rate->qpMin = qp;
  rate->qpMax = qp;
}

static void VENC_SetupVbr(H264EncRateCtrl *rate, int bitrate, int gopLen, int qp)
{
  rate->pictureRc = 1;
  rate->mbRc = 1;
  rate->pictureSkip = 0;
  rate->hrd = 0;
  rate->qpHdr = qp;
  rate->qpMin = 10;
  rate->qpMax = 51;
  rate->gopLen = gopLen;
  rate->bitPerSecond = bitrate;
  rate->intraQpDelta = 0;
}

static int VENC_AppendPadding(struct VENC_Context *p_ctx, uint8_t *p_out, size_t out_len, size_t *p_out_len)
{
  uint32_t out_addr = (uint32_t) p_out;
  int pad_size = 8 - (out_addr % 8);
  int pad_len = 0;

  *p_out_len = 0;

  /* No need of padding */
  if (out_addr % 8 == 0)
    return 0;

  /* adjust pad size */
  if (pad_size < 6)
    pad_size += 8;

  /* Do we add enought space for padding ? */
  if (pad_size > out_len)
    return -1;

  /* Ok now we add nal pad */
  p_out[pad_len++] = 0x00;
  p_out[pad_len++] = 0x00;
  p_out[pad_len++] = 0x00;
  p_out[pad_len++] = 0x01;
  p_out[pad_len++] = 0x2c; /* FIXME : adapt for nal_ref_idc ? */
  pad_size -= 5;
  while (pad_size--)
    p_out[pad_len++] = 0xff;

  *p_out_len = pad_len;

  return 0;
}

static int VENC_EncodeStart(struct VENC_Context *p_ctx, uint8_t *p_out, size_t out_len, size_t *p_out_len)
{
  H264EncOut enc_out;
  H264EncIn enc_in;
  size_t start_len;
  size_t pad_len;
  int ret;

  enc_in.pOutBuf = (u32 *) p_out;
  enc_in.busOutBuf = (ptr_t) p_out;
  enc_in.outBufSize = out_len;
  ret = H264EncStrmStart(p_ctx->hdl, &enc_in, &enc_out);
  if (ret)
    return ret;

  start_len = enc_out.streamSize;
  ret = VENC_AppendPadding(p_ctx, &p_out[start_len], out_len - start_len, &pad_len);
  if (ret)
    return ret;

  *p_out_len = start_len + pad_len;

  return 0;
}

static int VENC_EncodeFrame(struct VENC_Context *p_ctx, uint8_t *p_in, uint8_t *p_out, size_t out_len,
                            size_t *p_out_len, int is_intra_force)
{
  H264EncOut enc_out;
  H264EncIn enc_in;
  int ret;

  /* In both N6_VENC_INPUT_YUV2 and N6_VENC_INPUT_RGB565 only busLuma is used */
  enc_in.busLuma = (ptr_t) p_in;
  enc_in.busChromaU = 0;
  enc_in.busChromaV = 0;
  enc_in.pOutBuf = (u32 *) p_out;
  enc_in.busOutBuf = (ptr_t) p_out;
  enc_in.outBufSize = out_len;
  enc_in.codingType = (p_ctx->pic_cnt % (p_ctx->gop_len + 1) == 0) ? H264ENC_INTRA_FRAME : H264ENC_PREDICTED_FRAME;
  enc_in.codingType = is_intra_force ? H264ENC_INTRA_FRAME : enc_in.codingType;
  enc_in.timeIncrement = 1;
  enc_in.ipf = H264ENC_REFERENCE_AND_REFRESH; /* FIXME : can be H264ENC_NO_REFERENCE_NO_REFRESH in I only mode */
  enc_in.ltrf = H264ENC_NO_REFERENCE_NO_REFRESH;
  enc_in.lineBufWrCnt = 0;
  enc_in.sendAUD = 0;

  ret = H264EncStrmEncode(p_ctx->hdl, &enc_in, &enc_out, NULL, NULL, NULL);
  if (ret != H264ENC_FRAME_READY)
    return -1;

  p_ctx->pic_cnt++;
  *p_out_len = enc_out.streamSize;

  return 0;
}

static int VENC_Encode(uint8_t *p_in, uint8_t *p_out, size_t out_len, size_t *p_out_len, int is_intra_force)
{
  struct VENC_Context *p_ctx = &VENC_Instance;
  size_t start_len = 0;
  size_t frame_len;
  int ret;

  if (!p_ctx->is_sps_pps_done)
  {
    ret = VENC_EncodeStart(p_ctx, p_out, out_len, &start_len);
    if (ret)
      return ret;
    p_ctx->is_sps_pps_done = 1;
  }

  ret = VENC_EncodeFrame(p_ctx, p_in, &p_out[start_len], out_len - start_len, &frame_len, is_intra_force);
  if (ret)
    return ret;

  *p_out_len = start_len + frame_len;

  return 0;
}

void ENC_Init(ENC_Conf_t *p_conf)
{
  const int rate_ctrl_mode = VENC_RATE_CTRL_VBR;
  struct VENC_Context *p_ctx = &VENC_Instance;
  H264EncPreProcessingCfg cfg;
  H264EncCodingCtrl ctrl;
  H264EncConfig config;
  H264EncRateCtrl rate;
  int target_bitrate;
  int ret;

  memset(&config, 0, sizeof(config));
  p_ctx->gop_len = p_conf->fps - 1;
  /* init encoder */
  config.streamType = H264ENC_BYTE_STREAM;
  config.viewMode = H264ENC_BASE_VIEW_SINGLE_BUFFER;
  config.level = H264ENC_LEVEL_5_1;
  config.width = p_conf->width;
  config.height = p_conf->height;
  config.frameRateNum = p_conf->fps;
  config.frameRateDenom = 1;
  config.refFrameAmount = 1;
  ret = H264EncInit(&config, &p_ctx->hdl);
  assert(ret == H264ENC_OK);

  /* setup source format */
  ret = H264EncGetPreProcessing(p_ctx->hdl, &cfg);
  assert(ret == H264ENC_OK);
  /* RGB565 (2 B/px) instead of RGB888/ARGB8888 (4 B/px): halves the PSRAM
   * bandwidth of both the DCMIPP capture writes and the VENC input reads.
   * ARGB8888 caused DCMIPP pixel-packer overruns (corrupted line endings on
   * the right side of the image).  Same format as the ST VENC examples. */
  cfg.inputType = H264ENC_RGB565;
  ret = H264EncSetPreProcessing(p_ctx->hdl, &cfg);
  assert(ret == H264ENC_OK);

  /* setup coding ctrl */
  ret = H264EncGetCodingCtrl(p_ctx->hdl, &ctrl);
  assert(ret == H264ENC_OK);
  ctrl.idrHeader = 1;
  ret = H264EncSetCodingCtrl(p_ctx->hdl, &ctrl);
  assert(ret == H264ENC_OK);

  /* setup rate ctrl */
  ret = H264EncGetRateCtrl(p_ctx->hdl, &rate);
  assert(ret == H264ENC_OK);
  target_bitrate = ((p_conf->width * p_conf->height * 12) * p_conf->fps) / 30;
  if (rate_ctrl_mode == VENC_RATE_CTRL_QP_CONSTANT)
  {
    VENC_SetupConstantQp(&rate, RATE_CTRL_QP);
  } else if (rate_ctrl_mode == VENC_RATE_CTRL_VBR)
  {
    VENC_SetupVbr(&rate, target_bitrate, p_conf->fps, RATE_CTRL_QP);
  } else
  {
    assert(0);
  }
  ret = H264EncSetRateCtrl(p_ctx->hdl, &rate);
  assert(ret == H264ENC_OK);
}

void ENC_DeInit()
{
  struct VENC_Context *p_ctx = &VENC_Instance;
  int ret;

  ret = H264EncRelease(p_ctx->hdl);
  assert(ret == H264ENC_OK);
  memset(p_ctx, 0, sizeof(*p_ctx));
}

/* Reset both allocators so ENC_Init() can be called again */
void ENC_ResetAllocator(void)
{
  venc_hw_allocator_pos = venc_hw_allocator_buffer;
  ewl_heap_used = 0;
}

/* End the current encode session.
 *
 * Calls H264EncStrmEnd() to output an EOS NAL unit (caller discards it) and
 * transitions the encoder ENCODING -> INITIALIZED.  After this call,
 * H264EncStrmStart() can be safely called for the next session, which
 * re-emits SPS/PPS so a fresh ffplay instance can decode the new stream.
 *
 * Must be called at the end of every stream session.
 * H264EncRelease() must NOT be called — it crashes on this target. */
int ENC_EndSession(uint8_t *p_out, size_t out_len)
{
  struct VENC_Context *p_ctx = &VENC_Instance;
  H264EncOut enc_out;
  H264EncIn  enc_in;
  int ret;

  if (!p_ctx->is_sps_pps_done)
    return H264ENC_OK; /* nothing to end */

  memset(&enc_in, 0, sizeof(enc_in));
  enc_in.pOutBuf    = (u32 *) p_out;
  enc_in.busOutBuf  = (ptr_t) p_out;
  enc_in.outBufSize = (u32)   out_len;

  ret = H264EncStrmEnd(p_ctx->hdl, &enc_in, &enc_out);

  if (ret == H264ENC_OK) {
    p_ctx->is_sps_pps_done = 0;
    p_ctx->pic_cnt         = 0;
  }
  return ret;
}

/* Reset per-stream state for the next session (called after ENC_EndSession). */
void ENC_ResetSession(void)
{
  VENC_Instance.pic_cnt = 0;
}

int ENC_EncodeFrame(uint8_t *p_in, uint8_t *p_out, size_t out_len, int is_intra_force)
{
  size_t out_compressed_frame_len;
  int ret;

  ret = VENC_Encode(p_in, p_out, out_len, &out_compressed_frame_len, is_intra_force);

  return ret ? -1 : out_compressed_frame_len;
}

/* -------------------------------------------------------------------------
 * Safe bump allocator for EWL heap calls (replaces malloc / calloc / free).
 *
 * Root-cause of H264ENC_INSTANCE_ERROR:
 *   h264Instance_s is ~15 KB.  The system heap after BSS has only 4.5 KB
 *   (0x341f5e10 – 0x341f7000).  LibNosys _sbrk has no bounds check, so
 *   calloc() overflows past 0x341f7000 into usbx_mem_pool_uncached which
 *   starts at exactly 0x341f7000.  ux_system_initialize() zeros that pool,
 *   wiping the inst self-pointer checksum buried ~12 KB into the struct.
 *
 *   Fix: allocate from a dedicated static pool placed in AXISRAM3456, far
 *   from the heap/ISR-stack/uncached-USB-pool collision zone.
 * ------------------------------------------------------------------------- */
void *EWLmalloc(u32 n)
{
  size_t aligned = ((size_t)n + 7u) & ~7u;  /* 8-byte align */
  assert(ewl_heap_used + aligned <= EWL_HEAP_POOL_SIZE);
  void *ptr = &ewl_heap_pool[ewl_heap_used];
  ewl_heap_used += aligned;
  return ptr;
}

void EWLfree(void *p)
{
  (void)p; /* bump allocator — no individual deallocation needed */
}

void *EWLcalloc(u32 n, u32 s)
{
  size_t total = (size_t)n * (size_t)s;
  void *ptr = EWLmalloc(total);
  memset(ptr, 0, total);
  return ptr;
}

/* Implement simple EWLMallocLinear. No dealloc supported */
i32 EWLMallocLinear(const void *instance, u32 size, EWLLinearMem_t *info)
{
  if (venc_hw_allocator_pos + size > venc_hw_allocator_buffer + VENC_ALLOCATOR_SIZE)
    return -1;

  info->size = size;
  info->virtualAddress = (u32 *) venc_hw_allocator_pos;
  info->busAddress = (ptr_t)info->virtualAddress;

  venc_hw_allocator_pos += size;

  return 0;
}
