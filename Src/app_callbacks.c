/**
 ******************************************************************************
 * @file    app_callbacks.c
 * @brief   Camera & button callbacks (weak-symbol overrides).
 ******************************************************************************
 */
#include "app_callbacks.h"
#include "app_shared.h"

#include <stdio.h>
#include "app_cam.h"
#include "cmw_camera.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_dcmipp.h"
#ifdef STM32N6570_DK_REV
#include "stm32n6570_discovery.h"
#else
#include "stm32n6xx_nucleo.h"
#endif

/* ==========================================================================
 * Camera & button callbacks
 * ========================================================================== */

int CMW_CAMERA_PIPE_FrameEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1) {
    if (h264_streaming) {
      /* Double-buffer mode: P1STM0AR (status reg) holds the address of the
       * buffer the hardware just completed (VENC_SDCard example pattern). */
      h264_ready_buf = (uint8_t *)hcamera_dcmipp.Instance->P1STM0AR;
      h264_frame_ready = 1;
    }
    else if (!warmup_done)
      warmup_frames++;
    else if (snapshot_in_progress)
      frame_ready = 1;
  }
  return HAL_OK;
}

int CMW_CAMERA_PIPE_VsyncEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    CAM_IspUpdate();
  return HAL_OK;
}

void BSP_PB_Callback(Button_TypeDef Button)
{
  if(Button == BUTTON_USER1){
  	state = SD_CARD;
  	printf("[FSM] RESTART OF THE CONFIG PROCEDURE...\r\n");
  }
}

