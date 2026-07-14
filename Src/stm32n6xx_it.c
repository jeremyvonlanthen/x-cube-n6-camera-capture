 /**
 ******************************************************************************
 * @file    stm32n6xx_it.c
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

/* Includes ------------------------------------------------------------------*/
#include "stm32n6xx_hal.h"
#include "stm32n6xx_it.h"
#include "stm32n6570_discovery.h"
#include "stm32n6570_discovery_sd.h"

#include "cmw_camera.h"

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Secure Fault exception.
  * @param  None
  * @retval None
  */
void SecureFault_Handler(void)
{
  /* Go to infinite loop when Secure Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
  while (1)
  {
  }
}

/******************************************************************************/
/*                 STM32N6xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32n6xx.s).                                               */
/******************************************************************************/

void CSI_IRQHandler(void)
{
  HAL_DCMIPP_CSI_IRQHandler(CMW_CAMERA_GetDCMIPPHandle());
}

void DCMIPP_IRQHandler(void)
{
  HAL_DCMIPP_IRQHandler(CMW_CAMERA_GetDCMIPPHandle());
}

/* SDMMC2 (microSD via BSP).  DMA transfer completion is signaled through
 * the BSP handler, which dispatches to BSP_SD_Write/ReadCpltCallback
 * (implemented in sd_diskio.c). */
void SDMMC2_IRQHandler(void)
{
  BSP_SD_IRQHandler(0);
}

void HardFault_Handler(void)
{
//  printf("HardFault! HFSR=%08lX CFSR=%08lX BFAR=%08lX MMFAR=%08lX\n",
//         SCB->HFSR, SCB->CFSR, SCB->BFAR, SCB->MMFAR);
  while(1);
}

// Ajouter avec les autres handlers
extern JPEG_HandleTypeDef hjpeg_instance; // forward déclaration

void JPEG_IRQHandler(void)
{
  // accéder au hjpeg via jpg_ctx
  extern void JPG_IRQHandler(void);
  JPG_IRQHandler();
}

void EXTI13_IRQHandler(void)  // à adapter selon BUTTON_USER1_EXTI_IRQn
{
    BSP_PB_IRQHandler(BUTTON_USER1);
}
