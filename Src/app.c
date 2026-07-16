/**
 ******************************************************************************
 * @file    app.c
 * @brief   DIAS application core.
 *
 * Building blocks available in this module:
 *   - Camera capture through DCMIPP (pipe1 full-res, pipe2 downsized)
 *   - Hardware JPEG encoding (JPG_*) and transfer over UART to the Python GUI
 *   - H264 encoding (VENC hardware, ENC_*) of a 1280x720@30 RGB565 stream
 *   - MP4 (video) and JPEG (snapshot) recording to microSD (REC_*, FreeRTOS)
 *   - Runtime DCMIPP crop/decimation/downsize configuration received from
 *     the Python GUI over UART (Config_t)
 *
 * app_run() runs the DIAS state machine, on top of the helpers
 * (SD_init / app_mode_config_run).
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

#include "app.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_shared.h"
#include "app_rtc.h"
#include "app_uart.h"
#include "app_capture.h"
#include "app_pipes.h"
#include "app_record.h"
#include "app_callbacks.h"
#include "app_rec.h"

#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_dcmipp.h"
#ifdef STM32N6570_DK_REV
#include "stm32n6570_discovery.h"
#include "stm32n6570_discovery_sd.h"
#else
#include "stm32n6xx_nucleo.h"
#endif
#include "FreeRTOS.h"
#include "task.h"
#include "utils.h"

/* ==========================================================================
 * Module state (shared definitions; declared extern in app_shared.h)
 * ========================================================================== */

/* Configuration received from the Python GUI */
Config_t config_py = { 0 };

/* DIAS state machine */
state_t state = SD_CARD;

/* Capture buffers (PSRAM) */
uint8_t buffer_full_frame[MAX_CAPTURE_FRAME_SIZE] ALIGN_32 IN_PSRAM;
uint8_t hires_jpeg_buffer[MAX_JPEG_FRAME_SIZE] ALIGN_32 IN_PSRAM;

/* Warmup capture placeholder (pipe2 destination, unused in single-pipe modes) */
uint8_t *buffer_warmup = NULL;

/* Hardware JPEG encoder configuration */
JPG_conf_t jpg_conf = { 0 };

/* Capture/mode flags */
volatile int snapshot_in_progress = 0;
volatile int frame_ready = 0;
volatile int warmup_frames = 0;
volatile int warmup_done = 0;
volatile int uart_busy = 0; //1 = UART used for binary data, printf muted

/* H264 recording state (shared with app_record.c / app_callbacks.c) */
volatile int h264_streaming = 0;
volatile int h264_frame_ready = 0;
volatile int force_intra = 0;
uint8_t * volatile h264_ready_buf = NULL;

/* ==========================================================================
 * Console & memory helpers
 * ========================================================================== */

int __io_putchar(int ch)
{
  if (!uart_busy)
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
  return ch;
}

/* Simple bump allocator in AXISRAM (allocations are never freed) */
__attribute__((section(".axisram_bss"))) static uint8_t axisram_pool[1 * 1024 * 1024]; //FIXME: problème repéré avec Léonard
static uint32_t axisram_offset = 0;

void *axisram_alloc(uint32_t size)
{
  void *ptr;

  taskENTER_CRITICAL();

  /* Keep every allocation 32-byte aligned (cache line) */
  axisram_offset = (axisram_offset + 31) & ~31;

  if (axisram_offset + size > sizeof(axisram_pool)) {
    taskEXIT_CRITICAL();
    while (1) {} /* pool exhausted: trap */
  }
  ptr = &axisram_pool[axisram_offset];
  axisram_offset += size;

  taskEXIT_CRITICAL();

  return ptr;
}

/* Resets the AXISRAM bump allocator (detect-mode re-entry reuses the pool) */
void axisram_reset(void)
{
  taskENTER_CRITICAL();
  axisram_offset = 0;
  taskEXIT_CRITICAL();
}

/* ==========================================================================
 * Public API (building blocks for the state machine)
 * ========================================================================== */

/* One-time peripheral init: LEDs, TAMP button (polling) and SD recorder
 * (SD card + FAT32 mount + FreeRTOS SD writer task). */
int SD_init(void)
{
	int rec_ready = REC_Init();
	switch(rec_ready)
	{
	case 0:
		printf("[uSD] external memory init successful\r\n");
		break;
	case -1:
		printf("[uSD] required formatting failed (FAT32)\r\n");
		break;
	case -2:
		printf("[uSD] no uSD card detected/mounted (retry in 2 sec)\r\n");
		break;
	case -3:
		printf("[uSD] SDMMC2 clock config failed\r\n");
		break;
	default:
		break;
	}

  return (rec_ready == 0);
}

void LED_mode(void)
{
	if(state < MOVEMENT_DETECTION){ // configuration process
		BSP_LED_Off(LED_GREEN);
		BSP_LED_On(LED_RED);
	}
	else{ // detection phase
		BSP_LED_On(LED_GREEN);
		BSP_LED_Off(LED_RED);
	}
}

/* ==========================================================================
 * Application entry point
 * ========================================================================== */

void app_run(void)
{
	/* TAMP button read by polling in MOVEMENT_DETECTION */
	BSP_PB_Init(BUTTON_TAMP, BUTTON_MODE_GPIO);

	while(1)
	{
		LED_mode();

		switch(state)
		{
		case SD_CARD:
			if(SD_init()){
				state = CONFIG_MODE_WARMUP;
				break;
			}
			HAL_Delay(2000);
			break;

		case CONFIG_MODE_WARMUP:
			printf("[FSM] config mode warmup... (%d frames @ %d fps)\r\n",
					WARMUP_FRAMES_TARGET, SENSOR_WARMUP_FPS);
			camera_warmup(SENSOR_WIDTH, SENSOR_HEIGHT, DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1);
			printf("[FSM] config warmup ended\r\n");

			state = SEND_YUV_FRAME;
			printf("[FSM] wait for send yuv frame... (capturer une image)\r\n");
			break;

		case SEND_YUV_FRAME:
			uint8_t cmd = 0;
			HAL_UART_Receive(&huart1, &cmd, 1, 100);

			if (cmd == 'S'){
				int jpeg_len = capture_yuv();
				printf("[FSM] frame captured: %d KB\r\n", jpeg_len / 1024);
				HAL_Delay(50);
				send_jpeg_uart(hires_jpeg_buffer, jpeg_len);
			}
			else if (cmd == 'T'){
				uint8_t dt[6] = { 0 };
				if (HAL_UART_Receive(&huart1, dt, sizeof(dt), 1000) == HAL_OK)
					rtc_set_datetime(dt);
			}
			else if(cmd == 'V'){
				state = RECEIVE_PIPES_CONFIG;
			}
			break;

		case RECEIVE_PIPES_CONFIG:
			uint8_t buffer[sizeof(Config_t)];
			uint8_t answer = 'F'; //F = Fail

			HAL_UART_Receive(&huart1, buffer, sizeof(Config_t), 100);
			memcpy(&config_py, buffer, sizeof(Config_t));
			if (config_py.magic == CONFIG_MAGIC){
				printf("[FSM] pipes config successfully received\r\n");
				answer = 'V';
				HAL_UART_Transmit(&huart1, &answer, 1, 100);

				state = DETECT_MODE_WARMUP;
				break;
			}
			HAL_UART_Transmit(&huart1, &answer, 1, 100);
			break;

		case DETECT_MODE_WARMUP:
			printf("[FSM] detection mode warmup... (%d frames @ %d fps)\r\n",
					WARMUP_FRAMES_TARGET, SENSOR_WARMUP_FPS);
			camera_warmup(SENSOR_WIDTH, SENSOR_HEIGHT, DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1);
			printf("[FSM] detection warmup ended\r\n");

			state = PIPES_CONFIGURATION;
			break;

		case PIPES_CONFIGURATION:
			printf("[FSM] pipes configuration procedure\r\n");
			dcmipp_apply_detect_config();

			state = MOVEMENT_DETECTION;
			printf("[FSM] start movement detection... (TAMP button)\r\n");
			break;

		case MOVEMENT_DETECTION:
			if(BSP_SD_GetCardState(0) != SD_TRANSFER_OK){
				printf("[uSD] uSD has been disconnected, config procedure restart...\r\n");
				state = SD_CARD;
				break;
			}

			if(BSP_PB_GetState(BUTTON_TAMP) == GPIO_PIN_SET){
				printf("[FSM] movement detected!\r\n");
				state = RECORDING;
				break;
			}
			HAL_Delay(1000);
			break;

		case RECORDING:
			char timestamp[20];
			rtc_make_timestamp(timestamp, sizeof(timestamp));

			record_jpeg_sd(timestamp, 1080); //jpg height 4:3 : 480, 720, 960, 1080 ... 1944 (full res.)
			record_h264_sd(timestamp, 1080); //mp4 height 4:3 : 480, 720, 960, 1080 (max)

			state = DETECT_MODE_WARMUP;
			break;

		default:
			break;
		}
	}
}
