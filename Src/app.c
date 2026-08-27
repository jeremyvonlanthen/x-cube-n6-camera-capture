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
#include "app_cam.h"
#include "app_pipes.h"
#include "app_record.h"
#include "app_callbacks.h"
#include "app_rec.h"
#include "app_sleep.h"

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
state_t state = CONFIG_MODE_WARMUP;
mode_t mode = _CONFIG;

/* Capture buffers (PSRAM) */
uint8_t buffer_full_frame[MAX_CAPTURE_FRAME_SIZE] ALIGN_32 IN_PSRAM;
uint8_t hires_jpeg_buffer[MAX_JPEG_FRAME_SIZE] ALIGN_32 IN_PSRAM;

/* Warmup capture placeholder (pipe2 destination, unused in single-pipe modes) */
uint8_t *buffer_warmup = NULL;

/* Hardware JPEG encoder configuration */
JPG_conf_t jpg_conf = { 0 };

/* Capture/mode flags */
volatile int sd_reinit_for_storage = 0;
volatile int snapshot_in_progress = 0;
volatile int frame_ready = 0;
volatile int warmup_frames = 0;
volatile int warmup_done = 0;
volatile int uart_busy = 0; //1 = UART used for binary data, printf muted
volatile int config_already_saved = 0;

/* H264 recording state (shared with app_record.c / app_callbacks.c) */
volatile int h264_streaming = 0;
volatile int h264_frame_ready = 0;
volatile int force_intra = 0;
uint8_t * volatile h264_ready_buf = NULL;
uint32_t actual_ticks;

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

	#if DEBUG_MODE
	HAL_DBGMCU_EnableDBGSleepMode();
	HAL_DBGMCU_EnableDBGStopMode();
	HAL_DBGMCU_EnableDBGStandbyMode();
	#endif

	char timestamp[20];
	int rec_files_height = 1080; // 480, 720, 960, 1080 (max)

	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET){
		printf("[FSM] RUNS NOW IN DIURNE MODE (until system restart)\r\n");
		mode = _DIURNE;
		state = SD_CARD_INIT;
	}
	else if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_5) == GPIO_PIN_RESET){
		printf("[FSM] RUNS NOW IN 24H MODE (until system restart)\r\n");
		mode = _24H;
		state = SD_CARD_INIT;
	}
	else
		printf("[FSM] RUNS NOW IN CONFIG MODE (until system restart)\r\n");

	while(1)
	{
		#if DEBUG_MODE
		LED_mode();
		#endif

		switch(state)
		{
		case CONFIG_MODE_WARMUP:
			printf("[FSM] config-mode warmup... (%d frames @ %d fps)\r\n", WARMUP_FRAMES_TARGET, SENSOR_WARMUP_FPS);
			camera_warmup(SENSOR_WIDTH, SENSOR_HEIGHT, DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1);

			printf("[FSM] wait for send yuv frame... (capturer une image)\r\n");
			state = SEND_YUV_FRAME;
			break;

		case SEND_YUV_FRAME:
			uint8_t cmd = 0;
			HAL_UART_Receive(&huart1, &cmd, 1, 100);

			switch(cmd)
			{
			case 'S':
				int jpeg_len = capture_yuv();
				printf("[FSM] frame captured: %d KB\r\n", jpeg_len / 1024);
				HAL_Delay(50);
				send_jpeg_uart(hires_jpeg_buffer, jpeg_len);
				break;

			case 'T':
				uint8_t dt[6] = { 0 };
				if (HAL_UART_Receive(&huart1, dt, sizeof(dt), 1000) == HAL_OK)
					rtc_set_datetime(dt);
				break;

			case 'V':
				state = RECEIVE_PIPES_CONFIG;
				break;
			}
			break;

		case RECEIVE_PIPES_CONFIG:
			uint8_t buffer[sizeof(Config_t)];
			uint8_t answer = 'F'; //F = Fail

			HAL_UART_Receive(&huart1, buffer, sizeof(Config_t), 100);
			memcpy(&config_py, buffer, sizeof(Config_t));

			if (config_py.magic == CONFIG_MAGIC){
				answer = 'V';
				printf("[FSM] pipes config successfully received\r\n");
				HAL_UART_Transmit(&huart1, &answer, 1, 100);

				state = SAVE_PIPES_CONFIG;
				break;
			}
			HAL_UART_Transmit(&huart1, &answer, 1, 100);
			break;

		case SAVE_PIPES_CONFIG:
			if(config_already_saved) break;

			//sauver la configuration dans la flash pour pouvoir y réaccéder après une extinction du STM
			//déclarer une adresse fixe pour pouvoir accéder à la config sauvée
			printf("[FSM] pipes config saved\n\r");
			config_already_saved = 1;
			break;

		case SD_CARD_INIT:
			if(SD_init()){
				if(sd_reinit_for_storage){
					sd_reinit_for_storage = 0;
					state = MULTIMEDIA_STORAGE;
					break;
				}

				REC_PowerDownSD();
				state = DETECT_MODE_WARMUP;
				break;
			}
			sleep_short_period(2000);
			break;

		case DETECT_MODE_WARMUP:
			printf("[FSM] detection-mode warmup... (%d frames @ %d fps)\r\n", WARMUP_FRAMES_TARGET, SENSOR_WARMUP_FPS);
			camera_warmup(SENSOR_WIDTH, SENSOR_HEIGHT, DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1);

			printf("[FSM] pipes configuration procedure\r\n");
			dcmipp_apply_detect_config(); //utiliser ici la config précédemment sauvée dans la flash

			state = OP_WINDOW_CHECK;
			break;

		case OP_WINDOW_CHECK:
			if(mode == _24H){
				state = MOVEMENT_DETECTION;
				break;
			}

			//à l'avenir, contrôle de la RTC/ALS
			//si nuit: standby/système OFF
			//si jour: state = MOVEMENT_DETECTION
			uint8_t day_window = 1;
			printf("[FSM] check diurne operating window: %s\r\n", day_window ? "DAY" : "NIGHT");

			if(day_window)
				state = MOVEMENT_DETECTION;
			else{} //standby/système OFF
			break;

		case MOVEMENT_DETECTION:
//			int ret = capture_detect_frame();
			//ajouter ici le code de Léonard: capture d'image et algo détection sur les 2 pipes

			if(BSP_PB_GetState(BUTTON_TAMP) == GPIO_PIN_SET){
				printf("[FSM] movement detected!\r\n");
				actual_ticks = HAL_GetTick();
				state = RECORD_MODE_INIT;
				break;
			}

			//check of mecanical insertion of SD card
			if(BSP_SD_IsDetected(0) != SD_PRESENT){
				printf("[uSD] uSD has been removed, SD re-init...\r\n");
				state = SD_CARD_INIT;
				break;
			}

			sleep_short_period(1000);
			state = OP_WINDOW_CHECK;
			break;

		case RECORD_MODE_INIT:
//			rtc_make_timestamp(timestamp, sizeof(timestamp));
//			record_jpeg_sd(timestamp, rec_files_height);
//			record_camera_setup(rec_files_height);

			state = VIDEO_RECORDING;
			break;

		case VIDEO_RECORDING:
			//ajouter à l'avenir un contrôle // de mouvement avec le pipe0

//			record_h264_run(timestamp, rec_files_height, 8);

			sd_reinit_for_storage = 1;
			state = SD_CARD_INIT;
			break;

		case MULTIMEDIA_STORAGE:
			//sauvegarde jpeg et mp4 dans la carte SD
			REC_PowerDownSD();
			state = DETECT_MODE_WARMUP;
			break;

		default:
			break;
		}
	}
}
