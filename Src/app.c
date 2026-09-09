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
#include "cmw_camera.h"
#include "app_detect.h"
#include "app_flash_config.h"
#include "app_pipes.h"
#include "app_record.h"
#include "app_callbacks.h"
#include "app_rec.h"
#include "app_sleep.h"

#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_dcmipp.h"
#ifdef STM32N6570_DK_REV
#include "stm32n6570_discovery.h"
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
volatile bool snapshot_in_progress = false;
volatile bool frame_ready = false;
volatile int  warmup_frames = 0;
volatile bool warmup_done = false;
volatile bool uart_busy = false; //true = UART used for binary data, printf muted
uint32_t actual_ticks;

/* H264 recording state (shared with app_record.c / app_callbacks.c) */
volatile bool h264_streaming = false;
volatile bool h264_frame_ready = false;
volatile bool force_intra = false;
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

/* TODO: replace with the real animal-classification algorithm (NPU model?).
 * Runs on the still image just captured by record_snapshot_to_ram() (in
 * buffer_full_frame / hires_jpeg_buffer). For now always answers "yes" so
 * the video path is what gets exercised end-to-end until a real classifier
 * is wired in here. */
static bool is_target_animal_detected(void)
{
  return true; /* TODO */
}

/* ==========================================================================
 * Application entry point
 * ========================================================================== */

void app_run(void)
{
	#if DEBUG_MODE
	HAL_DBGMCU_EnableDBGSleepMode();
	HAL_DBGMCU_EnableDBGStopMode();
	HAL_DBGMCU_EnableDBGStandbyMode();
	#endif

	char timestamp[20];
	char path[48]; // timestamp (19) + "/_config-sys_data-det.json" (26) + '\0'
	int rec_files_height = 960; // 480, 720, 960, 1080 (max)

	bool is_img_to_save = false;
	bool is_video_to_record = false;
	bool sd_reinit_for_storage = false;
	bool config_already_saved = false;
	DETECT_Result_t detect_result = {0};
	int32_t detect_exposure = 0, detect_gain = 0;

	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET){
		printf("[FSM] RUNS NOW IN 24H MODE (until system restart)\r\n");
		mode = _24H;
		state = SD_CARD_INIT;
	}
	else if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_5) == GPIO_PIN_RESET){
		printf("[FSM] RUNS NOW IN DIURNAL MODE (until system restart)\r\n");
		mode = _DIURNAL;
		state = SD_CARD_INIT;
	}
	else printf("[FSM] RUNS NOW IN CONFIG MODE (until system restart)\r\n");

	while(1)
	{
		switch(state)
		{
		case CONFIG_MODE_WARMUP:
			printf("[FSM] config-mode warmup... (%d frames @ %d fps)\r\n", WARMUP_FRAMES_TARGET, SENSOR_WARMUP_FPS);
			camera_warmup(SENSOR_WIDTH, SENSOR_HEIGHT, DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1, 0);

			printf("[FSM] wait to get config frame... (capturer une image)\r\n");
			state = SEND_IMG_FRAME;
			break;

		case SEND_IMG_FRAME:
			uint8_t cmd = 0;
			HAL_UART_Receive(&huart1, &cmd, 1, 100);

			switch(cmd)
			{
			case 'S':
				int jpeg_len = capture_img();
				printf("[FSM] frame captured: %d KB\r\n", jpeg_len / 1024);
				HAL_Delay(50);
				int32_t cap_exposure = 0, cap_gain = 0;
				CMW_CAMERA_GetExposure(&cap_exposure);
				CMW_CAMERA_GetGain(&cap_gain);
				send_img_uart(hires_jpeg_buffer, jpeg_len, cap_exposure, cap_gain);
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
				printf("[FSM] pipes config successfully received, storing...\r\n");
				HAL_UART_Transmit(&huart1, &answer, 1, 100);

				state = SAVE_PIPES_CONFIG;
				break;
			}
			HAL_UART_Transmit(&huart1, &answer, 1, 100);
			break;

		case SAVE_PIPES_CONFIG:
			if(config_already_saved) break;

			if (CONFIG_FLASH_Save(&config_py) == 0)
				printf("[FSM] pipes config saved to flash\n"
							 "[FSM] now ready to execute diurnal or 24h mode\r\n");
			else printf("[FSM] pipes config flash save FAILED\r\n");

			config_already_saved = true;
			break;

		case SD_CARD_INIT:
			if(SD_init(sd_reinit_for_storage)){
				if(sd_reinit_for_storage){
					sd_reinit_for_storage = false;
					state = MULTIMEDIA_STORAGE;
				}
				else{
					SD_PowerDown();
					state = DETECT_MODE_WARMUP;
				}
				break;
			}
			sleep_short_period(2000);
			break;

		case DETECT_MODE_WARMUP:
			printf("[FSM] detection-mode warmup... (%d frames @ %d fps)\r\n", WARMUP_FRAMES_TARGET, SENSOR_WARMUP_FPS);
			camera_warmup(SENSOR_WIDTH, SENSOR_HEIGHT, DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1, 1);

			if(config_py.magic != CONFIG_MAGIC){
				if (CONFIG_FLASH_Load(&config_py) == 0) printf("[FSM] pipes config loaded from flash\r\n");
				else printf("[FSM] pipes config flash load FAILED (config_py memory empty)\r\n");
			}

			printf("[FSM] pipes configuration procedure\r\n");
			dcmipp_apply_detect_config();

			printf("[FSM] detect stats calibration...\r\n");
			DETECT_Init();
			DETECT_CalibrateStats();

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
			printf("[FSM] check diurnal operating window: %s\r\n", day_window ? "DAY" : "NIGHT");

			if(day_window){
				state = MOVEMENT_DETECTION;
				break;
			}

			//standby/système OFF
			break;

		case MOVEMENT_DETECTION:
			if(SD_inserted()){
				printf("[uSD] uSD has been removed, SD re-init...\r\n");
				state = SD_CARD_INIT;
				break;
			}

			if(DETECT_ProcessFrame(&detect_result)){
				CMW_CAMERA_GetExposure(&detect_exposure);
				CMW_CAMERA_GetGain(&detect_gain);
				printf("[FSM] movement detected! (second plan: %.2f%%, premier plan: %.2f%%)\r\n",
				       detect_result.second_plan.deviation_voisinage.pct_pipe,
				       detect_result.premier_plan.deviation_voisinage.pct_pipe);
				actual_ticks = HAL_GetTick();
				state = RECORD_MODE_INIT;
				break;
			}

			sleep_short_period(1000);
			state = OP_WINDOW_CHECK;
			break;

		case RECORD_MODE_INIT:
			rtc_make_timestamp(timestamp, sizeof(timestamp));
			record_snapshot_to_ram(rec_files_height, detect_exposure, detect_gain);

			/* is_target_animal_detected() picks the primary format (MP4 if
			 * true, JPEG if false); RECORD_JPEG_AND_MP4 force-saves the other
			 * one too instead of skipping it (see app_shared.h). */
			bool animal_detected = is_target_animal_detected();
			is_video_to_record = animal_detected || RECORD_JPEG_AND_MP4;
			is_img_to_save = !animal_detected || RECORD_JPEG_AND_MP4;

			if(is_video_to_record)
				state = VIDEO_CAPTURE;
			else{
				sd_reinit_for_storage = true;
				state = SD_CARD_INIT;
			}
			break;

		case VIDEO_CAPTURE:
			//ajouter à l'avenir un contrôle // de mouvement avec le pipe0

			setup_record_h264(rec_files_height);
			record_h264_to_ram(rec_files_height, VIDEO_DURATION_S);

			sd_reinit_for_storage = true;
			state = SD_CARD_INIT;
			break;

		case MULTIMEDIA_STORAGE:
			REC_MakeDir(timestamp);

			if(is_img_to_save){
				snprintf(path, sizeof(path), "%s/non-identifie.jpeg", timestamp);
				if(record_snapshot_flush_to_sd(path) != 0) printf("[REC] snapshot save FAILED\r\n");
			}

			if(is_video_to_record){
				snprintf(path, sizeof(path), "%s/non-identifie.mp4", timestamp);
				if(record_h264_flush_to_sd(path) != 0) printf("[REC] video save FAILED\r\n");
			}

			snprintf(path, sizeof(path), "%s/_config-sys_data-det.json", timestamp);
			if(record_detection_json_to_sd(path, timestamp, &detect_result, &config_py, rec_files_height,
			                                detect_exposure, detect_gain) != 0)
				printf("[REC] json save FAILED\r\n");

			SD_PowerDown();
			state = DETECT_MODE_WARMUP;
			break;

		default:
			break;
		}
	}
}
