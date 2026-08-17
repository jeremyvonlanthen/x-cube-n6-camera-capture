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
state_t state = CONFIG_MODE_WARMUP;

/* Capture buffers (PSRAM) */
uint8_t buffer_full_frame[MAX_CAPTURE_FRAME_SIZE] ALIGN_32 IN_PSRAM;
uint8_t hires_jpeg_buffer[MAX_JPEG_FRAME_SIZE] ALIGN_32 IN_PSRAM;

/* Warmup capture placeholder (pipe2 destination, unused in single-pipe modes) */
uint8_t *buffer_warmup = NULL;

/* Hardware JPEG encoder configuration */
JPG_conf_t jpg_conf = { 0 };

/* Capture/mode flags */
volatile int sd_initialized = 0;
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

	#if (DEBUG_KEEP_SWD_ALIVE_IN_LOWPOWER == 1)
	/* Without this, DBGMCU (and so SWD/ST-LINK) loses power/clock as soon as
	 * the core enters SLEEP/STOP, forcing a reconnect on every wake and
	 * eventually a failed halt -- exactly the "Could not halt device" seen
	 * when single-stepping/breakpointing strategies 2 and 3. Remove/disable
	 * before measuring real current: this keeps extra clocks running. */
	HAL_DBGMCU_EnableDBGSleepMode();
	HAL_DBGMCU_EnableDBGStopMode();
	HAL_DBGMCU_EnableDBGStandbyMode();
	#endif

	char timestamp[20];
	int rec_files_height = 1080; // 480, 720, 960, 1080 (max)

	while(1)
	{
		//LED_mode();

		switch(state)
		{
		case CONFIG_MODE_WARMUP:
			printf("[FSM] config mode warmup... (%d frames @ %d fps)\r\n",
					WARMUP_FRAMES_TARGET, SENSOR_WARMUP_FPS);
			camera_warmup(SENSOR_WIDTH, SENSOR_HEIGHT, DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1);
			printf("[FSM] config warmup ended\r\n");
#if 0
			state = SEND_YUV_FRAME;
#endif
			state = DETECT_MODE_WARMUP;

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

			printf("[FSM] pipes configuration procedure\r\n");
#if 0
			dcmipp_apply_detect_config();
#endif

			state = SD_CARD;
			break;

		case SD_CARD:
			if(!sd_initialized && SD_init()){
				sd_initialized = 1;
				state = OP_WINDOW_CHECK;
				break;
			}

			if(sd_initialized){
				if(BSP_SD_GetCardState(0) != SD_TRANSFER_OK){
					printf("[uSD] uSD has been disconnected, config procedure restart...\r\n");
					sd_initialized = 0;
				}
				else{
					state = OP_WINDOW_CHECK;
					break;
				}
			}

			vTaskDelay(pdMS_TO_TICKS(2000));
			break;

		case OP_WINDOW_CHECK:
			//TODO: check operation window (24h/diurne)

			printf("[FSM] start movement detection... (TAMP button)\r\n");
			state = MOVEMENT_DETECTION;
			break;

		case MOVEMENT_DETECTION:
			// Variables pour mesurer le temps d'exécution de l'algorithme
			uint32_t start_time = HAL_GetTick();

			// -------------------------------------------------------------
			// EXÉCUTION DE VOTRE ALGORITHME STATISTIQUE (Pipes 1 & 2)
			// -------------------------------------------------------------
			uint8_t movement_detected = 0; //run_statistical_algo_pipe1_pipe2();
			HAL_Delay(100); //100ms de traitement statistique

			if (movement_detected) {
				state = RECORD_MODE_INIT;
				break;
			}

			// Calcul du temps écoulé pendant l'algorithme (en millisecondes)
			uint32_t elapsed_time = HAL_GetTick() - start_time;

			// Sécurité : si l'algo prend plus d'une seconde, on évite un overflow
			uint32_t sleep_duration_ms = (elapsed_time < 1000) ? (1000 - elapsed_time) : 0;

			// -------------------------------------------------------------
			// APPLICATION DE LA STRATÉGIE D'ATTENTE SÉLECTIONNÉE
			//
			// Le STM32N6 n'expose que 3 modes basse consommation au niveau HAL
			// (pas de Stop 0/1/2 ni de "low-power run" comme sur L4/U5) :
			//   - SLEEP   : coeur Cortex-M55 arrêté, TOUT le reste (bus AXI/AHB/
			//               APB, PLL1..4, périphériques) reste actif -> réveil
			//               instantané, gain attendu FAIBLE sur cette carte car
			//               le coeur ne pèse qu'une fraction de la conso totale
			//               (4 PLL qui tournent, XSPI PSRAM/NOR en memory-mapped).
			//   - STOP    : coeur + horloges bus + les 4 PLL coupés (d'où le
			//               SystemClock_Config() au réveil), RAM/état conservés
			//               -> gain attendu le plus intéressant, réveil de
			//               l'ordre de la centaine de µs (relock PLL).
			//   - STANDBY : RAM perdue, redémarrage depuis le vecteur de reset
			//               -> incompatible avec la reprise de cette FSM toutes
			//               les ~1s (il faudrait tout réinitialiser: caméra, SD,
			//               config...), donc pas proposé ici.
			// -------------------------------------------------------------
			#if (SLEEP_STRATEGY == 1)
					// 1. Référentiel : HAL_Delay = attente active, CPU/bus/PLL au
					// maximum pendant toute la durée -> borne haute de consommation.
					HAL_Delay(sleep_duration_ms);

			#elif (SLEEP_STRATEGY == 2)
					// 2. Mode SLEEP (CSLEEP) : un seul WFI dimensionné exactement sur
					// sleep_duration_ms via le LPTIM1. Volontairement explicite (plutôt
					// que de compter sur le tickless-idle de FreeRTOS) pour avoir une
					// mesure reproductible, indépendante des autres tâches RTOS.
					if (sleep_duration_ms > 0) {
						uint32_t period_ticks = sleep_duration_ms * 32; /* LSI ~32kHz */
						if (period_ticks > 0xFFFF) period_ticks = 0xFFFF;

						hlptim1.Init.Period = period_ticks;
						HAL_LPTIM_Init(&hlptim1);
						HAL_LPTIM_Counter_Start_IT(&hlptim1);

						HAL_SuspendTick();
						HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
						/* SLEEP ne coupe ni les bus ni les PLL: pas de resync horloge. */
						HAL_ResumeTick();
					}

			#elif (SLEEP_STRATEGY == 3)
					// 3. Mode STOP (CSTOP) : coeur + bus + PLL1..4 coupés.
					if (sleep_duration_ms > 0) {
						uint32_t period_ticks = sleep_duration_ms * 32; /* LSI ~32kHz */
						if (period_ticks > 0xFFFF) period_ticks = 0xFFFF;

						hlptim1.Init.Period = period_ticks;
						HAL_LPTIM_Init(&hlptim1);
						HAL_LPTIM_Counter_Start_IT(&hlptim1);

						#if (STOP_MODE_NARROW_CLOCKS == 1)
						/* main.c active au boot le maintien de TOUS les périphériques
						 * en horloge basse-conso ("garder les IP actifs pour pouvoir
						 * réveiller le CPU"). Pour ce réveil piloté uniquement par le
						 * LPTIM1, restreindre ce maintien au strict minimum le temps
						 * du sommeil, puis restaurer la politique de boot au réveil. */
						LL_BUS_DisableClockLowPower(~0);
						LL_MEM_DisableClockLowPower(~0);
						LL_AHB1_GRP1_DisableClockLowPower(~0);
						LL_AHB2_GRP1_DisableClockLowPower(~0);
						LL_AHB3_GRP1_DisableClockLowPower(~0);
						LL_AHB4_GRP1_DisableClockLowPower(~0);
						LL_AHB5_GRP1_DisableClockLowPower(~0);
						LL_APB1_GRP1_DisableClockLowPower(~0);
						LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_LPTIM1);
						LL_APB1_GRP2_DisableClockLowPower(~0);
						LL_APB2_GRP1_DisableClockLowPower(~0);
						LL_APB4_GRP1_DisableClockLowPower(~0);
						LL_APB4_GRP2_DisableClockLowPower(~0);
						LL_APB5_GRP1_DisableClockLowPower(~0);
						LL_MISC_DisableClockLowPower(~0);
						#endif

						/* Left unconfigured, the Stop-mode regulator voltage range
						 * (SVOS) has no guaranteed value. Other users report the exact
						 * same "never wakes from Stop, Sleep works fine" symptom on
						 * this same board with LPTIM1, and ST support's first ask is
						 * always to check this setting (AN5946) -- try SCALE5 (lowest
						 * power) first; if it still never wakes, try SCALE3 instead. */
						HAL_PWREx_ControlStopModeVoltageScaling(PWR_REGULATOR_STOP_VOLTAGE_SCALE5);

						HAL_SuspendTick();
						/* Regulator param is ignored on STM32N6 (single regulator,
						 * kept only for source compat with other STM32 families).
						 * WFE instead of WFI: some STM32 parts only wake from Stop
						 * with WFE, not WFI (known issue class on other families --
						 * worth ruling out here since WFI alone never returns). */
						HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);

						/* DIAGNOSTIC: proves WFE actually returned (LPTIM wake worked).
						 * Stays solid ON forever if SystemClock_Config() hangs below
						 * (it has a bare while(1) on PLL relock failure) -- remove once
						 * strategy 3 is confirmed stable. */
						BSP_LED_On(LED_RED);

						#if (STOP_MODE_NARROW_CLOCKS == 1)
						LL_BUS_EnableClockLowPower(~0);
						LL_MEM_EnableClockLowPower(~0);
						LL_AHB1_GRP1_EnableClockLowPower(~0);
						LL_AHB2_GRP1_EnableClockLowPower(~0);
						LL_AHB3_GRP1_EnableClockLowPower(~0);
						LL_AHB4_GRP1_EnableClockLowPower(~0);
						LL_AHB5_GRP1_EnableClockLowPower(~0);
						LL_APB1_GRP1_EnableClockLowPower(~0);
						LL_APB1_GRP2_EnableClockLowPower(~0);
						LL_APB2_GRP1_EnableClockLowPower(~0);
						LL_APB4_GRP1_EnableClockLowPower(~0);
						LL_APB4_GRP2_EnableClockLowPower(~0);
						LL_APB5_GRP1_EnableClockLowPower(~0);
						LL_MISC_EnableClockLowPower(~0);
						#endif

						/* Relancer le tick AVANT SystemClock_Config(): cette dernière
						 * appelle HAL_Delay(1) (rampe SMPS), et HAL_Delay() est
						 * remappé sur vTaskDelay() dans freertos_bsp.c -- il a donc
						 * besoin du tick FreeRTOS pour se débloquer. Le faire dans
						 * l'autre sens bloque la tâche indéfiniment (tick suspendu
						 * = plus aucun réveil possible pour ce vTaskDelay). */
						HAL_ResumeTick();

						/* PLL1..4 coupés par le mode STOP -> reconfig obligatoire. */
						SystemClock_Config();

						BSP_LED_Off(LED_RED); /* reached only if OscConfig/ClockConfig didn't trap */
					}

			#elif (SLEEP_STRATEGY == 4)
					// 4. SLEEP mode (confirmed reliable) + manual PLL shutdown: STOP
					// mode's LPTIM wake-up doesn't come back on this board (see
					// ST ticket), so instead of chasing that further, attack the
					// actual suspected dominant power draw directly while staying
					// on the wake path we know works. Before sleeping: move
					// CPUCLK/SYSCLK off the PLL tree onto HSI directly, then switch
					// PLL1..4 OFF (must be done in that order -- a PLL can't be
					// disabled while still selected as a clock source). On wake:
					// SystemClock_Config() (already used by strategy 3, known
					// working) puts the PLLs back and restores full speed.
					if (sleep_duration_ms > 0) {
						uint32_t period_ticks = sleep_duration_ms * 32; /* LSI ~32kHz */
						if (period_ticks > 0xFFFF) period_ticks = 0xFFFF;

						hlptim1.Init.Period = period_ticks;
						HAL_LPTIM_Init(&hlptim1);
						HAL_LPTIM_Counter_Start_IT(&hlptim1);

						/* Still in Run mode / tick running here: safe to use HAL_Delay
						 * indirectly (HAL_RCC_OscConfig polls PLL flags, not the tick). */
						RCC_ClkInitTypeDef clk_lp = {0};
						RCC_OscInitTypeDef osc_lp = {0};

						/* MSI@4MHz instead of HSI (~64MHz): lowest-frequency oscillator
						 * this family offers for CPUCLK/SYSCLK. Must be turned on and
						 * stable BEFORE it's selected as a clock source below. */
						osc_lp.OscillatorType = RCC_OSCILLATORTYPE_MSI;
						osc_lp.MSIState       = RCC_MSI_ON;
						osc_lp.MSIFrequency   = RCC_MSI_FREQ_4MHZ;
						HAL_RCC_OscConfig(&osc_lp);

						clk_lp.ClockType    = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK;
						clk_lp.CPUCLKSource = RCC_CPUCLKSOURCE_MSI;
						clk_lp.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
						HAL_RCC_ClockConfig(&clk_lp);

						/* Now safe to switch PLL1..4 off (no longer selected as source). */
						osc_lp.OscillatorType = RCC_OSCILLATORTYPE_NONE;
						osc_lp.PLL1.PLLState = RCC_PLL_OFF;
						osc_lp.PLL2.PLLState = RCC_PLL_OFF;
						osc_lp.PLL3.PLLState = RCC_PLL_OFF;
						osc_lp.PLL4.PLLState = RCC_PLL_OFF;
						HAL_RCC_OscConfig(&osc_lp);

						/* Isolated re-test of clock-gating alone (VOS1/VOS0 abandoned
						 * separately -- confirmed twice to break things on its own,
						 * this wasn't tested cleanly before since it was previously
						 * bundled with VOS). Narrow the "peripheral clock kept alive
						 * in low-power mode" bits down to LPTIM1 + PWR (AHB4 also
						 * carries PWR's own bit -- gating it broke this same test
						 * last time). SLEEP wakes via plain NVIC, not the PWR/EXTI
						 * deep-sleep circuit, so this should be safer here than it
						 * was for strategy 3's STOP mode. */
						LL_BUS_DisableClockLowPower(~0);
						LL_MEM_DisableClockLowPower(~0);
						LL_AHB1_GRP1_DisableClockLowPower(~0);
						LL_AHB2_GRP1_DisableClockLowPower(~0);
						LL_AHB3_GRP1_DisableClockLowPower(~0);
						LL_AHB4_GRP1_DisableClockLowPower(~0);
						LL_AHB4_GRP1_EnableClockLowPower(LL_AHB4_GRP1_PERIPH_PWR);
						LL_AHB5_GRP1_DisableClockLowPower(~0);
						LL_APB1_GRP1_DisableClockLowPower(~0);
						LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_LPTIM1);
						LL_APB1_GRP2_DisableClockLowPower(~0);
						LL_APB2_GRP1_DisableClockLowPower(~0);
						LL_APB4_GRP1_DisableClockLowPower(~0);
						LL_APB4_GRP2_DisableClockLowPower(~0);
						LL_APB5_GRP1_DisableClockLowPower(~0);
						LL_MISC_DisableClockLowPower(~0);

						HAL_SuspendTick();
						HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

						/* Same ordering rule as strategy 3: resume tick before any
						 * call that (indirectly) uses HAL_Delay()/vTaskDelay(). */
						HAL_ResumeTick();

						/* Restore the boot-time "keep everything alive" policy before
						 * the rest of the FSM resumes normal operation. */
						LL_BUS_EnableClockLowPower(~0);
						LL_MEM_EnableClockLowPower(~0);
						LL_AHB1_GRP1_EnableClockLowPower(~0);
						LL_AHB2_GRP1_EnableClockLowPower(~0);
						LL_AHB3_GRP1_EnableClockLowPower(~0);
						LL_AHB4_GRP1_EnableClockLowPower(~0);
						LL_AHB5_GRP1_EnableClockLowPower(~0);
						LL_APB1_GRP1_EnableClockLowPower(~0);
						LL_APB1_GRP2_EnableClockLowPower(~0);
						LL_APB2_GRP1_EnableClockLowPower(~0);
						LL_APB4_GRP1_EnableClockLowPower(~0);
						LL_APB4_GRP2_EnableClockLowPower(~0);
						LL_APB5_GRP1_EnableClockLowPower(~0);
						LL_MISC_EnableClockLowPower(~0);

						SystemClock_Config(); /* PLLs back ON, full speed restored */

						/* MSI is no longer selected as CPUCLK/SYSCLK source at this
						 * point (SystemClock_Config moved it to the PLL/IC tree) --
						 * turn it back off so it isn't left running uselessly until
						 * the next sleep window. */
						RCC_OscInitTypeDef osc_msi_off = {0};
						osc_msi_off.OscillatorType = RCC_OSCILLATORTYPE_MSI;
						osc_msi_off.MSIState       = RCC_MSI_OFF;
						HAL_RCC_OscConfig(&osc_msi_off);
					}
			#endif
			BSP_LED_On(LED_GREEN);
			HAL_Delay(10);
			BSP_LED_Off(LED_GREEN);

			//state = SD_CARD;
			break;

		case RECORD_MODE_INIT:
			rtc_make_timestamp(timestamp, sizeof(timestamp));
			record_jpeg_sd(timestamp, rec_files_height);
			record_camera_setup(rec_files_height);

			state = VIDEO_RECORDING;
			break;

		case VIDEO_RECORDING:
			record_h264_run(timestamp, rec_files_height, 8);

			state = DETECT_MODE_WARMUP;
			break;

		default:
			break;
		}
	}
}
