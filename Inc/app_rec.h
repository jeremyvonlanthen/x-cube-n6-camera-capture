/**
 ******************************************************************************
 * @file    app_rec.h
 * @brief   H264 -> MP4 recording to microSD (FAT32) using FreeRTOS.
 *
 * Data path:
 *   encoder task (app.c)          SD writer task (app_rec.c)
 *   ENC_EncodeFrame -> REC_PushFrame -> queue -> minimp4 -> FatFS f_write
 ******************************************************************************
 */
#ifndef APP_REC_H
#define APP_REC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* One-time peripheral init: initializes the SD recorder (REC_Init: SD card +
 * FAT32 mount + FreeRTOS SD writer task) and logs the failure reason on the
 * various error codes REC_Init can return.  Returns 1 on success, 0 on
 * failure (caller should retry after a short delay).
 *   quiet : true for a storage re-init (SD was just powered back up to
 *           flush a captured video/photo) -- collapses the mount/free-space
 *           detail into a single "re-init" line instead of repeating it. */
int SD_init(bool re_init, uint8_t consecutive_sd_init);

int SD_inserted(void);

/* Called once at startup (before any REC_Start).  Configures the SDMMC2
 * kernel clock, initializes the BSP SD, mounts the FAT32 volume and creates
 * the SD writer task.  Returns 0 on success.
 *   quiet : suppresses the FAT-type / free-space detail lines (see SD_init). */
int REC_Init(bool quiet);

/* Unmounts the FAT32 volume and powers down the SD peripheral (HAL SD
 * de-init, SDMMC2 clock, GPIOC/E pins) for current-consumption measurement.
 * VDDIO5 is deliberately left enabled -- it is a shared board I/O supply
 * (also used by the console UART pins), not an SD-only rail; see
 * SD_MspDeInit() in stm32n6570_discovery_sd.c. REC_Init() must be called
 * again before any further SD access. */
void SD_PowerDown(void);

/* Creates dirname on the FAT32 volume (e.g. a per-recording "<timestamp>/"
 * folder). Tolerates an already-existing directory. Called directly from the
 * caller thread, same as REC_Start's f_open -- no FreeRTOS queue involved.
 * Returns 0 on success. */
int REC_MakeDir(const char *dirname);

/* Opens a new VID_xxxx.MP4 on the card and starts the muxer.
 * ring_buf/ring_size: caller-provided PSRAM area used to buffer encoded
 * frames between the encoder task and the SD writer task (absorbs SD
 * write-latency pauses; bigger = fewer dropped frames).
 * Returns 0 on success. */
int REC_Start(int width, int height, int fps, uint8_t *ring_buf, size_t ring_size,
              const char *fname);

/* Queues one encoded H264 access unit (Annex-B, as produced by
 * ENC_EncodeFrame) for writing.  Copies the data into an internal PSRAM
 * slot; non-blocking.  Returns 0 on success, -1 if the queue is full
 * (frame dropped: caller should force an IDR on the next frame).
 * duration_90k: real measured duration of this frame in 1/90000 s units
 * (variable frame rate); pass 0 to use the nominal 1/fps duration. */
int REC_PushFrame(const uint8_t *p_data, size_t len, uint32_t duration_90k);

/* Flushes pending frames, finalizes the MP4 (writes the moov index) and
 * closes the file.  Blocks until done.  Returns 0 on success. */
int REC_Stop(void);

/* Writes an already-encoded buffer to a new file on the card (content-
 * agnostic -- JPEG, JSON, ...). The write is performed by the SD writer
 * task (FreeRTOS); this call blocks until the file is closed. Must not be
 * called while a video recording is active. Returns 0 on success. */
int REC_SaveFile(const uint8_t *p_data, size_t len, const char *fname);

#endif /* APP_REC_H */
