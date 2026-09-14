/**
 ******************************************************************************
 * @file    app_rec.c
 * @brief   H264 -> MP4 recording to microSD (FAT32) using FreeRTOS.
 *
 * Architecture (all FreeRTOS):
 *   - The encoder task (app.c) pushes encoded Annex-B access units with
 *     REC_PushFrame().  Frames are copied into fixed-size PSRAM slots and a
 *     descriptor is queued to the SD writer task.  The PSRAM ring absorbs
 *     FAT32 write-latency spikes (cluster allocation, FAT updates).
 *   - The SD writer task converts each access unit from Annex-B to MP4
 *     length-prefixed format (in a static PSRAM buffer, no heap), feeds
 *     SPS/PPS to the muxer once, and writes samples through minimp4 to the
 *     file via FatFS.
 *
 * minimp4 memory: routed to a static bump pool in PSRAM (MINIMP4_MALLOC
 * hooks).  The pool is reset at every REC_Start.  It only holds the mux
 * bookkeeping (sample index), NOT the video data, so 512 KB is plenty for
 * a 30 s / 900-frame recording.
 ******************************************************************************
 */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "app_rec.h"
#include "app_watchdog.h"
#include "utils.h"           /* IN_PSRAM, ALIGN_32 */
#include "stm32n6xx_hal.h"
#include "stm32n6570_discovery_sd.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ff.h"

/* ------------------------------------------------------------------------ */
/* minimp4 with custom allocator (static PSRAM pool, no libc heap)           */
/* ------------------------------------------------------------------------ */
static void *mp4_pool_alloc(size_t size);
static void *mp4_pool_realloc(void *ptr, size_t size);
static void  mp4_pool_free(void *ptr);

#define MINIMP4_MALLOC(s)      mp4_pool_alloc(s)
#define MINIMP4_REALLOC(p, s)  mp4_pool_realloc((p), (s))
#define MINIMP4_FREE(p)        mp4_pool_free(p)
#define MINIMP4_STRDUP(s)      NULL /* not used (no text comment) */

#define MINIMP4_IMPLEMENTATION
#define MINIMP4_TRANSCODE_SPS_ID 0  /* single encode session per file: SPS/PPS IDs are consistent */
#include "minimp4.h"

/* ------------------------------------------------------------------------ */
/* Configuration                                                             */
/* PSRAM extended to 32 MB (linker), so there is plenty of headroom now.     */
/* REC_MAX_FRAME must match H264_VENC_OUT_SIZE (largest encoded frame).      */
/* ------------------------------------------------------------------------ */
#define REC_MAX_FRAME       (1024 * 1024) /* = H264_VENC_OUT_SIZE (1080p keyframe) */
#define REC_MSG_NB          256           /* max frames in flight            */
#define REC_TASK_STACK_SIZE 4096          /* words -> 16 KB                  */
#define REC_TASK_PRIORITY   (tskIDLE_PRIORITY + 1)
#define REC_PREALLOC_BYTES  (64 * 1024 * 1024) /* f_expand hint, best effort */
#define MP4_POOL_SIZE       (128 * 1024)
#define MP4_TIMESCALE       90000u

/* ------------------------------------------------------------------------ */
/* Static buffers (PSRAM)                                                    */
/* ------------------------------------------------------------------------ */
/* Byte ring for encoded frames, provided by the caller at REC_Start()
 * (during recording it lives in the unused part of the camera frame
 * buffer: ~6 MB = several seconds of buffering).  Frames are stored
 * contiguously (no frame wraps around the end: if the tail gap is too
 * small it is wasted and the writer restarts at 0 — bounded by
 * REC_MAX_FRAME). */
static uint8_t *rec_ring;
static size_t   rec_ring_size;
static uint8_t mp4_pool[MP4_POOL_SIZE] ALIGN_32 IN_PSRAM;
/* Annex-B -> length-prefixed conversion buffer (one access unit) */
static uint8_t sample_buf[REC_MAX_FRAME + 1024] ALIGN_32 IN_PSRAM;

static size_t mp4_pool_off;
static size_t mp4_pool_peak;

/* Ring bookkeeping: single producer (encoder task) / single consumer (SD
 * writer task).  ring_free is the only shared counter, updated under a
 * critical section. */
static size_t ring_head;                 /* producer only */
static volatile size_t ring_free;

/* ------------------------------------------------------------------------ */
/* FreeRTOS objects (static allocation only in this project)                 */
/* ------------------------------------------------------------------------ */
typedef enum {
  REC_MSG_FRAME = 0,
  REC_MSG_STOP,
  REC_MSG_SAVE_FILE,       /* write an arbitrary buffer to a file (ptr/len) */
} rec_msg_type_t;

typedef struct {
  rec_msg_type_t type;
  uint32_t       offset;   /* frame start in rec_ring */
  uint32_t       len;      /* frame length (or file length) */
  uint32_t       waste;    /* wasted tail bytes to credit back on release */
  uint32_t       duration; /* frame duration in 1/90000 s (0 = nominal) */
  const uint8_t *ptr;      /* file data (REC_MSG_SAVE_FILE only) */
} rec_msg_t;

static StaticTask_t  rec_task_tcb;
static StackType_t   rec_task_stack[REC_TASK_STACK_SIZE];

static QueueHandle_t q_filled;
static StaticQueue_t q_filled_struct;
static uint8_t       q_filled_storage[REC_MSG_NB][sizeof(rec_msg_t)];

static SemaphoreHandle_t sem_stopped;
static StaticSemaphore_t sem_stopped_struct;

/* ------------------------------------------------------------------------ */
/* microSD Vcc load switch (U28 on the board schematic), driven by PQ7       */
/* ------------------------------------------------------------------------ */
#define SD_PWR_GPIO_PORT  GPIOQ
#define SD_PWR_GPIO_PIN   GPIO_PIN_7

/* Enables the SD card's own power supply (U28). Idempotent: safe to call on
 * every REC_Init(), including the very first boot. Assumes an active-high
 * enable (SET = powered) -- if SD_PowerDown() turns out not to reduce
 * consumption once this is wired in, the polarity is inverted: swap SET/RESET
 * below and in SD_PowerDown(). */
static void SD_PowerRail_Init(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  __HAL_RCC_GPIOQ_CLK_ENABLE();

  gpio_init.Pin   = SD_PWR_GPIO_PIN;
  gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull  = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_PWR_GPIO_PORT, &gpio_init);

  HAL_GPIO_WritePin(SD_PWR_GPIO_PORT, SD_PWR_GPIO_PIN, GPIO_PIN_SET);
  HAL_Delay(5); /* let U28's output ramp up before the card is addressed */
}

/* ------------------------------------------------------------------------ */
/* Recorder state (owned by the SD writer task once started)                 */
/* ------------------------------------------------------------------------ */
static FATFS fs;
static FIL   fil;
static MP4E_mux_t *mux;
static int   mux_track;
static int   sps_done, pps_done;
static unsigned frame_duration;   /* in MP4_TIMESCALE units */
static bool  rec_error;
static volatile bool rec_active;

static volatile int save_file_result;  /* REC_MSG_SAVE_FILE outcome (SD task -> caller) */


/* ------------------------------------------------------------------------ */
/* minimp4 pool allocator: bump allocator with realloc-copy semantics.       */
/* free() is a no-op; the whole pool is reset at REC_Start().  Each block    */
/* stores its size in an 8-byte header so realloc can copy the old data.     */
/* ------------------------------------------------------------------------ */
static void *mp4_pool_alloc(size_t size)
{
  size_t aligned = (size + 7u) & ~(size_t)7u;

  if (mp4_pool_off + aligned + 8u > MP4_POOL_SIZE) {
    printf("[REC] mp4 pool exhausted (%u used)\r\n", (unsigned)mp4_pool_off);
    return NULL;
  }
  uint8_t *p = &mp4_pool[mp4_pool_off];
  *(size_t *)(void *)p = size;
  mp4_pool_off += aligned + 8u;
  if (mp4_pool_off > mp4_pool_peak)
    mp4_pool_peak = mp4_pool_off;
  return p + 8;
}

static void *mp4_pool_realloc(void *ptr, size_t size)
{
  void *p_new;
  size_t old_size;

  if (ptr == NULL)
    return mp4_pool_alloc(size);

  old_size = *(size_t *)(void *)((uint8_t *)ptr - 8);
  p_new = mp4_pool_alloc(size);
  if (p_new == NULL)
    return NULL;
  memcpy(p_new, ptr, old_size < size ? old_size : size);
  return p_new;
}

static void mp4_pool_free(void *ptr)
{
  (void)ptr; /* bump allocator: no-op, pool reset at REC_Start */
}

static void mp4_pool_reset(void)
{
  mp4_pool_off = 0;
}

/* ------------------------------------------------------------------------ */
/* minimp4 write callback -> FatFS                                           */
/* ------------------------------------------------------------------------ */
static int mp4_write_cb(int64_t offset, const void *buffer, size_t size, void *token)
{
  FIL *f = (FIL *)token;
  UINT bw = 0;
  FRESULT res;

  if (f_tell(f) != (FSIZE_t)offset) {
    res = f_lseek(f, (FSIZE_t)offset);
    if (res != FR_OK) {
      printf("[REC] f_lseek(%lu) err=%d\r\n", (unsigned long)offset, res);
      return 1;
    }
  }
  res = f_write(f, buffer, (UINT)size, &bw);
  if (res != FR_OK || bw != (UINT)size) {
    printf("[REC] f_write err=%d bw=%u/%u\r\n", res, bw, (unsigned)size);
    return 1;
  }
  return 0;
}

/* ------------------------------------------------------------------------ */
/* Annex-B parsing                                                           */
/* ------------------------------------------------------------------------ */
/* Returns pointer to the first byte AFTER the next start code (00 00 01 or
 * 00 00 00 01) in [p, end), or NULL if none. */
static const uint8_t *next_start_code(const uint8_t *p, const uint8_t *end)
{
  while (p + 3 <= end) {
    if (p[0] == 0 && p[1] == 0 && p[2] == 1)
      return p + 3;
    p++;
  }
  return NULL;
}

/* Converts one Annex-B access unit to a length-prefixed MP4 sample and
 * pushes it to the muxer.  SPS (7) / PPS (8) are registered once; AUD (9)
 * and filler (12) NALs are discarded.  Returns 0 on success. */
static int rec_write_access_unit(const uint8_t *p_data, uint32_t len, uint32_t duration_90k)
{
  const uint8_t *end = p_data + len;
  const uint8_t *nal = next_start_code(p_data, end);
  size_t sample_len = 0;
  int is_idr = 0;
  int err;

  while (nal != NULL && nal < end) {
    const uint8_t *next = next_start_code(nal, end);
    /* NAL payload spans [nal, nal_end) where nal_end excludes the next
     * start code (and its possible 4th zero byte) */
    const uint8_t *nal_end = (next != NULL) ? next - 3 : end;
    while (nal_end > nal && nal_end[-1] == 0)  /* strip trailing_zero_8bits */
      nal_end--;

    if (nal_end > nal) {
      size_t nal_len = (size_t)(nal_end - nal);
      int nal_type = nal[0] & 0x1F;

      switch (nal_type) {
      case 7: /* SPS */
        if (!sps_done) {
          err = MP4E_set_sps(mux, mux_track, nal, (int)nal_len);
          if (err != MP4E_STATUS_OK)
            return -1;
          sps_done = 1;
        }
        break;
      case 8: /* PPS */
        if (!pps_done) {
          err = MP4E_set_pps(mux, mux_track, nal, (int)nal_len);
          if (err != MP4E_STATUS_OK)
            return -1;
          pps_done = 1;
        }
        break;
      case 9:  /* access unit delimiter */
      case 12: /* filler data (padding NAL appended by app_enc.c) */
        break;
      case 5: /* IDR slice */
        is_idr = 1;
        /* fallthrough */
      default: /* VCL slices, SEI, ... -> part of the sample */
        if (sample_len + 4 + nal_len > sizeof(sample_buf)) {
          printf("[REC] sample_buf overflow\r\n");
          return -1;
        }
        sample_buf[sample_len + 0] = (uint8_t)(nal_len >> 24);
        sample_buf[sample_len + 1] = (uint8_t)(nal_len >> 16);
        sample_buf[sample_len + 2] = (uint8_t)(nal_len >> 8);
        sample_buf[sample_len + 3] = (uint8_t)(nal_len >> 0);
        memcpy(&sample_buf[sample_len + 4], nal, nal_len);
        sample_len += 4 + nal_len;
        break;
      }
    }
    nal = next;
  }

  if (sample_len == 0)
    return 0; /* nothing to write (e.g. SPS/PPS-only unit) */

  if (!sps_done || !pps_done) {
    printf("[REC] slice before SPS/PPS, dropped\r\n");
    return 0;
  }

  err = MP4E_put_sample(mux, mux_track, sample_buf, (int)sample_len,
                        (duration_90k != 0u) ? (int)duration_90k : (int)frame_duration,
                        is_idr ? MP4E_SAMPLE_RANDOM_ACCESS : MP4E_SAMPLE_DEFAULT);
  if (err != MP4E_STATUS_OK) {
    printf("[REC] MP4E_put_sample err=%d\r\n", err);
    return -1;
  }
  return 0;
}

/* Filename for the next REC_SaveFile write, set by REC_SaveFile and consumed
 * by the SD writer task in rec_write_file (saving is sequential and blocks
 * the caller, so a single shared buffer is safe). */
static char rec_save_fname[48] = "IMG_0001.JPG";
/* File name of the recording currently open, logged when it is finalized */
static char rec_mp4_fname[48]  = "VID_0001.MP4";

/* Labels rec_write_file()'s log line by extension, so the picture/data-file
 * saves (both routed through the same REC_SaveFile/rec_write_file path) read
 * distinctly in the console log instead of two identical "file saved to". */
static const char *rec_file_kind(const char *fname)
{
  size_t len = strlen(fname);

  if (len >= 5 && strcmp(fname + len - 5, ".jpeg") == 0)
    return "picture";
  if (len >= 4 && strcmp(fname + len - 4, ".jpg") == 0)
    return "picture";
  if (len >= 5 && strcmp(fname + len - 5, ".json") == 0)
    return "data file";
  return "file";
}

/* Writes p_data to the file named rec_save_fname (SD writer task context). */
static int rec_write_file(const uint8_t *p_data, uint32_t len)
{
  FRESULT res;
  FIL jf;
  UINT bw = 0;
  char fname[48];

  strncpy(fname, rec_save_fname, sizeof(fname) - 1);
  fname[sizeof(fname) - 1] = '\0';

  res = f_open(&jf, fname, FA_WRITE | FA_CREATE_NEW);
  if (res != FR_OK) {
    printf("[REC] f_open('%s') failed (%d)\r\n", fname, res);
    return -1;
  }

  res = f_write(&jf, p_data, (UINT)len, &bw);
  f_close(&jf);
  if (res != FR_OK || bw != (UINT)len) {
    printf("[REC] f_write('%s') err=%d bw=%u/%lu\r\n", fname, res, bw, (unsigned long)len);
    return -1;
  }

  printf("[REC] %s saved to %s (%.1f KB)\r\n", rec_file_kind(fname), fname, (float)len / 1024.0f);
  return 0;
}

/* ------------------------------------------------------------------------ */
/* SD writer task                                                            */
/* ------------------------------------------------------------------------ */
static void rec_task_fct(void *arg)
{
  rec_msg_t msg;

  (void)arg;

  for (;;) {
    xQueueReceive(q_filled, &msg, portMAX_DELAY);

    watchdog_kick();

    if (msg.type == REC_MSG_FRAME) {
      /* Note: frames queued before a STOP are still written (FIFO order) */
      if (!rec_error && mux != NULL) {
        if (rec_write_access_unit(&rec_ring[msg.offset], msg.len, msg.duration) != 0)
          rec_error = true;
      }
      /* release the ring bytes (frame + wasted tail gap) */
      taskENTER_CRITICAL();
      ring_free += msg.len + msg.waste;
      taskEXIT_CRITICAL();
    }
    else if (msg.type == REC_MSG_SAVE_FILE) {
      save_file_result = rec_write_file(msg.ptr, msg.len);
      xSemaphoreGive(sem_stopped);  /* wakes up REC_SaveFile */
    }
    else { /* REC_MSG_STOP: finalize file */
      if (mux != NULL) {
        int err = MP4E_close(mux);
        if (err != MP4E_STATUS_OK) {
          printf("[REC] MP4E_close err=%d\r\n", err);
          rec_error = true;
        }
        mux = NULL;
      }
      f_truncate(&fil);  /* drop the f_expand preallocation tail */
      printf("[REC] video saved to %s (%.1f MB)\r\n",
             rec_mp4_fname, (float)f_size(&fil) / (1024.0f * 1024.0f));
      f_close(&fil);
      xSemaphoreGive(sem_stopped);
    }
  }
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */
int SD_init(bool re_init, uint8_t consecutive_sd_init)
{
  int rec_ready = REC_Init(re_init);

  switch (rec_ready) {
  case 0:
    if(re_init)
      printf("[uSD] uSD successfully re-init\r\n");
    break;
  case -1:
    printf("[uSD] required formatting failed (FAT32)\r\n");
    break;
  case -2:
  	if(!re_init)
  		printf("[uSD] no uSD card detected/mounted (%d. times -- retry in 2 sec)\r\n", consecutive_sd_init);
  	else
  		printf("[uSD] corrupted uSD, re-init not possible  -- raised error\r\n");
    break;
  case -3:
    printf("[uSD] SDMMC2 clock config failed\r\n");
    break;
  default:
    break;
  }

  return (rec_ready == 0);
}

int SD_inserted(void)
{
	return (BSP_SD_IsDetected(0) != SD_PRESENT);
}

/* Tracks whether SD_PowerDown() was the last thing done to hsd_sdmmc[0]:
 * it already runs BSP_SD_DeInit() (HAL_SD_DeInit + SDMMC2 clock gated off),
 * so REC_Init()'s own hot-removal HAL_SD_DeInit() call below must be
 * skipped in that case -- calling it again with the SDMMC2 kernel clock
 * disabled hangs forever (confirmed on hardware). Only a genuine
 * hot-removal (card pulled while the peripheral was still live, caught via
 * BSP_SD_IsDetected() in MOVEMENT_DETECTION) needs that reset. */
static volatile bool sd_was_cleanly_powered_down = false;

int REC_Init(bool quiet)
{
  static bool rtos_done = false;   /* FreeRTOS objects created only once */
  RCC_PeriphCLKInitTypeDef clk = { 0 };
  FRESULT res;
  int ret;

  SD_PowerRail_Init(); /* re-power the card if SD_PowerDown() cut it */

  /* SDMMC2 kernel clock: IC4 = PLL1 (800 MHz) / 4 = 200 MHz
   * (same 200 MHz kernel clock as the ST VENC_SDCard example). */
  clk.PeriphClockSelection = RCC_PERIPHCLK_SDMMC2;
  clk.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_IC4;
  clk.ICSelection[RCC_IC4].ClockSelection = RCC_ICCLKSOURCE_PLL1;
  clk.ICSelection[RCC_IC4].ClockDivider = 4;
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK) return -3;

  /* BSP SD init (SDMMC2, 4-bit, high speed).  Handles RIF config itself. */
  /* Recover from a hot-removal: reset the HAL SD handle so BSP_SD_Init
   * redoes a full, clean card identification.  A plain HAL_SD_DeInit is
   * enough here (BSP_SD_DeInit's fuller teardown -- clocks/GPIO/VDDIO5 --
   * is unnecessary for this quick reset, see SD_PowerDown() for that).
   * Skipped on the very first boot (handle not yet initialized) AND after a
   * clean SD_PowerDown() (which already ran HAL_SD_DeInit itself, with the
   * SDMMC2 kernel clock still enabled at the time -- doing it again here
   * once that clock is gated off hangs forever). */
  if (hsd_sdmmc[0].Instance != NULL && !sd_was_cleanly_powered_down) {
    HAL_SD_DeInit(&hsd_sdmmc[0]);
  }
  ret = BSP_SD_Init(0);
  if (ret != BSP_ERROR_NONE) return -2;

  sd_was_cleanly_powered_down = false; /* peripheral is live again */

  /* Mount the FAT32 volume (immediate mount to fail early) */
  res = f_mount(&fs, "", 1);
  if (res != FR_OK) return -1;

  if (!quiet) {
    printf("[uSD] uSD mounted and detected (FAT type %d)\r\n", fs.fs_type);

    /* Total and free space (f_getfree scans the FAT: can take 100s of ms).
     * 512 bytes/sector => 2048 sectors = 1 MB. */
    FATFS *pfs;
    DWORD  fre_clust;
    if (f_getfree("", &fre_clust, &pfs) == FR_OK) {
      uint64_t tot_sect = (uint64_t)(pfs->n_fatent - 2) * pfs->csize;
      uint64_t fre_sect = (uint64_t)fre_clust * pfs->csize;
      printf("[uSD] total %lu MB, free %lu MB (%d %%)\r\n",
             (unsigned long)(tot_sect / 2048u), (unsigned long)(fre_sect / 2048u),
             (unsigned int)(fre_sect*100/tot_sect));
    }
  }

  /* FreeRTOS objects: created ONCE.  On a re-mount (hot-removal / USER1
   * restart) we must NOT recreate the queue/semaphore/writer task on the
   * same static storage — that corrupts FreeRTOS and HardFaults on the
   * next SD write. */
  if (!rtos_done) {
    q_filled = xQueueCreateStatic(REC_MSG_NB, sizeof(rec_msg_t),
                                  &q_filled_storage[0][0], &q_filled_struct);
    sem_stopped = xSemaphoreCreateBinaryStatic(&sem_stopped_struct);

    xTaskCreateStatic(rec_task_fct, "sd_rec", REC_TASK_STACK_SIZE, NULL,
                      REC_TASK_PRIORITY, rec_task_stack, &rec_task_tcb);
    rtos_done = true;
  }

  return 0;
}

void SD_PowerDown(void)
{
  f_mount(NULL, "", 0);   /* unmount cleanly before pulling the rug */
  BSP_SD_DeInit(0);       /* HAL SD de-init + SDMMC2 clock/GPIO off */
  HAL_GPIO_WritePin(SD_PWR_GPIO_PORT, SD_PWR_GPIO_PIN, GPIO_PIN_RESET); /* U28 off: card actually unpowered */
  sd_was_cleanly_powered_down = true; /* skip REC_Init()'s redundant HAL_SD_DeInit next time */
  printf("[uSD] SD powered down\r\n");
}

/* Creates dirname on the FAT32 volume, tolerating FR_EXIST. */
int REC_MakeDir(const char *dirname)
{
  FRESULT res = f_mkdir(dirname);

  if (res != FR_OK && res != FR_EXIST) {
    printf("[REC] f_mkdir('%s') failed (%d)\r\n", dirname, res);
    return -1;
  }
  return 0;
}

int REC_Start(int width, int height, int fps, uint8_t *ring_buf, size_t ring_size,
              const char *fname)
{
  MP4E_track_t track = { 0 };
  FRESULT res;
  char name[48];

  if (rec_active || ring_buf == NULL || ring_size < 2u * REC_MAX_FRAME)
    return -1;

  rec_ring      = ring_buf;
  rec_ring_size = ring_size;

  mp4_pool_reset();
  sps_done = 0;
  pps_done = 0;
  rec_error = false;
  frame_duration = MP4_TIMESCALE / (unsigned)fps;

  /* Ring is empty here: the previous recording (if any) was fully drained
   * by REC_Stop before this function can run again. */
  ring_head = 0;
  ring_free = rec_ring_size;

  /* Use the caller-provided filename (timestamp), or a default fallback. */
  if (fname != NULL && fname[0] != '\0') {
    strncpy(name, fname, sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
  } else {
    strncpy(name, "VID_0001.MP4", sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
  }
  res = f_open(&fil, name, FA_WRITE | FA_CREATE_NEW);
  if (res != FR_OK) {
    printf("[REC] f_open('%s') failed (%d)\r\n", name, res);
    return -1;
  }
  /* Remember the name so REC_Stop can log it once the file is finalized */
  strncpy(rec_mp4_fname, name, sizeof(rec_mp4_fname) - 1);
  rec_mp4_fname[sizeof(rec_mp4_fname) - 1] = '\0';

  /* Note: no f_expand() preallocation here — scanning the FAT of a large
   * card takes several hundred ms (startup delay), and the large PSRAM
   * ring already absorbs FAT32 allocation latency spikes. */

  mux = MP4E_open(0 /* seekable output */, 0 /* no fragmentation */,
                  &fil, mp4_write_cb);
  if (mux == NULL) {
    f_close(&fil);
    printf("[REC] MP4E_open failed\r\n");
    return -1;
  }

  track.object_type_indication = MP4_OBJECT_TYPE_AVC;
  track.language[0] = 'u'; track.language[1] = 'n'; track.language[2] = 'd';
  track.track_media_kind = e_video;
  track.time_scale = MP4_TIMESCALE;
  track.default_duration = frame_duration;
  track.u.v.width = width;
  track.u.v.height = height;
  mux_track = MP4E_add_track(mux, &track);
  if (mux_track < 0) {
    printf("[REC] MP4E_add_track err=%d\r\n", mux_track);
    f_close(&fil);
    mux = NULL;
    return -1;
  }

  rec_active = true;
  return 0;
}

int REC_PushFrame(const uint8_t *p_data, size_t len, uint32_t duration_90k)
{
  rec_msg_t msg;
  size_t waste = 0;
  size_t need;

  if (!rec_active || len == 0)
    return -1;

  if (len > REC_MAX_FRAME) {
    printf("[REC] frame too big (%u), dropped\r\n", (unsigned)len);
    return -1;
  }

  /* Frames are stored contiguously: if the gap at the end of the ring is
   * too small, waste it and restart at offset 0. */
  if (ring_head + len > rec_ring_size)
    waste = rec_ring_size - ring_head;
  need = len + waste;

  /* Reserve space (SD writer may credit ring_free concurrently) */
  taskENTER_CRITICAL();
  if (ring_free < need) {
    taskEXIT_CRITICAL();
    return -1;
  }
  ring_free -= need;
  taskEXIT_CRITICAL();

  if (waste > 0)
    ring_head = 0;

  memcpy(&rec_ring[ring_head], p_data, len);

  msg.type   = REC_MSG_FRAME;
  msg.offset = (uint32_t)ring_head;
  msg.len    = (uint32_t)len;
  msg.waste  = (uint32_t)waste;
  msg.duration = duration_90k;

  ring_head += len;

  if (xQueueSend(q_filled, &msg, 0) != pdTRUE) {
    /* queue full (many tiny frames): roll back the reservation */
    taskENTER_CRITICAL();
    ring_free += need;
    taskEXIT_CRITICAL();
    ring_head = (waste > 0) ? rec_ring_size - waste : ring_head - len;
    return -1;
  }

  return 0;
}

int REC_Stop(void)
{
  rec_msg_t msg;

  if (!rec_active)
    return -1;

  rec_active = false;

  msg.type   = REC_MSG_STOP;
  msg.offset = 0;
  msg.len    = 0;
  msg.waste  = 0;
  msg.duration = 0;
  xQueueSend(q_filled, &msg, portMAX_DELAY);

  /* Wait for the SD writer task to drain the queue and finalize the MP4 */
  xSemaphoreTake(sem_stopped, portMAX_DELAY);

  return rec_error ? -1 : 0;
}

int REC_SaveFile(const uint8_t *p_data, size_t len, const char *fname)
{
  rec_msg_t msg;

  if (rec_active || p_data == NULL || len == 0)
    return -1;

  /* Store the target filename for the SD writer task (rec_write_file).
   * Fall back to a default if none was provided. */
  if (fname != NULL && fname[0] != '\0') {
    strncpy(rec_save_fname, fname, sizeof(rec_save_fname) - 1);
    rec_save_fname[sizeof(rec_save_fname) - 1] = '\0';
  }

  msg.type     = REC_MSG_SAVE_FILE;
  msg.offset   = 0;
  msg.len      = (uint32_t)len;
  msg.waste    = 0;
  msg.duration = 0;
  msg.ptr      = p_data;
  xQueueSend(q_filled, &msg, portMAX_DELAY);

  /* The SD writer task performs the write; block until the file is closed
   * (sem_stopped doubles as a generic completion semaphore: file saving
   * and video stop never overlap, both are driven by the same caller). */
  xSemaphoreTake(sem_stopped, portMAX_DELAY);

  return save_file_result;
}
