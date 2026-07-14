/**
 ******************************************************************************
 * @file    sd_diskio.c
 * @brief   FatFS diskio glue for STM32N6570-DK microSD (BSP_SD, SDMMC2).
 *
 * Transfers use the SDMMC internal DMA (IDMA) through a bounce buffer
 * placed in the non-cacheable internal SRAM region (.uncached_bss):
 *   - no D-cache maintenance needed (region is uncached),
 *   - the IDMA reads fast internal SRAM, immune to PSRAM bus contention.
 *
 * Rationale: CPU-polling writes sourced directly from PSRAM underrun the
 * SDMMC FIFO when DCMIPP + VENC saturate the PSRAM bandwidth during
 * recording (observed as FR_DISK_ERR after ~1 s of recording).
 *
 * A single FreeRTOS task (the SD writer task in app_rec.c) accesses the
 * filesystem, so no re-entrancy protection is needed (FF_FS_REENTRANT = 0).
 ******************************************************************************
 */

#include <string.h>

#include "ff.h"
#include "diskio.h"

#include "stm32n6570_discovery_sd.h"
#include "FreeRTOS.h"
#include "task.h"

#define SD_INSTANCE        0U
#define SD_READY_TIMEOUT   5000U  /* ms - cheap cards can pause >1 s (GC) */
#define SD_XFER_TIMEOUT    5000U  /* ms */

/* Bounce buffer in non-cacheable internal SRAM (MPU region set by main.c).
 * 16 KB = 32 sectors per DMA transfer. */
#define BOUNCE_SECTORS     32U
#define BOUNCE_SIZE        (BOUNCE_SECTORS * 512U)
static uint8_t sd_bounce[BOUNCE_SIZE] __attribute__((section(".uncached_bss"), aligned(32)));

static volatile DSTATUS sd_stat = STA_NOINIT;
static volatile int sd_write_done;
static volatile int sd_read_done;
static volatile int sd_abort;

/* BSP completion callbacks (called from BSP_SD_IRQHandler context).
 * Only volatile flags here - no FreeRTOS API from ISR. */
void BSP_SD_WriteCpltCallback(uint32_t Instance)
{
  (void)Instance;
  sd_write_done = 1;
}

void BSP_SD_ReadCpltCallback(uint32_t Instance)
{
  (void)Instance;
  sd_read_done = 1;
}

void BSP_SD_AbortCallback(uint32_t Instance)
{
  (void)Instance;
  sd_abort = 1;
}

static int sd_wait_flag(volatile int *flag)
{
  uint32_t start = HAL_GetTick();

  while (!*flag) {
    if (sd_abort)
      return -1;
    if (HAL_GetTick() - start >= SD_XFER_TIMEOUT)
      return -1;
    /* IRQ sets the flag; yield in the meantime */
    taskYIELD();
  }
  return 0;
}

static int sd_wait_ready(void)
{
  uint32_t start = HAL_GetTick();

  while (BSP_SD_GetCardState(SD_INSTANCE) != SD_TRANSFER_OK) {
    if (HAL_GetTick() - start >= SD_READY_TIMEOUT)
      return -1;
    /* let other tasks run while the card is programming */
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return 0;
}

DSTATUS disk_status(BYTE pdrv)
{
  (void)pdrv;
  return sd_stat;
}

DSTATUS disk_initialize(BYTE pdrv)
{
  (void)pdrv;

  /* BSP_SD_Init is done once at boot (REC_Init).  Here only check state. */
  if (BSP_SD_IsDetected(SD_INSTANCE) == SD_PRESENT)
    sd_stat = 0;
  else
    sd_stat = STA_NOINIT;

  return sd_stat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
  (void)pdrv;

  while (count > 0) {
    UINT chunk = (count > BOUNCE_SECTORS) ? BOUNCE_SECTORS : count;

    if (sd_wait_ready() != 0)
      return RES_ERROR;

    sd_read_done = 0;
    sd_abort = 0;
    if (BSP_SD_ReadBlocks_DMA(SD_INSTANCE, (uint32_t *)(void *)sd_bounce,
                              (uint32_t)sector, chunk) != BSP_ERROR_NONE)
      return RES_ERROR;
    if (sd_wait_flag(&sd_read_done) != 0)
      return RES_ERROR;

    memcpy(buff, sd_bounce, chunk * 512U);

    buff   += chunk * 512U;
    sector += chunk;
    count  -= chunk;
  }

  return RES_OK;
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
  (void)pdrv;

  while (count > 0) {
    UINT chunk = (count > BOUNCE_SECTORS) ? BOUNCE_SECTORS : count;

    /* CPU copy PSRAM -> uncached SRAM: tolerant to bus contention */
    memcpy(sd_bounce, buff, chunk * 512U);

    if (sd_wait_ready() != 0)
      return RES_ERROR;

    sd_write_done = 0;
    sd_abort = 0;
    if (BSP_SD_WriteBlocks_DMA(SD_INSTANCE, (uint32_t *)(void *)sd_bounce,
                               (uint32_t)sector, chunk) != BSP_ERROR_NONE)
      return RES_ERROR;
    if (sd_wait_flag(&sd_write_done) != 0)
      return RES_ERROR;

    buff   += chunk * 512U;
    sector += chunk;
    count  -= chunk;
  }

  return RES_OK;
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  BSP_SD_CardInfo card_info;

  (void)pdrv;

  switch (cmd) {
  case CTRL_SYNC:
    return (sd_wait_ready() == 0) ? RES_OK : RES_ERROR;

  case GET_SECTOR_COUNT:
    BSP_SD_GetCardInfo(SD_INSTANCE, &card_info);
    *(LBA_t *)buff = card_info.LogBlockNbr;
    return RES_OK;

  case GET_SECTOR_SIZE:
    BSP_SD_GetCardInfo(SD_INSTANCE, &card_info);
    *(WORD *)buff = (WORD)card_info.LogBlockSize;
    return RES_OK;

  case GET_BLOCK_SIZE:
    /* erase block size in units of sectors */
    *(DWORD *)buff = 1;
    return RES_OK;

  default:
    return RES_PARERR;
  }
}
