/**
 ******************************************************************************
 * @file    app_flash_config.c
 * @brief   Persists Config_t to a fixed address in the external NOR flash
 *          (BSP_XSPI_NOR_*, already initialized and memory-mapped by main.c
 *          at boot -- nothing else in this project uses that flash).
 *
 * The config is stored in the LAST 4 KB sector of the chip (queried at
 * runtime via BSP_XSPI_NOR_GetInfo, so this works unmodified on both the
 * DK's 128 MB mx66uw1g45g and the Nucleo's 64 MB mx25um51245g) -- far from
 * any boot image at the start of the flash.
 ******************************************************************************
 */
#include "app_flash_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#ifdef STM32N6570_DK_REV
#include "stm32n6570_discovery_xspi.h"
#else
#include "stm32n6xx_nucleo_xspi.h"
#endif

#define FLASH_INSTANCE           0u
#define CONFIG_FLASH_SECTOR_SIZE 4096u

/* Config_t is packed to 29 bytes (odd). The XSPI NOR write/read data phase
 * runs in DTR (double transfer rate, 2 bytes/clock) mode -- an odd byte
 * count there made BSP_XSPI_NOR_Write fail with BSP_ERROR_COMPONENT_FAILURE
 * (confirmed on hardware: Erase_Block succeeds, Write doesn't). Round the
 * on-flash I/O size up to the next even byte and go through a padded
 * scratch buffer instead of writing/reading Config_t directly. */
#define CONFIG_FLASH_IO_SIZE (((uint32_t)sizeof(Config_t) + 1u) & ~1u)

/* Address of the reserved config sector; resolved on first use (flash size
 * differs between the DK and Nucleo NOR chips). */
static uint32_t config_flash_addr(void)
{
  static uint32_t addr;
  static bool     resolved = false;

  if (!resolved) {
    BSP_XSPI_NOR_Info_t info = { 0 };
    int32_t st = BSP_XSPI_NOR_GetInfo(FLASH_INSTANCE, &info);

    if (st == BSP_ERROR_NONE && info.FlashSize > 0) {
      addr = info.FlashSize - CONFIG_FLASH_SECTOR_SIZE;
    } else {
      /* Leave addr at 0 (its zero-init value) so the caller can tell this
       * failed -- but 0 is also the very start of the flash (boot image),
       * so callers must treat "addr == 0" as "not resolved", never as a
       * valid target. */
      printf("[FLASH] GetInfo failed (st=%ld, FlashSize=%lu)\r\n",
             (long)st, (unsigned long)info.FlashSize);
    }
    resolved = true;
  }
  return addr;
}

int CONFIG_FLASH_Save(const Config_t *cfg)
{
  uint32_t addr = config_flash_addr();
  int32_t st;
  int ret = -1;

  if (addr == 0u) {
    printf("[FLASH] save: address not resolved, aborting\r\n");
    return -1;
  }

  st = BSP_XSPI_NOR_DisableMemoryMappedMode(FLASH_INSTANCE);
  if (st != BSP_ERROR_NONE) {
    printf("[FLASH] DisableMemoryMappedMode failed (st=%ld)\r\n", (long)st);
    goto out;
  }

  /* BSP_XSPI_NOR_ERASE_4K (an enum value, not BSP_XSPI_NOR_BLOCK_4K -- the
   * latter is a byte-size macro that references an undefined symbol in the
   * vendor header, MX66UW1G45G_SUBSECTOR_4K instead of the actual
   * MX66UW1G45G_BLOCK_4K). CONFIG_FLASH_SECTOR_SIZE above already hardcodes
   * the erased size in bytes (4096) for the address arithmetic. */
  st = BSP_XSPI_NOR_Erase_Block(FLASH_INSTANCE, addr, BSP_XSPI_NOR_ERASE_4K);
  if (st != BSP_ERROR_NONE) {
    printf("[FLASH] Erase_Block failed (st=%ld, addr=0x%08lX)\r\n", (long)st, (unsigned long)addr);
    goto reenable;
  }

  {
    uint8_t io_buf[CONFIG_FLASH_IO_SIZE] = { 0 };

    memcpy(io_buf, cfg, sizeof(*cfg));
    st = BSP_XSPI_NOR_Write(FLASH_INSTANCE, io_buf, addr, sizeof(io_buf));
  }
  if (st != BSP_ERROR_NONE) {
    printf("[FLASH] Write failed (st=%ld, addr=0x%08lX)\r\n", (long)st, (unsigned long)addr);
    goto reenable;
  }

  ret = 0;

reenable:
  st = BSP_XSPI_NOR_EnableMemoryMappedMode(FLASH_INSTANCE);
  if (st != BSP_ERROR_NONE) {
    printf("[FLASH] EnableMemoryMappedMode failed (st=%ld)%s\r\n", (long)st,
           ret == 0 ? " -- config WAS written, but XIP re-enable failed" : "");
    ret = -1;
  }
out:
  return ret;
}

int CONFIG_FLASH_Load(Config_t *cfg)
{
  uint32_t addr = config_flash_addr();
  Config_t tmp = { 0 };
  int32_t st;
  int ret = -1;

  if (addr == 0u) {
    printf("[FLASH] load: address not resolved, aborting\r\n");
    return -1;
  }

  st = BSP_XSPI_NOR_DisableMemoryMappedMode(FLASH_INSTANCE);
  if (st != BSP_ERROR_NONE) {
    printf("[FLASH] DisableMemoryMappedMode failed (st=%ld)\r\n", (long)st);
    goto out;
  }

  {
    uint8_t io_buf[CONFIG_FLASH_IO_SIZE];

    st = BSP_XSPI_NOR_Read(FLASH_INSTANCE, io_buf, addr, sizeof(io_buf));
    if (st == BSP_ERROR_NONE)
      memcpy(&tmp, io_buf, sizeof(tmp));
  }
  if (st != BSP_ERROR_NONE) {
    printf("[FLASH] Read failed (st=%ld, addr=0x%08lX)\r\n", (long)st, (unsigned long)addr);
  } else if (tmp.magic != CONFIG_MAGIC) {
    printf("[FLASH] Read ok but magic mismatch (0x%08lX), sector never saved\r\n",
           (unsigned long)tmp.magic);
  } else {
    memcpy(cfg, &tmp, sizeof(*cfg));
    ret = 0;
  }

  st = BSP_XSPI_NOR_EnableMemoryMappedMode(FLASH_INSTANCE);
  if (st != BSP_ERROR_NONE) {
    printf("[FLASH] EnableMemoryMappedMode failed (st=%ld)\r\n", (long)st);
    ret = -1;
  }
out:
  return ret;
}
