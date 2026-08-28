/**
 ******************************************************************************
 * @file    app_flash_config.h
 * @brief   Persists the GUI-provided DCMIPP pipes config (Config_t) to a
 *          fixed address in the external NOR flash, so it survives a power
 *          cycle and is available on autonomous DIURNE/24H boots (which skip
 *          the UART config phase entirely).
 ******************************************************************************
 */
#ifndef APP_FLASH_CONFIG_H
#define APP_FLASH_CONFIG_H

#include "app_shared.h"

/* Erases and writes *cfg to the reserved flash sector. Returns 0 on success,
 * -1 on a BSP/erase/write error. */
int CONFIG_FLASH_Save(const Config_t *cfg);

/* Reads the reserved flash sector into *cfg and checks its magic. Returns 0
 * if a valid config was found (magic matches, *cfg is updated), -1 otherwise
 * (BSP error, or the sector was never written -- *cfg is left untouched). */
int CONFIG_FLASH_Load(Config_t *cfg);

#endif /* APP_FLASH_CONFIG_H */
