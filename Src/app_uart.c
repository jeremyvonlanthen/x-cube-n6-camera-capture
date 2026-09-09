/**
 ******************************************************************************
 * @file    app_uart.c
 * @brief   UART transfer helpers (protocol with the Python GUI).
 ******************************************************************************
 */
#include "app_uart.h"
#include "app_shared.h"

#include "stm32n6xx_hal.h"

#define UART_TX_CHUNK_SIZE    32768

/* ==========================================================================
 * UART transfer helpers (protocol with the Python GUI)
 * ========================================================================== */

/* Sends one encoded JPEG over UART:
 *   0xAA | length (4 B, little endian) | JPEG data | exposure (4 B) | gain (4 B) */
void send_img_uart(const uint8_t *jpeg, int jpeg_len, int32_t exposure_us, int32_t gain_mdb)
{
	uart_busy = true;

  uint8_t sync = 0xAA;
  uint8_t size_buf[4];
  const uint8_t *src = jpeg;
  int remaining = jpeg_len;

  if (jpeg_len <= 0)
    return;

  HAL_UART_Transmit(&huart1, &sync, 1, HAL_MAX_DELAY);

  size_buf[0] = (jpeg_len >> 0) & 0xFF;
  size_buf[1] = (jpeg_len >> 8) & 0xFF;
  size_buf[2] = (jpeg_len >> 16) & 0xFF;
  size_buf[3] = (jpeg_len >> 24) & 0xFF;
  HAL_UART_Transmit(&huart1, size_buf, 4, HAL_MAX_DELAY);

  while (remaining > 0) {
    int chunk = remaining > UART_TX_CHUNK_SIZE ? UART_TX_CHUNK_SIZE : remaining;
    HAL_UART_Transmit(&huart1, (uint8_t *)src, chunk, HAL_MAX_DELAY);
    src += chunk;
    remaining -= chunk;
  }

  HAL_UART_Transmit(&huart1, (uint8_t *)&exposure_us, sizeof(exposure_us), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t *)&gain_mdb, sizeof(gain_mdb), HAL_MAX_DELAY);

  uart_busy = false;
}

