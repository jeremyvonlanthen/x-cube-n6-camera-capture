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

#define UART_TX_TIMEOUT_MS    200

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

  if (jpeg_len <= 0) {
    uart_busy = false;
    return;
  }

  if (HAL_UART_Transmit(&huart1, &sync, 1, UART_TX_TIMEOUT_MS) != HAL_OK)
    goto out;

  size_buf[0] = (jpeg_len >> 0) & 0xFF;
  size_buf[1] = (jpeg_len >> 8) & 0xFF;
  size_buf[2] = (jpeg_len >> 16) & 0xFF;
  size_buf[3] = (jpeg_len >> 24) & 0xFF;
  if (HAL_UART_Transmit(&huart1, size_buf, 4, UART_TX_TIMEOUT_MS) != HAL_OK)
    goto out;

  while (remaining > 0) {
    int chunk = remaining > UART_TX_CHUNK_SIZE ? UART_TX_CHUNK_SIZE : remaining;
    if (HAL_UART_Transmit(&huart1, (uint8_t *)src, chunk, UART_TX_TIMEOUT_MS) != HAL_OK)
      goto out;
    src += chunk;
    remaining -= chunk;
  }

  if (HAL_UART_Transmit(&huart1, (uint8_t *)&exposure_us, sizeof(exposure_us), UART_TX_TIMEOUT_MS) != HAL_OK)
    goto out;
  HAL_UART_Transmit(&huart1, (uint8_t *)&gain_mdb, sizeof(gain_mdb), UART_TX_TIMEOUT_MS);

out:
  uart_busy = false;
}

