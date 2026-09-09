#ifndef APP_UART_H
#define APP_UART_H
#include <stdint.h>
/* Sends one encoded JPEG over UART to the Python GUI, followed by the
 * exposure/gain the caller already read (avoids a redundant
 * CMW_CAMERA_GetExposure/GetGain call when they're read once upstream). */
void send_img_uart(const uint8_t *jpeg, int jpeg_len, int32_t exposure_us, int32_t gain_mdb);
#endif /* APP_UART_H */
