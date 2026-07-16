#ifndef APP_UART_H
#define APP_UART_H
#include <stdint.h>
/* Sends one encoded JPEG over UART to the Python GUI. */
void send_jpeg_uart(const uint8_t *jpeg, int jpeg_len);
#endif /* APP_UART_H */
