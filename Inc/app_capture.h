#ifndef APP_CAPTURE_H
#define APP_CAPTURE_H
#include <stdint.h>
/* (Re)initializes the camera at full resolution and lets the AE/ISP converge. */
void camera_warmup(uint32_t output_format);
/* One full-resolution YUV422 snapshot, JPEG-encoded into hires_jpeg_buffer. */
int  capture_yuv(void);
#endif /* APP_CAPTURE_H */
