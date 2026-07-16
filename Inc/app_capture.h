#ifndef APP_CAPTURE_H
#define APP_CAPTURE_H
#include <stdint.h>
/* (Re)initializes the camera at the given capture resolution (downscaled from
 * the sensor) and lets the AE/ISP converge. */
void camera_warmup(uint32_t cap_w, uint32_t cap_h, uint32_t output_format);
/* One full-resolution YUV422 snapshot, JPEG-encoded into hires_jpeg_buffer. */
int  capture_yuv(void);
#endif /* APP_CAPTURE_H */
