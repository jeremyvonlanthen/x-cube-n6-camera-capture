#ifndef APP_CAPTURE_H
#define APP_CAPTURE_H
#include <stdint.h>
/* (Re)initializes the camera at the given capture resolution (downscaled from
 * the sensor) and lets the AE/ISP converge.
 *   two_pipes : also configure pipe2 (needed before DETECT_MODE_WARMUP, whose
 *               dcmipp_apply_detect_config() then reconfigures pipe2's
 *               crop/decimation for the two detect channels). Must be passed
 *               explicitly -- it used to be inferred from output_format ==
 *               MONO_Y8_G8_1, but that broke once the config-mode warmup
 *               also switched to MONO (see CONFIG_MODE_WARMUP in app.c). */
void camera_warmup(uint32_t cap_w, uint32_t cap_h, uint32_t output_format, uint8_t two_pipes);
/* One full-resolution MONO snapshot, JPEG-encoded into hires_jpeg_buffer. */
int  capture_yuv(void);
#endif /* APP_CAPTURE_H */
