#ifndef APP_CAPTURE_H
#define APP_CAPTURE_H
#include <stdint.h>
#include <stdbool.h>

void camera_warmup(uint8_t warmup_frames_target, uint8_t warmup_fps, bool two_pipes);

int  capture_img(void);

#endif /* APP_CAPTURE_H */
