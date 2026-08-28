#ifndef APP_PIPES_H
#define APP_PIPES_H

#include <stdint.h>

/* Applies the crop/decimation/downsize config currently held in config_py to
 * DCMIPP pipe1 & pipe2. Does NOT touch flash: call CONFIG_FLASH_Load
 * (app_flash_config.h) from the FSM first if config_py needs refreshing
 * (e.g. on a DIURNE/24H boot, which skips the UART config phase). */
void dcmipp_apply_detect_config(void);

/* One snapshot on pipe1+pipe2 using whatever config is already programmed
 * (dcmipp_apply_detect_config) -- DCMIPP is NOT reconfigured. Returns 0 on
 * success, -1 on capture timeout. */
int capture_detect_frame(void);

/* Detect-mode pipe output dimensions (post crop/decimation/downsize), valid
 * after dcmipp_apply_detect_config(). Used by app_detect.c to size the
 * per-pixel statistics buffers. */
void dcmipp_get_detect_dims(uint16_t *width_pipe1, uint16_t *height_pipe1,
                             uint16_t *width_pipe2, uint16_t *height_pipe2);

/* Raw capture buffers filled by capture_detect_frame(), valid after
 * dcmipp_apply_detect_config(). Row stride is SENSOR_WIDTH for both. */
void dcmipp_get_capture_buffers(uint8_t **buffer_pipe1, uint8_t **buffer_pipe2);

#endif /* APP_PIPES_H */
