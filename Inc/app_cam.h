 /**
 ******************************************************************************
 * @file    app_cam.h
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#ifndef APP_CAM
#define APP_CAM

#include <stdint.h>

typedef struct {
  int capture_width;
  int capture_height;
  int fps;
  int dcmipp_output_format;
  int is_rgb_swap;
} CAM_conf_t;

void CAM_Init(CAM_conf_t *conf, uint8_t two_pipes);
void CAM_CapturePipe_Start(uint8_t *capture_pipe_dst_pipe1, uint8_t *capture_pipe_dst_pipe2, uint32_t cam_mode, uint8_t two_pipes);
void CAM_IspUpdate(void);
void CAM_Deinit(void);

#endif
