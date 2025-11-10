 /**
 ******************************************************************************
 * @file    app_postprocess.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_POSTPROCESS_H
#define __APP_POSTPROCESS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "app_config.h"
#include "postprocess_conf.h"
#include "objdetect_yolov2_pp_if.h"
#include "objdetect_yolov5_pp_if.h"
#include "objdetect_yolov8_pp_if.h"
#include "objdetect_centernet_pp_if.h"
#include "objdetect_ssd_st_pp_if.h"
#include "objdetect_pp_output_if.h"

#define POSTPROCESS_YOLO_V2       (1) /* Yolov2 postprocessing; Input model: uint8; output: float32 */
#define POSTPROCESS_YOLO_V5_UU    (2) /* Yolov5 postprocessing; Input model: uint8; output: uint8   */
#define POSTPROCESS_YOLO_V8_UF    (3) /* Yolov8 postprocessing; Input model: uint8; output: float32 */
#define POSTPROCESS_YOLO_V8_UI    (4) /* Yolov8 postprocessing; Input model: uint8; output: int8    */

/* Exported functions ------------------------------------------------------- */
int32_t app_postprocess_init( void* static_params_postprocess);
int32_t app_postprocess_run( void *pInput, postprocess_out_t*pOutput, void *pInput_static_param);

#ifdef __cplusplus
}
#endif

#endif /*__APP_POSTPROCESS_H */
