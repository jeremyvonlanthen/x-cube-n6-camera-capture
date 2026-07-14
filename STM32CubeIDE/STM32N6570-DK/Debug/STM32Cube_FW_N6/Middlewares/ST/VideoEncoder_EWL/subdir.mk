################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL/ewl_impl.c 

OBJS += \
./STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL/ewl_impl.o 

C_DEPS += \
./STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL/ewl_impl.d 


# Each subdirectory must supply rules for building sources it contributes
STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL/ewl_impl.o: C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL/ewl_impl.c STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DSTM32N657xx -DUSE_FULL_ASSERT -DUSE_FULL_LL_DRIVER -DVECT_TAB_SRAM -DUSE_IMX335_SENSOR -DUSE_VD66GY_SENSOR -DUSE_VD55G1_SENSOR -DUSE_VD1943_SENSOR -DCAMERA_SELFY=1 -DSTM32N6570_DK_REV -DTX_MAX_PARALLEL_NETWORKS=1 -DFEAT_FREERTOS -c -I../../../Inc -I"C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/STM32CubeIDE/STM32N6570-DK/Src" -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/DSP/Include -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/Common -I../../../STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx -I../../../Lib/Camera_Middleware -I../../../Lib/Camera_Middleware/sensors -I../../../Lib/Camera_Middleware/sensors/imx335 -I../../../Lib/Camera_Middleware/sensors/vd55g1 -I../../../Lib/Camera_Middleware/sensors/vd6g -I../../../Lib/Camera_Middleware/sensors/vd1943 -I../../../Lib/Camera_Middleware/ISP_Library/isp/Inc -I../../../Lib/FreeRTOS/Source/include -I../../../Lib/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure -I../../../STM32Cube_FW_N6/Middlewares/Third_Party/VideoEncoder/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL -I../../../STM32Cube_FW_N6/Middlewares/Third_Party/VideoEncoder/source/common -I../../../STM32Cube_FW_N6/Middlewares/Third_Party/VideoEncoder/source/h264 -I../../../Lib/FreeRTOS/Source/CMSIS_RTOS_V2 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STM32Cube_FW_N6-2f-Middlewares-2f-ST-2f-VideoEncoder_EWL

clean-STM32Cube_FW_N6-2f-Middlewares-2f-ST-2f-VideoEncoder_EWL:
	-$(RM) ./STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL/ewl_impl.cyclo ./STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL/ewl_impl.d ./STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL/ewl_impl.o ./STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL/ewl_impl.su

.PHONY: clean-STM32Cube_FW_N6-2f-Middlewares-2f-ST-2f-VideoEncoder_EWL

