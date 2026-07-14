################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/aps256xx.c 

OBJS += \
./STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/aps256xx.o 

C_DEPS += \
./STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/aps256xx.d 


# Each subdirectory must supply rules for building sources it contributes
STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/aps256xx.o: C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/aps256xx.c STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DSTM32N657xx -DUX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT -DUX_DEVICE_STANDALONE -DUSE_FULL_ASSERT -DUSE_FULL_LL_DRIVER -DVECT_TAB_SRAM -DUSE_IMX335_SENSOR -DUSE_VD66GY_SENSOR -DUSE_VD55G1_SENSOR -DUSE_VD1943_SENSOR -DCAMERA_SELFY=1 -DSTM32N6570_DK_REV -DTX_MAX_PARALLEL_NETWORKS=1 -DFEAT_FREERTOS -DUVC_LIB_USE_USBX -DUX_INCLUDE_USER_DEFINE_FILE -DUSBL_PACKET_PER_MICRO_FRAME=3 -DUX_STANDALONE -DUVCL_USBX_USE_FREERTOS -DUVC_LIB_USE_DMA -c -I../../../Inc -I"C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/STM32CubeIDE/STM32N6570-DK/Src" -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/DSP/Include -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/Common -I../../../STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx -I../../../Lib/Camera_Middleware -I../../../Lib/Camera_Middleware/sensors -I../../../Lib/Camera_Middleware/sensors/imx335 -I../../../Lib/Camera_Middleware/sensors/vd55g1 -I../../../Lib/Camera_Middleware/sensors/vd6g -I../../../Lib/Camera_Middleware/sensors/vd1943 -I../../../Lib/Camera_Middleware/ISP_Library/isp/Inc -I../../../Lib/FreeRTOS/Source/include -I../../../Lib/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure -I../../../Lib/FatFs -I../../../Lib/minimp4 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STM32Cube_FW_N6-2f-Drivers-2f-BSP-2f-Components-2f-aps256xx

clean-STM32Cube_FW_N6-2f-Drivers-2f-BSP-2f-Components-2f-aps256xx:
	-$(RM) ./STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/aps256xx.cyclo ./STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/aps256xx.d ./STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/aps256xx.o ./STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/aps256xx.su

.PHONY: clean-STM32Cube_FW_N6-2f-Drivers-2f-BSP-2f-Components-2f-aps256xx

