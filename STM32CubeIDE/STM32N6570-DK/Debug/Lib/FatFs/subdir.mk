################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/Lib/FatFs/ff.c \
C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/Lib/FatFs/ffunicode.c 

OBJS += \
./Lib/FatFs/ff.o \
./Lib/FatFs/ffunicode.o 

C_DEPS += \
./Lib/FatFs/ff.d \
./Lib/FatFs/ffunicode.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/FatFs/ff.o: C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/Lib/FatFs/ff.c Lib/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DSTM32N657xx -DUX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT -DUX_DEVICE_STANDALONE -DUSE_FULL_ASSERT -DUSE_FULL_LL_DRIVER -DVECT_TAB_SRAM -DUSE_IMX335_SENSOR -DUSE_VD66GY_SENSOR -DUSE_VD55G1_SENSOR -DUSE_VD1943_SENSOR -DCAMERA_SELFY=1 -DSTM32N6570_DK_REV -DTX_MAX_PARALLEL_NETWORKS=1 -DFEAT_FREERTOS -DUVC_LIB_USE_USBX -DUX_INCLUDE_USER_DEFINE_FILE -DUSBL_PACKET_PER_MICRO_FRAME=3 -DUX_STANDALONE -DUVCL_USBX_USE_FREERTOS -DUVC_LIB_USE_DMA -c -I../../../Inc -I"C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/STM32CubeIDE/STM32N6570-DK/Src" -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/DSP/Include -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/Common -I../../../STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx -I../../../Lib/Camera_Middleware -I../../../Lib/Camera_Middleware/sensors -I../../../Lib/Camera_Middleware/sensors/imx335 -I../../../Lib/Camera_Middleware/sensors/vd55g1 -I../../../Lib/Camera_Middleware/sensors/vd6g -I../../../Lib/Camera_Middleware/sensors/vd1943 -I../../../Lib/Camera_Middleware/ISP_Library/isp/Inc -I../../../Lib/FreeRTOS/Source/include -I../../../Lib/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure -I../../../Lib/FatFs -I../../../Lib/minimp4 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Lib/FatFs/ffunicode.o: C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/Lib/FatFs/ffunicode.c Lib/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DSTM32N657xx -DUX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT -DUX_DEVICE_STANDALONE -DUSE_FULL_ASSERT -DUSE_FULL_LL_DRIVER -DVECT_TAB_SRAM -DUSE_IMX335_SENSOR -DUSE_VD66GY_SENSOR -DUSE_VD55G1_SENSOR -DUSE_VD1943_SENSOR -DCAMERA_SELFY=1 -DSTM32N6570_DK_REV -DTX_MAX_PARALLEL_NETWORKS=1 -DFEAT_FREERTOS -DUVC_LIB_USE_USBX -DUX_INCLUDE_USER_DEFINE_FILE -DUSBL_PACKET_PER_MICRO_FRAME=3 -DUX_STANDALONE -DUVCL_USBX_USE_FREERTOS -DUVC_LIB_USE_DMA -c -I../../../Inc -I"C:/Users/jeremy.vonlanth/switchdrive/00_RaD/5_DIAS/DIAS/STM32CubeIDE/STM32N6570-DK/Src" -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/DSP/Include -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/Common -I../../../STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx -I../../../Lib/Camera_Middleware -I../../../Lib/Camera_Middleware/sensors -I../../../Lib/Camera_Middleware/sensors/imx335 -I../../../Lib/Camera_Middleware/sensors/vd55g1 -I../../../Lib/Camera_Middleware/sensors/vd6g -I../../../Lib/Camera_Middleware/sensors/vd1943 -I../../../Lib/Camera_Middleware/ISP_Library/isp/Inc -I../../../Lib/FreeRTOS/Source/include -I../../../Lib/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure -I../../../Lib/FatFs -I../../../Lib/minimp4 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-FatFs

clean-Lib-2f-FatFs:
	-$(RM) ./Lib/FatFs/ff.cyclo ./Lib/FatFs/ff.d ./Lib/FatFs/ff.o ./Lib/FatFs/ff.su ./Lib/FatFs/ffunicode.cyclo ./Lib/FatFs/ffunicode.d ./Lib/FatFs/ffunicode.o ./Lib/FatFs/ffunicode.su

.PHONY: clean-Lib-2f-FatFs

