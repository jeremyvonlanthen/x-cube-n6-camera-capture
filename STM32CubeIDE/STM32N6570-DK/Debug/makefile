################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Src/subdir.mk
-include STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Src/subdir.mk
-include STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Source/Templates/gcc/subdir.mk
-include STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Source/Templates/subdir.mk
-include STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK/subdir.mk
-include STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/subdir.mk
-include STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/subdir.mk
-include Lib/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure/subdir.mk
-include Lib/FreeRTOS/Source/subdir.mk
-include Lib/FatFs/subdir.mk
-include Lib/Camera_Middleware/sensors/vd6g/subdir.mk
-include Lib/Camera_Middleware/sensors/vd55g1/subdir.mk
-include Lib/Camera_Middleware/sensors/vd1943/subdir.mk
-include Lib/Camera_Middleware/sensors/imx335/subdir.mk
-include Lib/Camera_Middleware/sensors/subdir.mk
-include Lib/Camera_Middleware/ISP_Library/isp/Src/subdir.mk
-include Lib/Camera_Middleware/subdir.mk
-include Gcc/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := DIAS
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
DIAS.elf \

MAP_FILES += \
DIAS.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
DIAS.list \

OBJCOPY_BIN += \
DIAS.bin \


# All Target
all: main-build

# Main-build Target
main-build: DIAS.elf secondary-outputs

# Tool invocations
DIAS.elf DIAS.map: $(OBJS) $(USER_OBJS) ../../../Gcc/STM32N657xx.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "DIAS.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m55 -T"../../../Gcc/STM32N657xx.ld" --specs=nosys.specs -Wl,-Map="DIAS.map" -Wl,--gc-sections -static -Wl,--cmse-implib -Wl,--out-implib=./secure_nsclib.o -u _printf_float -Wl,--no-warn-rwx-segments --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

DIAS.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "DIAS.list"
	@echo 'Finished building: $@'
	@echo ' '

DIAS.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "DIAS.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) DIAS.bin DIAS.elf DIAS.list DIAS.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
