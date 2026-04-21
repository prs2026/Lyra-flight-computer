################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32WLxx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/Inc/subdir.mk
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


BUILD_ARTIFACT_NAME := camcontrolstm32cubepleaseworkibegofyou
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
camcontrolstm32cubepleaseworkibegofyou.elf \

MAP_FILES += \
camcontrolstm32cubepleaseworkibegofyou.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
camcontrolstm32cubepleaseworkibegofyou.list \


# All Target
all: main-build

# Main-build Target
main-build: camcontrolstm32cubepleaseworkibegofyou.elf secondary-outputs

# Tool invocations
camcontrolstm32cubepleaseworkibegofyou.elf camcontrolstm32cubepleaseworkibegofyou.map: $(OBJS) $(USER_OBJS) C:\Users\E\STM32CubeIDE\workspace_1.13.1\camcontrolstm32cubepleaseworkibegofyou\STM32WLE5CCUX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "camcontrolstm32cubepleaseworkibegofyou.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\E\STM32CubeIDE\workspace_1.13.1\camcontrolstm32cubepleaseworkibegofyou\STM32WLE5CCUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="camcontrolstm32cubepleaseworkibegofyou.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

camcontrolstm32cubepleaseworkibegofyou.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "camcontrolstm32cubepleaseworkibegofyou.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) camcontrolstm32cubepleaseworkibegofyou.elf camcontrolstm32cubepleaseworkibegofyou.list camcontrolstm32cubepleaseworkibegofyou.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
