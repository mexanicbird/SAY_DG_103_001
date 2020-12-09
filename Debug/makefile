################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
SAY_DG_103_001.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SAY_DG_103_001.list \

OBJCOPY_BIN += \
SAY_DG_103_001.bin \


# All Target
all: SAY_DG_103_001.elf secondary-outputs

# Tool invocations
SAY_DG_103_001.elf: $(OBJS) $(USER_OBJS) C:\Users\Alexey\STM32CubeIDE\workspace_1.4.0\SAY_DG_103_001\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "SAY_DG_103_001.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Alexey\STM32CubeIDE\workspace_1.4.0\SAY_DG_103_001\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SAY_DG_103_001.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SAY_DG_103_001.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SAY_DG_103_001.list"
	@echo 'Finished building: $@'
	@echo ' '

SAY_DG_103_001.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SAY_DG_103_001.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
