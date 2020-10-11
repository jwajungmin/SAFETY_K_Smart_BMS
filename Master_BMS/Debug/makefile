################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BMS_2020_07.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BMS_2020_07.list \

OBJCOPY_BIN += \
BMS_2020_07.bin \


# All Target
all: BMS_2020_07.elf secondary-outputs

# Tool invocations
BMS_2020_07.elf: $(OBJS) $(USER_OBJS) C:\Users\jmjwa\STM32CubeIDE\workspace_1.3.0\BMS_2020_07\STM32L452RETX_FLASH.ld
	arm-none-eabi-gcc -o "BMS_2020_07.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\jmjwa\STM32CubeIDE\workspace_1.3.0\BMS_2020_07\STM32L452RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BMS_2020_07.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BMS_2020_07.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BMS_2020_07.list"
	@echo 'Finished building: $@'
	@echo ' '

BMS_2020_07.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "BMS_2020_07.bin"
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
