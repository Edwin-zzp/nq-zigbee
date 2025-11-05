################################################################################
# MRS Version: 1.9.2
# 自动生成的文件。不要编辑！
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include userlib/subdir.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
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
SECONDARY_FLASH += \
app.hexAndbin \

SECONDARY_LIST += \
app.lst \

SECONDARY_SIZE += \
app.siz \


# 所有目标
all: app.elf secondary-outputs

# 工具调用
app.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -T "C:\Users\zhang\Desktop\work\4G\ZigBee\app\qn-zigbee\app\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -L"C:\Users\zhang\Desktop\work\4G\ZigBee\app\qn-zigbee\app\userlib" -Xlinker --print-memory-usage -Wl,-Map,"app.map" --specs=nano.specs --specs=nosys.specs -o "app.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
app.hexAndbin: app.elf
	@	riscv-none-embed-objcopy -O ihex "app.elf"  "app.hex" && riscv-none-embed-objcopy -O binary "app.elf"  "app.bin"
	@	@
app.lst: app.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble -M xw "app.elf" > "app.lst"
	@	@
app.siz: app.elf
	@	riscv-none-embed-size --format=berkeley "app.elf"
	@	@
	$(MAKE) --no-print-directory post-build

# 其他目标
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) app.elf
	-@
post-build:
	-riscv-none-embed-objcopy -O ihex "app.elf" "app.hex" && riscv-none-embed-objcopy -O binary "app.elf" "app.bin"
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents post-build

-include ../makefile.targets
