################################################################################
# MRS Version: 1.9.2
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Debug/debug.c 

OBJS += \
./Debug/debug.o 

C_DEPS += \
./Debug/debug.d 


# Each subdirectory must supply rules for building sources it contributes
Debug/%.o: ../Debug/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -DCH32V20x -I"C:\Users\zhang\Desktop\work\4G\ZigBee\app\github\app\Debug" -I"C:\Users\zhang\Desktop\work\4G\ZigBee\app\github\app\userlib" -I"C:\Users\zhang\Desktop\work\4G\ZigBee\app\github\app\Core" -I"C:\Users\zhang\Desktop\work\4G\ZigBee\app\github\app\User" -I"C:\Users\zhang\Desktop\work\4G\ZigBee\app\github\app\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

