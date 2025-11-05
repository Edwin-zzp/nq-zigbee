################################################################################
# MRS Version: 1.9.2
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../userlib/crc16.c \
../userlib/device_hw.c \
../userlib/device_logic.c \
../userlib/proto.c \
../userlib/ringbuf.c \
../userlib/timer.c \
../userlib/usart2_dma.c 

OBJS += \
./userlib/crc16.o \
./userlib/device_hw.o \
./userlib/device_logic.o \
./userlib/proto.o \
./userlib/ringbuf.o \
./userlib/timer.o \
./userlib/usart2_dma.o 

C_DEPS += \
./userlib/crc16.d \
./userlib/device_hw.d \
./userlib/device_logic.d \
./userlib/proto.d \
./userlib/ringbuf.d \
./userlib/timer.d \
./userlib/usart2_dma.d 


# Each subdirectory must supply rules for building sources it contributes
userlib/%.o: ../userlib/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -DCH32V20x -I"C:\Users\zhang\Desktop\work\4G\ZigBee\app\qn-zigbee\app\Debug" -I"C:\Users\zhang\Desktop\work\4G\ZigBee\app\qn-zigbee\app\userlib" -I"C:\Users\zhang\Desktop\work\4G\ZigBee\app\qn-zigbee\app\Core" -I"C:\Users\zhang\Desktop\work\4G\ZigBee\app\qn-zigbee\app\User" -I"C:\Users\zhang\Desktop\work\4G\ZigBee\app\qn-zigbee\app\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

