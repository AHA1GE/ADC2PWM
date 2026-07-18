################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/OLED.c \
../User/OLED_Fonts.c \
../User/OLED_driver.c \
../User/bldc_control.c \
../User/bldc_hal.c \
../User/ch32v00x_it.c \
../User/drv8311.c \
../User/main.c \
../User/system_ch32v00x.c 

C_DEPS += \
./User/OLED.d \
./User/OLED_Fonts.d \
./User/OLED_driver.d \
./User/bldc_control.d \
./User/bldc_hal.d \
./User/ch32v00x_it.d \
./User/drv8311.d \
./User/main.d \
./User/system_ch32v00x.d 

OBJS += \
./User/OLED.o \
./User/OLED_Fonts.o \
./User/OLED_driver.o \
./User/bldc_control.o \
./User/bldc_hal.o \
./User/ch32v00x_it.o \
./User/drv8311.o \
./User/main.o \
./User/system_ch32v00x.o 

DIR_OBJS += \
./User/*.o \

DIR_DEPS += \
./User/*.d \

DIR_EXPANDS += \
./User/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -fdump-rtl-expand -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/Harry/Documents/GitHub/ADC2PWM/MounRiver_Firmware_CH32V003/BLDC_MOTOR_FAN_AIO/Debug" -I"c:/Users/Harry/Documents/GitHub/ADC2PWM/MounRiver_Firmware_CH32V003/BLDC_MOTOR_FAN_AIO/Core" -I"c:/Users/Harry/Documents/GitHub/ADC2PWM/MounRiver_Firmware_CH32V003/BLDC_MOTOR_FAN_AIO/User" -I"c:/Users/Harry/Documents/GitHub/ADC2PWM/MounRiver_Firmware_CH32V003/BLDC_MOTOR_FAN_AIO/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

