################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../camera_ptzcontrol.c \
../cmdqu.c \
../cyfxtx.c \
../cyfxuvcdscr.c \
../sensor.c \
../uvc.c 

S_UPPER_SRCS += \
../cyfx_gcc_startup.S 

OBJS += \
./camera_ptzcontrol.o \
./cmdqu.o \
./cyfx_gcc_startup.o \
./cyfxtx.o \
./cyfxuvcdscr.o \
./sensor.o \
./uvc.o 

C_DEPS += \
./camera_ptzcontrol.d \
./cmdqu.d \
./cyfxtx.d \
./cyfxuvcdscr.d \
./sensor.d \
./uvc.d 

S_UPPER_DEPS += \
./cyfx_gcc_startup.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -I"C:/Program Files/Cypress/EZ-USB FX3 SDK/1.3/fw_lib/1_3_3/inc" -O3 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=arm926ej-s -mthumb-interwork -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.S
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC Assembler'
	arm-none-eabi-gcc -x assembler-with-cpp -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=arm926ej-s -mthumb-interwork -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


