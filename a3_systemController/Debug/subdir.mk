################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SimpleGPIO.c \
../audio.c \
../main.c \
../sensor.c 

OBJS += \
./SimpleGPIO.o \
./audio.o \
./main.o \
./sensor.o 

C_DEPS += \
./SimpleGPIO.d \
./audio.d \
./main.d \
./sensor.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	arm-linux-gnueabi-gcc -O0 -g3 -Wall -c -fmessage-length=0 -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


