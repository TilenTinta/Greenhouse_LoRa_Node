################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ideetron/AES-128_V10.c \
../Drivers/ideetron/Encrypt_V31.c 

OBJS += \
./Drivers/ideetron/AES-128_V10.o \
./Drivers/ideetron/Encrypt_V31.o 

C_DEPS += \
./Drivers/ideetron/AES-128_V10.d \
./Drivers/ideetron/Encrypt_V31.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ideetron/%.o Drivers/ideetron/%.su Drivers/ideetron/%.cyclo: ../Drivers/ideetron/%.c Drivers/ideetron/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L151xB -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"C:/DATA/Projects/Rastlinjak/Greenhouse_LoRa_Node/Firmware_L151/Drivers/ideetron" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-ideetron

clean-Drivers-2f-ideetron:
	-$(RM) ./Drivers/ideetron/AES-128_V10.cyclo ./Drivers/ideetron/AES-128_V10.d ./Drivers/ideetron/AES-128_V10.o ./Drivers/ideetron/AES-128_V10.su ./Drivers/ideetron/Encrypt_V31.cyclo ./Drivers/ideetron/Encrypt_V31.d ./Drivers/ideetron/Encrypt_V31.o ./Drivers/ideetron/Encrypt_V31.su

.PHONY: clean-Drivers-2f-ideetron

