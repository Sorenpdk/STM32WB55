################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application2/Application2.c 

OBJS += \
./Application2/Application2.o 

C_DEPS += \
./Application2/Application2.d 


# Each subdirectory must supply rules for building sources it contributes
Application2/Application2.o: ../Application2/Application2.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/Application2 -I../../STM32WB55/SPLib/Inc -I../../STM32WB55/SPLib/Src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application2/Application2.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

