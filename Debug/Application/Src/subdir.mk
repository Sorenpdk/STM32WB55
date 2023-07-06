################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Src/accelerometer_lis2dw12.c \
../Application/Src/app_main.c 

OBJS += \
./Application/Src/accelerometer_lis2dw12.o \
./Application/Src/app_main.o 

C_DEPS += \
./Application/Src/accelerometer_lis2dw12.d \
./Application/Src/app_main.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Src/accelerometer_lis2dw12.o: ../Application/Src/accelerometer_lis2dw12.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../../STM32WB55/SPLib/Inc -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Src -I../../STM32WB55/SPDriver/Inc -I../../STM32WB55/SPDriver/Src -I"C:/Users/soren/Desktop/STM32WB55/SPDriver/Inc" -I../../STM32WB55/Application/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Src/accelerometer_lis2dw12.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/app_main.o: ../Application/Src/app_main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../../STM32WB55/SPLib/Inc -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Src -I../../STM32WB55/SPDriver/Inc -I../../STM32WB55/SPDriver/Src -I"C:/Users/soren/Desktop/STM32WB55/SPDriver/Inc" -I../../STM32WB55/Application/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Src/app_main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

