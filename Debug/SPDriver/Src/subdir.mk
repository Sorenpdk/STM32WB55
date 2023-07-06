################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SPDriver/Src/SP_GPIO.c \
../SPDriver/Src/SP_IntFlash.c \
../SPDriver/Src/SP_SPI.c \
../SPDriver/Src/SP_Uart.c 

OBJS += \
./SPDriver/Src/SP_GPIO.o \
./SPDriver/Src/SP_IntFlash.o \
./SPDriver/Src/SP_SPI.o \
./SPDriver/Src/SP_Uart.o 

C_DEPS += \
./SPDriver/Src/SP_GPIO.d \
./SPDriver/Src/SP_IntFlash.d \
./SPDriver/Src/SP_SPI.d \
./SPDriver/Src/SP_Uart.d 


# Each subdirectory must supply rules for building sources it contributes
SPDriver/Src/SP_GPIO.o: ../SPDriver/Src/SP_GPIO.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../../STM32WB55/SPLib/Inc -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Src -I../../STM32WB55/SPDriver/Inc -I../../STM32WB55/SPDriver/Src -I../../STM32WB55/Application/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"SPDriver/Src/SP_GPIO.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SPDriver/Src/SP_IntFlash.o: ../SPDriver/Src/SP_IntFlash.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../../STM32WB55/SPLib/Inc -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Src -I../../STM32WB55/SPDriver/Inc -I../../STM32WB55/SPDriver/Src -I../../STM32WB55/Application/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"SPDriver/Src/SP_IntFlash.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SPDriver/Src/SP_SPI.o: ../SPDriver/Src/SP_SPI.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../../STM32WB55/SPLib/Inc -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Src -I../../STM32WB55/SPDriver/Inc -I../../STM32WB55/SPDriver/Src -I../../STM32WB55/Application/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"SPDriver/Src/SP_SPI.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SPDriver/Src/SP_Uart.o: ../SPDriver/Src/SP_Uart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../../STM32WB55/SPLib/Inc -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Src -I../../STM32WB55/SPDriver/Inc -I../../STM32WB55/SPDriver/Src -I../../STM32WB55/Application/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"SPDriver/Src/SP_Uart.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

