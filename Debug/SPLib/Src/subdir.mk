################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SPLib/Src/bitManipulator.c \
../SPLib/Src/bitSet.c \
../SPLib/Src/linkedList.c \
../SPLib/Src/ringBuffer.c \
../SPLib/Src/simpleFSM.c \
../SPLib/Src/simpleTimer.c 

OBJS += \
./SPLib/Src/bitManipulator.o \
./SPLib/Src/bitSet.o \
./SPLib/Src/linkedList.o \
./SPLib/Src/ringBuffer.o \
./SPLib/Src/simpleFSM.o \
./SPLib/Src/simpleTimer.o 

C_DEPS += \
./SPLib/Src/bitManipulator.d \
./SPLib/Src/bitSet.d \
./SPLib/Src/linkedList.d \
./SPLib/Src/ringBuffer.d \
./SPLib/Src/simpleFSM.d \
./SPLib/Src/simpleTimer.d 


# Each subdirectory must supply rules for building sources it contributes
SPLib/Src/bitManipulator.o: ../SPLib/Src/bitManipulator.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Inc -I../../STM32WB55/SPLib/Src -I../../STM32WB55/Application -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"SPLib/Src/bitManipulator.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SPLib/Src/bitSet.o: ../SPLib/Src/bitSet.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Inc -I../../STM32WB55/SPLib/Src -I../../STM32WB55/Application -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"SPLib/Src/bitSet.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SPLib/Src/linkedList.o: ../SPLib/Src/linkedList.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Inc -I../../STM32WB55/SPLib/Src -I../../STM32WB55/Application -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"SPLib/Src/linkedList.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SPLib/Src/ringBuffer.o: ../SPLib/Src/ringBuffer.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Inc -I../../STM32WB55/SPLib/Src -I../../STM32WB55/Application -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"SPLib/Src/ringBuffer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SPLib/Src/simpleFSM.o: ../SPLib/Src/simpleFSM.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Inc -I../../STM32WB55/SPLib/Src -I../../STM32WB55/Application -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"SPLib/Src/simpleFSM.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SPLib/Src/simpleTimer.o: ../SPLib/Src/simpleTimer.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../STM32WB55/SPLib/Inc -I../../STM32WB55/SPLib/Src -I../../STM32WB55/Application -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"SPLib/Src/simpleTimer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

