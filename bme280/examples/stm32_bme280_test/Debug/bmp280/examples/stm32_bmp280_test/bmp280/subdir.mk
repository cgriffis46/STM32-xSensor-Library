################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
D:/Documents/STM32_xSensor_Library/bmp280/examples/stm32_bmp280_test/bmp280/bmp280.cpp 

OBJS += \
./bmp280/examples/stm32_bmp280_test/bmp280/bmp280.o 

CPP_DEPS += \
./bmp280/examples/stm32_bmp280_test/bmp280/bmp280.d 


# Each subdirectory must supply rules for building sources it contributes
bmp280/examples/stm32_bmp280_test/bmp280/bmp280.o: D:/Documents/STM32_xSensor_Library/bmp280/examples/stm32_bmp280_test/bmp280/bmp280.cpp bmp280/examples/stm32_bmp280_test/bmp280/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Drivers/CMSIS/Include -I"D:/Documents/STM32_xSensor_Library/bmp280/inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bmp280-2f-examples-2f-stm32_bmp280_test-2f-bmp280

clean-bmp280-2f-examples-2f-stm32_bmp280_test-2f-bmp280:
	-$(RM) ./bmp280/examples/stm32_bmp280_test/bmp280/bmp280.cyclo ./bmp280/examples/stm32_bmp280_test/bmp280/bmp280.d ./bmp280/examples/stm32_bmp280_test/bmp280/bmp280.o ./bmp280/examples/stm32_bmp280_test/bmp280/bmp280.su

.PHONY: clean-bmp280-2f-examples-2f-stm32_bmp280_test-2f-bmp280

