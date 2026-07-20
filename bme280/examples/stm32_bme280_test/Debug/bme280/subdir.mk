################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../bme280/BME280.cpp 

OBJS += \
./bme280/BME280.o 

CPP_DEPS += \
./bme280/BME280.d 


# Each subdirectory must supply rules for building sources it contributes
bme280/%.o bme280/%.su bme280/%.cyclo: ../bme280/%.cpp bme280/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/coryg/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/Drivers/CMSIS/Include -I"D:/Documents/STM32_xSensor_Library/bme280/inc" -I"D:/Documents/STM32_xSensor_Library/bme280/examples/stm32_bme280_test/bme280" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bme280

clean-bme280:
	-$(RM) ./bme280/BME280.cyclo ./bme280/BME280.d ./bme280/BME280.o ./bme280/BME280.su

.PHONY: clean-bme280

