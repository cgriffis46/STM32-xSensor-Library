################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../aht20/AHT20.cpp 

OBJS += \
./aht20/AHT20.o 

CPP_DEPS += \
./aht20/AHT20.d 


# Each subdirectory must supply rules for building sources it contributes
aht20/%.o aht20/%.su aht20/%.cyclo: ../aht20/%.cpp aht20/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_NUCLEO_32 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32L4xx_Nucleo_32 -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/Documents/STM32_xSensor_Library/STM32 xSensor Library/aht20/examples/stm32_aht20_test/aht20" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-aht20

clean-aht20:
	-$(RM) ./aht20/AHT20.cyclo ./aht20/AHT20.d ./aht20/AHT20.o ./aht20/AHT20.su

.PHONY: clean-aht20

