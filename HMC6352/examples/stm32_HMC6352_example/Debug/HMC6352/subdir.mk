################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../HMC6352/hmc6352.cpp 

OBJS += \
./HMC6352/hmc6352.o 

CPP_DEPS += \
./HMC6352/hmc6352.d 


# Each subdirectory must supply rules for building sources it contributes
HMC6352/%.o HMC6352/%.su HMC6352/%.cyclo: ../HMC6352/%.cpp HMC6352/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_NUCLEO_32 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/BSP/STM32L4xx_Nucleo_32 -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/Documents/STM32_xSensor_Library/STM32 xSensor Library/HMC6352/examples/stm32_HMC6352_example/HMC6352" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-HMC6352

clean-HMC6352:
	-$(RM) ./HMC6352/hmc6352.cyclo ./HMC6352/hmc6352.d ./HMC6352/hmc6352.o ./HMC6352/hmc6352.su

.PHONY: clean-HMC6352

