################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../RTC-lib/src/RTC_DS1307.cpp \
../RTC-lib/src/RTC_DS3231.cpp \
../RTC-lib/src/RTC_PCF8523.cpp \
../RTC-lib/src/RTC_PCF8563.cpp \
../RTC-lib/src/RTClib.cpp 

C_SRCS += \
../RTC-lib/src/STM32L4xx_RTC_Lib.c 

C_DEPS += \
./RTC-lib/src/STM32L4xx_RTC_Lib.d 

OBJS += \
./RTC-lib/src/RTC_DS1307.o \
./RTC-lib/src/RTC_DS3231.o \
./RTC-lib/src/RTC_PCF8523.o \
./RTC-lib/src/RTC_PCF8563.o \
./RTC-lib/src/RTClib.o \
./RTC-lib/src/STM32L4xx_RTC_Lib.o 

CPP_DEPS += \
./RTC-lib/src/RTC_DS1307.d \
./RTC-lib/src/RTC_DS3231.d \
./RTC-lib/src/RTC_PCF8523.d \
./RTC-lib/src/RTC_PCF8563.d \
./RTC-lib/src/RTClib.d 


# Each subdirectory must supply rules for building sources it contributes
RTC-lib/src/%.o RTC-lib/src/%.su RTC-lib/src/%.cyclo: ../RTC-lib/src/%.cpp RTC-lib/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/Documents/STM32_xSensor_Library/STM32 xSensor Library/rtc-lib/stm32-rtclib-example/RTC-lib" -I"D:/Documents/STM32_xSensor_Library/STM32 xSensor Library/rtc-lib/stm32-rtclib-example/RTC-lib/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
RTC-lib/src/%.o RTC-lib/src/%.su RTC-lib/src/%.cyclo: ../RTC-lib/src/%.c RTC-lib/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RTC-2d-lib-2f-src

clean-RTC-2d-lib-2f-src:
	-$(RM) ./RTC-lib/src/RTC_DS1307.cyclo ./RTC-lib/src/RTC_DS1307.d ./RTC-lib/src/RTC_DS1307.o ./RTC-lib/src/RTC_DS1307.su ./RTC-lib/src/RTC_DS3231.cyclo ./RTC-lib/src/RTC_DS3231.d ./RTC-lib/src/RTC_DS3231.o ./RTC-lib/src/RTC_DS3231.su ./RTC-lib/src/RTC_PCF8523.cyclo ./RTC-lib/src/RTC_PCF8523.d ./RTC-lib/src/RTC_PCF8523.o ./RTC-lib/src/RTC_PCF8523.su ./RTC-lib/src/RTC_PCF8563.cyclo ./RTC-lib/src/RTC_PCF8563.d ./RTC-lib/src/RTC_PCF8563.o ./RTC-lib/src/RTC_PCF8563.su ./RTC-lib/src/RTClib.cyclo ./RTC-lib/src/RTClib.d ./RTC-lib/src/RTClib.o ./RTC-lib/src/RTClib.su ./RTC-lib/src/STM32L4xx_RTC_Lib.cyclo ./RTC-lib/src/STM32L4xx_RTC_Lib.d ./RTC-lib/src/STM32L4xx_RTC_Lib.o ./RTC-lib/src/STM32L4xx_RTC_Lib.su

.PHONY: clean-RTC-2d-lib-2f-src

