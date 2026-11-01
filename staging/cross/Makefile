# Builds the staged L1 transport, HY3131 driver and imeter for the
# Cortex-M3 (STM32L152xD) with arm-none-eabi-g++: a compile and size check
# against ST's real headers and CMSIS-RTOS2. Nothing is linked.
#
#   L1_DEVICE: cmsis_device_l1/Include (github.com/STMicroelectronics/cmsis_device_l1)

EX         ?= ../../STM32L476RG_MC2515_Example/STM32L476RG_MC2515_Example
L1_DEVICE  ?= /tmp/cmsis_device_l1/Include
CMSIS_CORE ?= $(EX)/Drivers/CMSIS/Include
RTOS       ?= $(EX)/Middlewares/Third_Party/FreeRTOS/Source
ARM_CXX    ?= arm-none-eabi-g++
FLAGS = -mcpu=cortex-m3 -mthumb -std=c++17 -Os -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
        -ffunction-sections -fdata-sections -DSTM32L152xD
INC = -I../itransport/hw/stm32l1/inc -I../isensor/hy3131/inc -I../imeter/inc \
      -I$(EX)/iTransport/inc -I$(EX)/iTransport/hw/freertos/inc -I$(EX)/isensor/inc \
      -I$(L1_DEVICE) -I$(CMSIS_CORE) -I$(RTOS)/CMSIS_RTOS_V2 -I$(RTOS)/include \
      -I$(RTOS)/portable/GCC/ARM_CM3 -I$(EX)/Core/Inc
OUT ?= /tmp/gw121-cross

cross:
	mkdir -p $(OUT)
	$(ARM_CXX) $(FLAGS) $(INC) -c gw121_stack.cpp -o $(OUT)/gw121_stack.o
	$(ARM_CXX) $(FLAGS) $(INC) -c ../imeter/src/Meter.cpp -o $(OUT)/Meter.o
	$(ARM_CXX) $(FLAGS) $(INC) -c ../itransport/hw/stm32l1/src/Stm32L1SpiPort.cpp -o $(OUT)/Stm32L1SpiPort.o
	arm-none-eabi-size $(OUT)/*.o

.PHONY: cross
