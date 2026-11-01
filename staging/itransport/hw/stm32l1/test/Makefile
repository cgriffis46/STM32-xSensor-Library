# Host test of the STM32L1 SPI port and transport, against fake registers.
# `make cross` also builds the sources for the Cortex-M3 (STM32L152xD) with
# arm-none-eabi-g++, against ST's real L1 headers (give L1_DEVICE and CMSIS_CORE).

ITRANSPORT ?= ../../../../../STM32L476RG_MC2515_Example/STM32L476RG_MC2515_Example/iTransport
CXX        ?= g++
CXXFLAGS   ?= -std=c++17 -Wall -Wextra -Werror -O1 -g
SRC = stm32l1_spi_test.cpp ../src/Stm32L1SpiPort.cpp $(ITRANSPORT)/src/SpiBlockTransport.cpp $(ITRANSPORT)/src/BusTransport.cpp
INC = -Ifake -I../inc -I$(ITRANSPORT)/inc

stm32l1_spi_test: $(SRC) ../inc/*.h
	$(CXX) $(CXXFLAGS) $(INC) -o $@ $(SRC)

test: stm32l1_spi_test
	./stm32l1_spi_test

# cmsis_device_l1 (github.com/STMicroelectronics/cmsis_device_l1)/Include,
# and CMSIS/Core/Include (any CubeIDE project's Drivers/CMSIS/Include).
L1_DEVICE  ?= /tmp/cmsis_device_l1/Include
CMSIS_CORE ?= ../../../../../STM32L476RG_MC2515_Example/STM32L476RG_MC2515_Example/Drivers/CMSIS/Include
ARM_CXX    ?= arm-none-eabi-g++
ARM_FLAGS  ?= -mcpu=cortex-m3 -mthumb -std=c++17 -Os -Wall -Wextra -Werror -fno-exceptions -fno-rtti -DSTM32L152xD
cross:
	$(ARM_CXX) $(ARM_FLAGS) -I../inc -I$(ITRANSPORT)/inc -I$(L1_DEVICE) -I$(CMSIS_CORE) \
	  -c ../src/Stm32L1SpiPort.cpp -o /tmp/Stm32L1SpiPort.o
	$(ARM_CXX) $(ARM_FLAGS) -I../inc -I$(ITRANSPORT)/inc -I$(L1_DEVICE) -I$(CMSIS_CORE) \
	  -c cross_check.cpp -o /tmp/cross_check.o
	arm-none-eabi-size /tmp/Stm32L1SpiPort.o /tmp/cross_check.o

clean:
	rm -f stm32l1_spi_test

.PHONY: test cross clean
