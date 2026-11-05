# The 121GW bring-up firmware (LCD, keys, rotary switch, HY3131): EEVBlog.bin for the stock bootloader.
#
#   make            build/EEVBlog.bin (and .elf, .map)
#
# arm-none-eabi-gcc (STM32CubeIDE's, or Arm's), and two header sets:
#   L1_DEVICE   cmsis_device_l1/Include (github.com/STMicroelectronics/cmsis_device_l1)
#   CMSIS_CORE  CMSIS 5 Core/Include (core_cm3.h): any STM32Cube project's Drivers/CMSIS/Include

STAGING    ?= ../../../..
EX         ?= $(STAGING)/../STM32L476RG_MC2515_Example/STM32L476RG_MC2515_Example
L1_DEVICE  ?= /tmp/cmsis_device_l1/Include
CMSIS_CORE ?= $(EX)/Drivers/CMSIS/Include
HMI_INC    ?= $(STAGING)/../l432kc-sensor-bringup/l432kc-sensor-bringup/iDisplay/hmi/inc
PREFIX     ?= arm-none-eabi-
OUT        ?= build

CPU   = -mcpu=cortex-m3 -mthumb
FLAGS = $(CPU) -std=c++17 -Os -g -Wall -Wextra -Werror -fno-exceptions -fno-rtti -fno-threadsafe-statics \
        -fno-use-cxa-atexit -ffunction-sections -fdata-sections -DSTM32L152xD
INC = -I.. -I$(STAGING)/boards/gw121/inc -I$(STAGING)/boards/gw121/sim -I$(STAGING)/imeter/inc \
      -I$(STAGING)/isensor/hy3131/inc -I$(STAGING)/isensor/hy3131/src -I$(STAGING)/idisplay/input/inc -I$(STAGING)/idisplay/segment/inc \
      -I$(STAGING)/idisplay/hw/stm32l1/inc -I$(STAGING)/itransport/inc -I$(STAGING)/itransport/hw/stm32l1/inc \
      -I$(HMI_INC) -I$(EX)/iTransport/inc -I$(EX)/isensor/inc -I$(L1_DEVICE) -I$(CMSIS_CORE)
SRC = main.cpp startup.cpp $(STAGING)/imeter/src/Meter.cpp
LDFLAGS = $(CPU) -T gw121_app.ld -nostartfiles --specs=nano.specs --specs=nosys.specs \
          -Wl,--gc-sections -Wl,-Map=$(OUT)/gw121_app.map -Wl,--print-memory-usage

$(OUT)/EEVBlog.bin: $(OUT)/gw121_app.elf
	$(PREFIX)objcopy -O binary $< $@
	@$(PREFIX)size $<

$(OUT)/gw121_app.elf: $(SRC) gw121_app.ld ../*.h $(STAGING)/boards/gw121/inc/*.h $(STAGING)/boards/gw121/sim/*.h $(STAGING)/idisplay/segment/inc/*.h
	mkdir -p $(OUT)
	$(PREFIX)g++ $(FLAGS) $(INC) $(SRC) $(LDFLAGS) -o $@ -lm

clean:
	rm -rf $(OUT)

.PHONY: clean
