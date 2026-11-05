# Host test of the LCD bring-up app.

EX       ?= ../../../../../STM32L476RG_MC2515_Example/STM32L476RG_MC2515_Example
STAGING  ?= ../../../..
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g
INC = -I.. -I$(STAGING)/boards/gw121/inc -I$(STAGING)/boards/gw121/sim -I$(STAGING)/imeter/inc \
      -I$(STAGING)/isensor/hy3131/inc -I$(STAGING)/idisplay/input/inc -I$(STAGING)/idisplay/segment/inc \
      -I$(STAGING)/../l432kc-sensor-bringup/l432kc-sensor-bringup/iDisplay/hmi/inc \
      -I$(EX)/iTransport/inc -I$(EX)/isensor/inc

app_test: app_test.cpp ../*.h $(STAGING)/boards/gw121/inc/*.h $(STAGING)/boards/gw121/sim/*.h $(STAGING)/imeter/inc/*.h $(STAGING)/imeter/src/Meter.cpp $(STAGING)/idisplay/segment/inc/*.h
	$(CXX) $(CXXFLAGS) $(INC) -o $@ app_test.cpp $(STAGING)/imeter/src/Meter.cpp

test: app_test
	./app_test

clean:
	rm -f app_test

.PHONY: test clean
