# Host test of the 121GW simulator. `make packets` also writes the packets
# it sent, for the web page's decoder test.

EX       ?= ../../../../../STM32L476RG_MC2515_Example/STM32L476RG_MC2515_Example
STAGING  ?= ../../../..
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g
INC = -I.. -I$(STAGING)/boards/gw121/inc -I$(STAGING)/imeter/inc -I$(STAGING)/isensor/hy3131/inc \
      -I$(STAGING)/idisplay/input/inc -I$(STAGING)/idisplay/segment/inc \
      -I$(STAGING)/../l432kc-sensor-bringup/l432kc-sensor-bringup/iDisplay/hmi/inc \
      -I$(EX)/iTransport/inc -I$(EX)/isensor/inc

sim_test: sim_test.cpp ../*.h $(STAGING)/boards/gw121/inc/*.h $(STAGING)/imeter/inc/*.h $(STAGING)/imeter/src/Meter.cpp
	$(CXX) $(CXXFLAGS) $(INC) -o $@ sim_test.cpp $(STAGING)/imeter/src/Meter.cpp

test: sim_test
	./sim_test

packets: sim_test
	./sim_test packets.txt

clean:
	rm -f sim_test packets.txt

.PHONY: test packets clean
