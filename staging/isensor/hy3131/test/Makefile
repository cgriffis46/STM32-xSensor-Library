# Host test of the HY3131 driver. Needs iBlockTransport.h, SensorStateMachine.h
# and DebugLog.h: taken from the MCP2515 example's copy of Claude-iTransport
# unless ITRANSPORT points at another.

ITRANSPORT ?= ../../../../STM32L476RG_MC2515_Example/STM32L476RG_MC2515_Example
CXX       ?= g++
CXXFLAGS  ?= -std=c++17 -Wall -Wextra -Werror -O1 -g

hy3131_test: hy3131_test.cpp ../inc/*.h ../src/*.tpp
	$(CXX) $(CXXFLAGS) -I../inc -I$(ITRANSPORT)/iTransport/inc -I$(ITRANSPORT)/isensor/inc -o $@ hy3131_test.cpp

test: hy3131_test
	./hy3131_test

clean:
	rm -f hy3131_test

.PHONY: test clean
