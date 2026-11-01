# Host tests of the 121GW board package.

EX       ?= ../../../../STM32L476RG_MC2515_Example/STM32L476RG_MC2515_Example
HY3131   ?= ../../../isensor/hy3131
IMETER   ?= ../../../imeter
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g

gw121_test: gw121_test.cpp ../inc/*.h $(IMETER)/inc/*.h $(IMETER)/src/Meter.cpp $(HY3131)/inc/*.h $(HY3131)/src/*.tpp $(HY3131)/test/SimHy3131.h
	$(CXX) $(CXXFLAGS) -I../inc -I$(IMETER)/inc -I$(HY3131)/inc -I$(HY3131)/test -I$(EX)/iTransport/inc -I$(EX)/isensor/inc \
	  -o $@ gw121_test.cpp $(IMETER)/src/Meter.cpp

test: gw121_test
	./gw121_test

clean:
	rm -f gw121_test

.PHONY: test clean
