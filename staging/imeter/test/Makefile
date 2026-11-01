# Host tests of imeter: Meter against a scripted front end, and end to end
# through Hy3131FrontEnd and hy3131_dmm against the simulated HY3131.

ITRANSPORT ?= ../../../STM32L476RG_MC2515_Example/STM32L476RG_MC2515_Example
HY3131     ?= ../../isensor/hy3131
CXX        ?= g++
CXXFLAGS   ?= -std=c++17 -Wall -Wextra -Werror -O1 -g

meter_test: meter_test.cpp ../src/Meter.cpp ../inc/*.h $(HY3131)/inc/*.h $(HY3131)/src/*.tpp $(HY3131)/test/SimHy3131.h
	$(CXX) $(CXXFLAGS) -I../inc -I$(HY3131)/inc -I$(HY3131)/test -I$(ITRANSPORT)/iTransport/inc -I$(ITRANSPORT)/isensor/inc \
	  -o $@ meter_test.cpp ../src/Meter.cpp

test: meter_test
	./meter_test

clean:
	rm -f meter_test

.PHONY: test clean
