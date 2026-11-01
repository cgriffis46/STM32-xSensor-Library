# Host test of BitBangSpiBlockTransport, against a pin-level simulated HY3131.

EX       ?= ../../../STM32L476RG_MC2515_Example/STM32L476RG_MC2515_Example
HY3131   ?= ../../isensor/hy3131
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g

bitbang_test: bitbang_test.cpp ../inc/*.h $(HY3131)/inc/*.h $(HY3131)/src/*.tpp $(HY3131)/test/SimHy3131.h
	$(CXX) $(CXXFLAGS) -I../inc -I$(HY3131)/inc -I$(HY3131)/test -I$(EX)/iTransport/inc -I$(EX)/isensor/inc -o $@ bitbang_test.cpp

test: bitbang_test
	./bitbang_test

clean:
	rm -f bitbang_test

.PHONY: test clean
