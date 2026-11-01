# Host test of Stm32L1Lcd against fake registers.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g

lcd_test: lcd_test.cpp ../inc/*.h ../../../segment/inc/*.h
	$(CXX) $(CXXFLAGS) -Ifake -I../inc -I../../../segment/inc -o $@ lcd_test.cpp

test: lcd_test
	./lcd_test

clean:
	rm -f lcd_test

.PHONY: test clean
