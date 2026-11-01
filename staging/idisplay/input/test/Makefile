# Host tests of KeyScanner and Buzzer.
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g

input_test: input_test.cpp ../inc/*.h
	$(CXX) $(CXXFLAGS) -I../inc -o $@ input_test.cpp

test: input_test
	./input_test

clean:
	rm -f input_test

.PHONY: test clean
