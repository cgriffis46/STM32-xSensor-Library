# Host tests of the segment LCD HMI.

IDISPLAY_HMI ?= ../../../../l432kc-sensor-bringup/l432kc-sensor-bringup/iDisplay/hmi/inc
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g

segment_test: segment_test.cpp FakeTags.h ../inc/*.h
	$(CXX) $(CXXFLAGS) -I../inc -I$(IDISPLAY_HMI) -o $@ segment_test.cpp

test: segment_test
	./segment_test

clean:
	rm -f segment_test

.PHONY: test clean
