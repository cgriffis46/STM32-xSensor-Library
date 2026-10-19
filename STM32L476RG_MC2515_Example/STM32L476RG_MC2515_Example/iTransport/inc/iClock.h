#pragma once
#include <cstdint>

// iClock is a free running counter to time things by: now() in ticks of
// ticksPerSecond(), counting up and wrapping at 2^32. Take differences
// as signed 32 bit ((int32_t)(a - b)) and they are right across the
// wrap, for spans up to half of it.
//
// Where the time comes from is the implementation's business: the
// STM32 RTC on its 32.768 kHz crystal (Stm32RtcClock), a timer, a test's
// simulated clock. It need not be the RTOS tick, and is often better
// than it: a watch crystal is good to 20 ppm, keeps running in low power
// modes where the tick stops, and does not care how the CPU is clocked.
//
// now() must be callable from an interrupt, so an interrupt handler can
// stamp an event the moment it sees it.
class iClock {
public:
    virtual ~iClock() = default;

    virtual uint32_t ticksPerSecond() const = 0;

    virtual uint32_t now() = 0;
};
