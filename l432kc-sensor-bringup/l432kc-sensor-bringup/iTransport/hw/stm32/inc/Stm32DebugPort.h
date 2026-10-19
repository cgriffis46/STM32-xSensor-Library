#pragma once
// The STM32 side of DebugLog.h: the clock is HAL_GetTick(), the lock
// masks interrupts through PRIMASK (saved and put back, so it nests and
// is safe in an interrupt), and the debug pins are written through
// their port's BSRR (one store, no read-modify-write, interrupt safe).
//
//     static const Stm32DebugPin pins[] = {      // DbgPin order; any free GPIOs,
//         {GPIOA, GPIO_PIN_8},                   //   set up as push-pull outputs
//         {GPIOA, GPIO_PIN_11},                  //   in CubeMX
//         {GPIOB, GPIO_PIN_4},
//         {GPIOB, GPIO_PIN_5},
//     };
//     Stm32DebugPort::setPins(pins, 4);
//     static Stm32HalUartTransport debugUart(&huart2);
//     dbg::begin(&debugUart, Stm32DebugPort::port());
//     ... then dbg::poll() from the idle loop or a low priority task.
//
// The UART: any UART with its TX interrupt (or DMA) enabled in CubeMX.
// Its HAL callbacks come from itransport's Stm32UartItCallbacks.cpp, as
// for any other Stm32HalUartTransport. Run it fast (921600 8N1 or
// more); level 3 can produce several kilobytes a second.
//
// Uses only CMSIS's __get_PRIMASK/__set_PRIMASK/__disable_irq and the
// HAL's HAL_GetTick() and GPIO_TypeDef, through the CubeMX project's
// main.h like the other STM32 transports.

#include "main.h"
#include "DebugLog.h"

struct Stm32DebugPin {
    GPIO_TypeDef* port;
    uint16_t      pin;   // GPIO_PIN_x
};

class Stm32DebugPort {
public:
    // pins[i] is DbgPin i (see DebugLog.h); ids past n are ignored. The
    // array must outlive the logger (a static const one).
    static void setPins(const Stm32DebugPin* pins, uint8_t n) {
        pins_() = pins;
        count_() = n;
    }

    static dbg::Port port() {
        dbg::Port p;
        p.nowMs = &nowMs;
        p.lock = &lock;
        p.unlock = &unlock;
        p.pin = &pin;
        return p;
    }

private:
    static uint32_t nowMs() { return HAL_GetTick(); }
    static uint32_t lock() {
        const uint32_t saved = __get_PRIMASK();
        __disable_irq();
        return saved;
    }
    static void unlock(uint32_t saved) { __set_PRIMASK(saved); }
    static void pin(uint8_t id, bool level) {
        if (pins_() == nullptr || id >= count_()) return;
        const Stm32DebugPin& p = pins_()[id];
        p.port->BSRR = level ? (uint32_t)p.pin : ((uint32_t)p.pin << 16);
    }

    // Function-local statics: a header-only class with no .cpp to put them in.
    static const Stm32DebugPin*& pins_() { static const Stm32DebugPin* p = nullptr; return p; }
    static uint8_t& count_() { static uint8_t n = 0; return n; }
};
