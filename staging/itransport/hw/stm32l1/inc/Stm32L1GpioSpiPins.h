#pragma once
#include <cstdint>
#include "stm32l1xx.h"

// Pins for BitBangSpiBlockTransport on an STM32L1: each pin is a GPIO port
// and a pin mask. Writes go through BSRR (one store each); DO is read from
// IDR. The pins must already be set up as outputs (CK, CS, DI: push-pull,
// "very high" or "high" speed) and an input (DO), by CubeMX or the project.
//
// The EEVblog 121GW's HY3131 (from its schematic):
//
//   Stm32L1GpioSpiPins hyPins{ {GPIOF, 1u << 0},    // HY_CK  PF0
//                              {GPIOF, 1u << 1},    // HY_CS  PF1
//                              {GPIOF, 1u << 2},    // HY_DI  PF2
//                              {GPIOF, 1u << 3},    // HY_DO  PF3
//                              2 };                 // delay loops per half clock
//
// delayLoops: the HY3131 wants CK high >= 20 ns, low >= 60 ns, and DO
// settles 60 ns after CK falls. At 32 MHz one GPIO store is about 30 ns,
// so a couple of loops is plenty. 0 for none.
struct Stm32L1GpioSpiPins {
    struct Pin { GPIO_TypeDef* port; uint16_t mask; };

    Pin      ck, cs, din, dout;   // din: MOSI (the chip's DI), dout: MISO (the chip's DO)
    uint32_t delayLoops = 2;

    void ckHigh() { ck.port->BSRR = ck.mask; }
    void ckLow()  { ck.port->BSRR = (uint32_t)ck.mask << 16; }
    void csHigh() { cs.port->BSRR = cs.mask; }
    void csLow()  { cs.port->BSRR = (uint32_t)cs.mask << 16; }
    void di(bool b) { din.port->BSRR = b ? (uint32_t)din.mask : (uint32_t)din.mask << 16; }
    bool doLevel() const { return (dout.port->IDR & dout.mask) != 0U; }
    // For hy3131_param_t::doLevel: p.doLevel = &Stm32L1GpioSpiPins::doLevelOf; p.doCtx = &hyPins;
    // (the pins are only addresses, so the copy the transport holds and this
    // one read the same pin).
    static bool doLevelOf(void* self) { return static_cast<const Stm32L1GpioSpiPins*>(self)->doLevel(); }

    void delay() const {
        for (volatile uint32_t i = 0; i < delayLoops; i = i + 1) {}
    }
};
