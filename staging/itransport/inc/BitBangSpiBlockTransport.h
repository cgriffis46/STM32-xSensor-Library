#pragma once
#include <cstddef>
#include <cstdint>
#include "iBlockTransport.h"

// iBlockTransport on four GPIO pins, SPI mode 0, MSB first. For a chip on
// pins with no SPI peripheral behind them, such as the HY3131 in the
// EEVblog 121GW (PF0..PF3 of an STM32L152).
//
// TPins is a small policy class, so the pin writes compile to single stores:
//
//   struct Pins {
//       void ckHigh(); void ckLow();       // clock
//       void csHigh(); void csLow();       // chip-select (active low)
//       void di(bool b);                   // MOSI: the chip's DI
//       bool doLevel() const;              // MISO: the chip's DO
//       void delay() const;                // half a clock period, or nothing
//   };
//
// Stm32L1GpioSpiPins (hw/stm32l1) is one. A test's simulated chip is another.
//
// A transfer runs to the end inside beginRead()/beginWrite(). The CPU is the
// shifter, so there is nothing to hand to an interrupt. isBusy() is always
// false afterwards. At 32 MHz a byte takes a few microseconds; the HY3131's
// longest transfer (24 bytes) takes about 0.1 ms. Interrupts may stretch a
// clock period, and SPI doesn't mind.
//
// One device per instance. There is no bus mutex: bit-banged pins belong to
// one chip.
template <typename TPins>
class BitBangSpiBlockTransport : public iBlockTransport {
public:
    template <typename... TArgs>
    explicit BitBangSpiBlockTransport(TArgs&&... pinArgs) : pins_(static_cast<TArgs&&>(pinArgs)...) {
        pins_.ckLow();
        pins_.csHigh();
    }

    bool beginWrite(const uint8_t* header, uint8_t headerLen, const uint8_t* data, size_t len) override {
        if (!valid(header, headerLen, data, len)) return false;
        pins_.csLow();
        pins_.delay();
        for (uint8_t i = 0; i < headerLen; ++i) xfer(header[i]);
        for (size_t i = 0; i < len; ++i) xfer(data[i]);
        end();
        return true;
    }

    // The bytes clocked in while the header goes out are dropped, as
    // iBlockTransport says. fill_ goes out while the data comes in.
    bool beginRead(const uint8_t* header, uint8_t headerLen, uint8_t* data, size_t len) override {
        if (!valid(header, headerLen, data, len)) return false;
        pins_.csLow();
        pins_.delay();
        for (uint8_t i = 0; i < headerLen; ++i) (void)xfer(header[i]);
        for (size_t i = 0; i < len; ++i) data[i] = xfer(fill_);
        end();
        return true;
    }

    bool isBusy() const override { return false; }
    bool lastOpFailed() const override { return false; }

    void setFill(uint8_t b) { fill_ = b; }

    // The chip's DO, read with CS high: for a chip that signals there, as
    // the HY3131 does its interrupt. (A driver that inherits its transport
    // protected, like hy3131_dmm, can't be asked for this from outside; give
    // hy3131_param_t::doLevel the pins' own reader instead. See
    // Stm32L1GpioSpiPins::doLevelOf.)
    bool doLevel() const { return pins_.doLevel(); }

    TPins& pins() { return pins_; }

private:
    static bool valid(const uint8_t* header, uint8_t headerLen, const void* data, size_t len) {
        if (headerLen > kMaxHeaderLen || len > kMaxDataLen) return false;
        if (headerLen == 0 && len == 0) return false;
        if ((headerLen != 0 && header == nullptr) || (len != 0 && data == nullptr)) return false;
        return true;
    }

    // Mode 0: DI set while CK is low, the chip latches it as CK rises; DO
    // read while CK is high (the chip changed it as CK last fell).
    uint8_t xfer(uint8_t out) {
        uint8_t in = 0;
        for (int b = 7; b >= 0; --b) {
            pins_.di(((out >> b) & 1u) != 0);
            pins_.delay();
            pins_.ckHigh();
            pins_.delay();
            in = (uint8_t)((in << 1) | (pins_.doLevel() ? 1u : 0u));
            pins_.ckLow();
        }
        return in;
    }

    void end() {
        pins_.delay();
        pins_.csHigh();
        pins_.di(true);   // idle high, as the 121GW's own firmware leaves it
    }

    TPins   pins_;
    uint8_t fill_ = 0xFF;
};
