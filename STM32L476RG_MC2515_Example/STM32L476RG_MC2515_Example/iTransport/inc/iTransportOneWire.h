#pragma once
#include <cstdint>

// The seam between "how we drive a 1-Wire bus" and "what the chip on
// it means" — for 1-Wire what ISensorTransport is for I2C and SPI,
// and in the same style: every call below *starts* an operation and
// returns immediately. Call isBusy() every tick to find out when it
// has finished, and lastOpFailed() for the outcome.
//
// The three operations are the three things a 1-Wire master ever does:
//
//   reset()        the reset pulse that begins every transaction;
//                  presence() then says whether a device answered
//   writeBytes()   ROM and function commands, e.g. CC 44
//   readBytes()    whatever the device returns, e.g. the scratchpad
//
// Each returns true once the operation has been *issued*. false means
// it was not started: an operation is already in flight, the bus is
// in use elsewhere, or len is 0 or over kMaxLen. A caller that keeps
// getting false should give up after its own timeout.
//
// Not derived from iTransport, despite the name: iTransport is a
// stream whose received bytes are pushed to a sink as they arrive.
// 1-Wire is request and response — the master clocks every bit, so
// nothing arrives unasked — and a read here lands in the caller's
// buffer, which suits a driver built as issue/wait state pairs.
//
// One transaction at a time: the bus is taken and released per call,
// not across a reset / command / read sequence. That is all a single
// device addressed with Skip ROM needs. Several drivers sharing one
// wire, each addressing its own device with Match ROM, would also
// need to keep each other out for a whole sequence; nothing here does.
class iTransportOneWire {
public:
    virtual ~iTransportOneWire() = default;

    // Most bytes one writeBytes() or readBytes() may carry. Covers a
    // Match ROM command with its 8-byte ROM code and a function
    // command (10), and a 9-byte scratchpad read.
    static constexpr uint8_t kMaxLen = 16;

    // Begin a reset pulse. Once isBusy() is false and lastOpFailed()
    // is false, presence() says whether any device answered it.
    virtual bool reset() = 0;

    // Begin writing `len` bytes, least significant bit first.
    // len: 1..kMaxLen. The bytes are taken before this returns, so
    // `buf` may be a local.
    virtual bool writeBytes(const uint8_t* buf, uint8_t len) = 0;

    // Begin reading `len` bytes into `buf`. len: 1..kMaxLen. `buf`
    // must stay valid until isBusy() returns false.
    virtual bool readBytes(uint8_t* buf, uint8_t len) = 0;

    // True while the most recent operation is still in flight.
    virtual bool isBusy() const = 0;

    // Valid once isBusy() == false. True if the last operation hit a
    // bus fault: the line held low, a write that didn't read back as
    // sent, or an error from the hardware underneath. An absent device
    // is NOT a failure of reset() — it is presence() == false. (An
    // absent device can't be told from one sending all 1s during
    // readBytes(); that is what the chip's CRC is for.)
    virtual bool lastOpFailed() const = 0;

    // Result of the most recent reset(): true if a device pulled the
    // line low in reply.
    virtual bool presence() const = 0;
};
