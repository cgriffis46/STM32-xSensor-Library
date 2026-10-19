#pragma once
#include <cstdint>

// ISensorTransport is the seam between "how we talk to a chip" and
// "what the chip means." Nothing above this interface knows whether
// it's I2C, SPI, or a mock for unit tests.
//
// All operations are non-blocking: every write/read below *starts* a
// transfer and returns immediately. Call isBusy() every tick() to find
// out when it's done, and lastOpFailed() to check the outcome.
//
// Two families of access, because sensors come in two kinds:
//
//   Register-addressed (BMP280, BME280, LPS35HW, MPL3115A2, ...):
//       writeReg()  writeRegs()  readRegs()
//
//   Command-style, with no register address at all (AHT20, SHT31,
//   HTU21DF, Si7021, HMC6352, ...): the chip takes a command of one
//   or more bytes, and is read back as a plain run of bytes.
//       writeBytes()  readBytes()
//
// Every one of the five returns true once the transfer has been
// *issued*. false means it was not started: a transfer is already in
// flight, the bus is in use elsewhere, or the request can't be
// carried (len is 0, or over the limits below). A caller that keeps
// getting false should give up after its own timeout rather than
// retry forever.
class ISensorTransport {
public:
    virtual ~ISensorTransport() = default;

    // Most data bytes one write may carry, not counting the register
    // address. Writes are copied into the transport (see below), so
    // every transport sets aside this much for them.
    static constexpr uint8_t kMaxWriteLen = 32;

    // Begin writing `value` to register `reg`.
    virtual bool writeReg(uint8_t reg, uint8_t value) = 0;

    // Begin writing `len` bytes from `buf` to consecutive registers
    // starting at `reg`, as one bus transaction. len: 1..kMaxWriteLen.
    //
    // The bytes are copied before this returns, here and in
    // writeBytes(), so `buf` may be a local that goes out of scope
    // straight away.
    virtual bool writeRegs(uint8_t reg, const uint8_t* buf, uint8_t len) = 0;

    // Begin reading `len` bytes starting at register `reg` into `buf`.
    // `buf` must stay valid until isBusy() returns false: unlike a
    // write, a read lands in the caller's own buffer.
    virtual bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) = 0;

    // Begin writing `len` bytes exactly as given, with no register
    // address in front. len: 1..kMaxWriteLen. Copied, as above.
    //   I2C: START, address+W, the bytes, STOP.
    //   SPI: chip-select low, the bytes, chip-select high.
    virtual bool writeBytes(const uint8_t* buf, uint8_t len) = 0;

    // Begin reading `len` bytes with no register address sent first.
    // `buf` must stay valid until isBusy() returns false.
    //   I2C: START, address+R, the bytes, STOP.
    //   SPI: chip-select low, `len` bytes clocked in, chip-select high.
    virtual bool readBytes(uint8_t* buf, uint8_t len) = 0;

    // True while the most recent transfer is still in flight.
    virtual bool isBusy() const = 0;

    // Valid once isBusy() == false. True if the last transfer NACKed,
    // timed out, or otherwise failed.
    virtual bool lastOpFailed() const = 0;

    // Confirms a device actually responds on the bus, before any
    // register access is attempted. How this is implemented is
    // entirely bus-specific — hence a virtual here rather than in
    // SensorBase. NOTE: unlike everything else in this interface,
    // this is allowed to block briefly; not every bus (or every HAL)
    // offers a non-blocking way to do it.
    virtual bool checkDevice() = 0;
};
