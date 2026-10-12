#pragma once
#include <cstddef>
#include <cstdint>

// A transport for chips that frame every access as a short header —
// command, address and control bytes — followed by a block of data
// of any length, all under one chip-select:
//
//   WIZnet W5500       3-byte header (address, control), then the data
//   WIZnet W5100/W5200 4-byte header
//   ENC28J60, SPI flash, FRAM, ATWINC1500, ...
//
// ISensorTransport doesn't fit these: its register address is one
// byte and a transfer is at most kMaxWriteLen bytes, where an
// Ethernet controller moves whole frames to and from its buffer
// memory in one transfer.
//
// Same conventions as ISensorTransport: both calls only *start* the
// transfer and return true once it is issued, false if it was not
// (another transfer in flight, the bus in use elsewhere, or a request
// that can't be carried). Call isBusy() until it is false, then
// lastOpFailed().
//
// The header is copied before the call returns. The data is not:
// `data` must stay valid, and for a read must not be touched, until
// isBusy() returns false — the hardware (DMA, on STM32) works
// straight from and into the caller's buffer.
class iBlockTransport {
public:
    virtual ~iBlockTransport() = default;

    static constexpr uint8_t kMaxHeaderLen = 8;
    static constexpr size_t  kMaxDataLen   = 0xFFFF;

    // Select, send header[0..headerLen), send data[0..len), deselect.
    // headerLen: 0..kMaxHeaderLen. len: 0..kMaxDataLen. Not both 0.
    virtual bool beginWrite(const uint8_t* header, uint8_t headerLen,
                            const uint8_t* data, size_t len) = 0;

    // Select, send header[0..headerLen), clock in len bytes to data,
    // deselect. The bytes clocked in while the header goes out are
    // thrown away. Limits as above.
    virtual bool beginRead(const uint8_t* header, uint8_t headerLen,
                           uint8_t* data, size_t len) = 0;

    // True while the most recent transfer is still in flight.
    virtual bool isBusy() const = 0;

    // Valid once isBusy() == false.
    virtual bool lastOpFailed() const = 0;
};
