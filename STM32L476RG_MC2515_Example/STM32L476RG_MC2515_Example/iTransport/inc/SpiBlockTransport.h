#pragma once
#include "iBlockTransport.h"
#include "BusTransport.h"

// iBlockTransport over SPI. A transfer is two hardware calls under
// one chip-select: the header goes out, and when the SPI interrupt
// says it has, the data phase is started from that same interrupt
// (BusTransport::ContinueTransfer()). The waiting thread is only
// woken once, when the data phase lands.
//
// Two calls rather than one so that neither direction needs a copy:
// a write sends straight from the caller's buffer, and a read
// receives straight into it, with no header-sized gap in front that
// would have to be copied out afterwards. At W5500 frame sizes (up to
// 16 KB) that copy would cost more RAM than the STM32L4 can spare.
//
// Bus arbitration and completion are BusTransport's, as for
// SPITransport: one busMutex per physical SPI bus, shared by every
// transport on it — an SPITransport for a sensor and a
// SpiBlockTransport for an Ethernet chip can share a bus.
//
// The hal*() calls are left for a further-derived class:
// Stm32HalSpiBlockTransport on STM32. As in SPITransport, the handles
// are opaque void* here and never dereferenced.
class SpiBlockTransport : public iBlockTransport, public BusTransport {
public:
    SpiBlockTransport(void* spiHandle, void* csPort, uint16_t csPin, void* busMutex);
    ~SpiBlockTransport() override = default;

    bool beginWrite(const uint8_t* header, uint8_t headerLen,
                    const uint8_t* data, size_t len) override;
    bool beginRead(const uint8_t* header, uint8_t headerLen,
                   uint8_t* data, size_t len) override;
    bool isBusy() const override { return BusTransport::isBusy(); }
    bool lastOpFailed() const override { return BusTransport::lastOpFailed(); }

protected:
    // Return true once *issued*. Each must end in onTransferComplete(),
    // from the SPI interrupt or directly. halTransmit() never writes to
    // `tx`; it is non-const only because older HAL versions take it so.
    // halReceive() clocks out whatever it likes while it receives.
    virtual bool halTransmit(uint8_t* tx, uint16_t len) = 0;
    virtual bool halReceive(uint8_t* rx, uint16_t len) = 0;
    virtual void halCsLow() = 0;
    virtual void halCsHigh() = 0;

    // Interrupt context: the header has gone out; start the data.
    bool ContinueTransfer(bool& failed) override;

    // Raises chip-select. Runs in the waiting thread, from isBusy().
    void onTransferLanded() override;

    void*    csPort_;
    uint16_t csPin_;

private:
    bool start(const uint8_t* header, uint8_t headerLen, uint8_t* data, size_t len, bool isRead);

    uint8_t       header_[kMaxHeaderLen] = {0};
    uint8_t*      data_      = nullptr;
    uint16_t      dataLen_   = 0;
    bool          isRead_    = false;
    volatile bool inHeader_  = false; // header phase in flight; the data phase is still to come
};
