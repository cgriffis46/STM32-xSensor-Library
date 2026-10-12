#include "SPITransport.h"

// NOTE: deliberately NOT calling halCsHigh() here to idle the line —
// it's pure virtual, and calling a pure virtual from a base class's
// own constructor is undefined behavior (the derived class's vtable
// isn't wired up yet at this point). Stm32HalSPITransport's own
// constructor handles the idle-high setup instead, where calling its
// own override is safe.
SPITransport::SPITransport(void* spiHandle, void* csPort, uint16_t csPin, void* busMutex)
    : BusTransport(spiHandle, busMutex), csPort_(csPort), csPin_(csPin) {}

bool SPITransport::start(bool isRead) {
    isRead_ = isRead;

    halCsLow();
    const bool issued = isRead ? halTransmitReceive(txBuf_, rxBuf_, xferLen_)
                               : halTransmit(txBuf_, xferLen_);
    if (!issued) halCsHigh(); // failed to even start — don't leave CS asserted
    return endIssue(issued);
}

// Each call below fills txBuf_ only AFTER beginTransfer() succeeds:
// by then this instance has no transfer in flight, so the buffer is
// free to overwrite.

bool SPITransport::writeReg(uint8_t reg, uint8_t value) {
    return writeRegs(reg, &value, 1);
}

bool SPITransport::writeRegs(uint8_t reg, const uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0 || len > kMaxWriteLen) return false;
    if (!beginTransfer()) return false;

    xferLen_  = static_cast<uint8_t>(len + 1);
    txBuf_[0] = addressBit_ == AddressBit::WriteHigh ? static_cast<uint8_t>(reg | kRwBit)
                                                     : static_cast<uint8_t>(reg & ~kRwBit);
    for (uint8_t i = 0; i < len; ++i) txBuf_[i + 1] = buf[i];
    return start(/*isRead=*/false);
}

bool SPITransport::readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0 || len > kMaxWriteLen) return false; // stays within scratch buffers
    if (!beginTransfer()) return false;

    destBuf_ = buf;
    rxSkip_  = 1; // rxBuf_[0] captured garbage while we clocked out the address byte
    xferLen_ = static_cast<uint8_t>(len + 1);

    txBuf_[0] = addressBit_ == AddressBit::WriteHigh ? static_cast<uint8_t>(reg & ~kRwBit)
                                                     : static_cast<uint8_t>(reg | kRwBit);
    for (uint8_t i = 1; i < xferLen_; ++i) txBuf_[i] = 0xFF; // dummy bytes to drive the clock
    return start(/*isRead=*/true);
}

// No address byte and no read/write bit: the bytes go out exactly as
// given, for chips that take a command rather than a register address.
bool SPITransport::writeBytes(const uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0 || len > kMaxWriteLen) return false;
    if (!beginTransfer()) return false;

    xferLen_ = len;
    for (uint8_t i = 0; i < len; ++i) txBuf_[i] = buf[i];
    return start(/*isRead=*/false);
}

bool SPITransport::readBytes(uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0 || len > kMaxWriteLen) return false;
    if (!beginTransfer()) return false;

    destBuf_ = buf;
    rxSkip_  = 0; // nothing went out first, so every byte clocked in is data
    xferLen_ = len;
    for (uint8_t i = 0; i < xferLen_; ++i) txBuf_[i] = 0xFF;
    return start(/*isRead=*/true);
}

void SPITransport::onTransferLanded() {
    halCsHigh();
    if (isRead_ && !failed_) {
        for (uint8_t i = 0; i + rxSkip_ < xferLen_; ++i) {
            destBuf_[i] = rxBuf_[i + rxSkip_];
        }
    }
}

bool SPITransport::checkDevice() {
    // No bus-level presence signal exists for SPI. A sensor using this
    // transport that wants a real check should read its chip-ID
    // register (e.g. BMP280 0xD0 == 0x58) and compare it itself.
    return true;
}
