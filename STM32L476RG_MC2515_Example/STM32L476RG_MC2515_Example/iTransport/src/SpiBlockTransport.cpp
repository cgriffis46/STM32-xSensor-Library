#include "SpiBlockTransport.h"

// As in SPITransport, chip-select is not idled here: halCsHigh() is
// pure virtual until the derived constructor runs.
SpiBlockTransport::SpiBlockTransport(void* spiHandle, void* csPort, uint16_t csPin, void* busMutex)
    : BusTransport(spiHandle, busMutex), csPort_(csPort), csPin_(csPin) {}

bool SpiBlockTransport::beginWrite(const uint8_t* header, uint8_t headerLen,
                                   const uint8_t* data, size_t len) {
    // The HAL only reads from a transmit buffer.
    return start(header, headerLen, const_cast<uint8_t*>(data), len, /*isRead=*/false);
}

bool SpiBlockTransport::beginRead(const uint8_t* header, uint8_t headerLen,
                                  uint8_t* data, size_t len) {
    return start(header, headerLen, data, len, /*isRead=*/true);
}

bool SpiBlockTransport::start(const uint8_t* header, uint8_t headerLen,
                              uint8_t* data, size_t len, bool isRead) {
    if (headerLen > kMaxHeaderLen || len > kMaxDataLen) return false;
    if (headerLen == 0 && len == 0) return false;
    if ((headerLen != 0 && header == nullptr) || (len != 0 && data == nullptr)) return false;
    if (!beginTransfer()) return false;

    // Only now, with nothing of ours in flight, are the members free.
    for (uint8_t i = 0; i < headerLen; ++i) header_[i] = header[i];
    data_     = data;
    dataLen_  = static_cast<uint16_t>(len);
    isRead_   = isRead;
    inHeader_ = (headerLen != 0);

    halCsLow();
    bool issued;
    if (headerLen != 0) {
        issued = halTransmit(header_, headerLen);
    } else {
        issued = isRead ? halReceive(data_, dataLen_) : halTransmit(data_, dataLen_);
    }
    if (!issued) {
        inHeader_ = false;
        halCsHigh();
    }
    return endIssue(issued);
}

// Interrupt context. inHeader_ is cleared before the data phase is
// started: a hal*() that completes synchronously calls back in here
// from inside itself, and that second call must finish the transfer.
bool SpiBlockTransport::ContinueTransfer(bool& failed) {
    if (!inHeader_) return false;
    inHeader_ = false;
    if (failed || dataLen_ == 0) return false;

    const bool issued = isRead_ ? halReceive(data_, dataLen_) : halTransmit(data_, dataLen_);
    if (!issued) {
        failed = true;
        return false;
    }
    return true;
}

void SpiBlockTransport::onTransferLanded() {
    halCsHigh();
}
