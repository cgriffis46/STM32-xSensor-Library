#include "OneWireUartTransport.h"

OneWireUartTransport::OneWireUartTransport(void* uartHandle, void* busMutex)
    : BusTransport(uartHandle, busMutex) {}

void OneWireUartTransport::onUartTxComplete(void* uartHandle) { BusTransport::onBusEvent(uartHandle, EvTxDone); }
void OneWireUartTransport::onUartRxComplete(void* uartHandle) { BusTransport::onBusEvent(uartHandle, EvRxDone); }
void OneWireUartTransport::onUartError(void* uartHandle)      { BusTransport::onBusEvent(uartHandle, EvError); }

// Interrupt context. The operation is over when the last character
// has both gone out and come back, or as soon as the UART reports an
// error. Signals exactly once per operation.
void OneWireUartTransport::HandleBusEvent(uint8_t event) {
    if (signalled_) return;

    bool failed = false;
    if (event == EvTxDone)      txDone_ = true;
    else if (event == EvRxDone) rxDone_ = true;
    else                        failed = true;

    if (failed || (txDone_ && rxDone_)) {
        signalled_ = true;
        SignalTransferComplete(failed);
    }
}

bool OneWireUartTransport::start(Op op, uint32_t baud, uint16_t n) {
    op_        = op;
    slots_     = n;
    txDone_    = false;
    rxDone_    = false;
    signalled_ = false;

    if (baud != baud_) {
        if (!halSetBaud(baud)) {
            baud_ = 0; // whatever it is now, it isn't known
            return endIssue(false);
        }
        baud_ = baud;
    }

    // Receive armed first, so the echo of the first character can't be missed.
    bool issued = halReceive(rxSlots_, n);
    if (issued && !halTransmit(txSlots_, n)) {
        halAbort(); // don't leave the receive half armed
        issued = false;
    }
    return endIssue(issued);
}

bool OneWireUartTransport::reset() {
    if (!beginTransfer()) return false;

    presence_   = false;
    txSlots_[0] = kResetChar;
    return start(OpReset, kResetBaud, 1);
}

// txSlots_ is filled only AFTER beginTransfer() succeeds: by then this
// instance has nothing in flight, so the buffer is free to overwrite.
bool OneWireUartTransport::writeBytes(const uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0 || len > kMaxLen) return false;
    if (!beginTransfer()) return false;

    for (uint8_t b = 0; b < len; ++b) {
        for (uint8_t bit = 0; bit < 8; ++bit) { // least significant bit first
            txSlots_[b * 8u + bit] = (buf[b] & (1u << bit)) ? kSlotOne : kSlotZero;
        }
    }
    return start(OpWrite, kDataBaud, static_cast<uint16_t>(len * 8u));
}

bool OneWireUartTransport::readBytes(uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0 || len > kMaxLen) return false;
    if (!beginTransfer()) return false;

    destBuf_ = buf;
    destLen_ = len;
    const uint16_t n = static_cast<uint16_t>(len * 8u);
    for (uint16_t i = 0; i < n; ++i) txSlots_[i] = kSlotOne;
    return start(OpRead, kDataBaud, n);
}

// Waiting thread, from isBusy(), with the bus still held. Turns what
// came back on the line into the operation's result.
void OneWireUartTransport::onTransferLanded() {
    if (failed_) {
        halAbort(); // one half may still be running; stop it before the bus is released
        return;
    }

    switch (op_) {
    case OpReset:
        if (rxSlots_[0] == 0x00) {
            failed_ = true;   // line never came back up: shorted, or no pull-up
        } else {
            presence_ = (rxSlots_[0] != kResetChar); // a device changed what we sent
        }
        break;

    case OpWrite:
        // Nothing else drives the line during a write, so every
        // character must read back exactly as sent.
        for (uint16_t i = 0; i < slots_; ++i) {
            if (rxSlots_[i] != txSlots_[i]) { failed_ = true; break; }
        }
        break;

    case OpRead:
        for (uint8_t b = 0; b < destLen_; ++b) {
            uint8_t value = 0;
            for (uint8_t bit = 0; bit < 8; ++bit) {
                if (rxSlots_[b * 8u + bit] & 0x01u) value |= static_cast<uint8_t>(1u << bit);
            }
            destBuf_[b] = value;
        }
        break;
    }
}
