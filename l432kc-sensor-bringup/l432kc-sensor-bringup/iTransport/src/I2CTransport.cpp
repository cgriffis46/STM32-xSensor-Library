#include "I2CTransport.h"

I2CTransport::I2CTransport(void* busHandle, uint8_t deviceAddr7bit, void* busMutex)
    : BusTransport(busHandle, busMutex), devAddr8bit_(static_cast<uint8_t>(deviceAddr7bit << 1)) {}

void I2CTransport::copyWrite(const uint8_t* buf, uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) writeBuf_[i] = buf[i];
}

bool I2CTransport::writeReg(uint8_t reg, uint8_t value) {
    return writeRegs(reg, &value, 1);
}

bool I2CTransport::writeRegs(uint8_t reg, const uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0 || len > kMaxWriteLen) return false;
    if (!beginTransfer()) return false;
    copyWrite(buf, len);
    return endIssue(halMemWrite(reg, writeBuf_, len));
}

bool I2CTransport::readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0) return false;
    if (!beginTransfer()) return false;
    return endIssue(halMemRead(reg, buf, len));
}

bool I2CTransport::writeBytes(const uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0 || len > kMaxWriteLen) return false;
    if (!beginTransfer()) return false;
    copyWrite(buf, len);
    return endIssue(halMasterTransmit(writeBuf_, len));
}

bool I2CTransport::readBytes(uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0) return false;
    if (!beginTransfer()) return false;
    return endIssue(halMasterReceive(buf, len));
}

// checkDevice() still has to take the mutex first so it can't collide
// with another task's in-flight transfer on the same bus — the actual
// "is anyone there" check is halIsDeviceReady() (Stm32HalI2CTransport),
// which may itself be a blocking HAL call; that's the concrete
// class's business, not this one's.
bool I2CTransport::checkDevice() {
    if (!ObtainMutex(kMutexTimeoutTicks)) {
        failed_ = true;
        return false;
    }
    const bool ready = halIsDeviceReady(/*Trials=*/2, /*Timeout=*/10);
    ReleaseMutex();
    failed_ = !ready;
    return ready;
}
