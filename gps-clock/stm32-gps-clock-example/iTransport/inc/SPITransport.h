#pragma once
#include "ISensorTransport.h"
#include "BusTransport.h"

// Generic SPI transport: the five ISensorTransport transfer calls
// expressed as SPI operations — chip-select, the read/write framing
// (the read/write bit on the address, dummy clock bytes to drive full-duplex
// reads), and the tx/rx scratch buffers. Bus arbitration, the
// transfer-completion signal and the busy/failed bookkeeping are
// inherited from BusTransport, shared with I2CTransport: a transfer
// finishes when the SPI interrupt calls onTransferComplete(), the
// same way an I2C one does, rather than by polling the peripheral.
//
// Every literal STM32 call (HAL_SPI_Transmit_IT,
// HAL_SPI_TransmitReceive_IT, and HAL_GPIO_WritePin for chip-select)
// lives in Stm32HalSPITransport, and so does the HAL header itself —
// this header has no STM32 type in it anywhere, for the same
// anonymous-struct reason given in I2CTransport.h. The SPI handle and
// CS port are stored as opaque void*: this class never dereferences
// them, only passes them through to the hal*() methods.
class SPITransport : public ISensorTransport, public BusTransport {
public:
    // spiHandle should be the bus's real HAL handle (e.g. &hspi1) and
    // csPort the real GPIO port (e.g. GPIOB), both passed through as
    // void* — Stm32HalSPITransport casts them back. busMutex: one per
    // SPI bus, the same value for every device on it — see
    // BusTransport. The bus is taken BEFORE this device's chip-select
    // goes low, so two devices are never selected at once.
    SPITransport(void* spiHandle, void* csPort, uint16_t csPin, void* busMutex);
    ~SPITransport() override = default;

    // Which way round bit 7 of a register address says read or write.
    // Bosch, ST and most sensors set it to read (ReadHigh, the default).
    // Semtech's radios (SX1231 / RFM69, SX127x / RFM9x) set it to write.
    // writeRegs()/readRegs() take the plain register number either way
    // and put the bit on themselves; writeBytes()/readBytes() send no
    // address and are not affected.
    enum class AddressBit : uint8_t { ReadHigh, WriteHigh };
    void setAddressBit(AddressBit b) { addressBit_ = b; }
    AddressBit addressBit() const { return addressBit_; }

    bool writeReg(uint8_t reg, uint8_t value) override;
    bool writeRegs(uint8_t reg, const uint8_t* buf, uint8_t len) override;
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) override;
    bool writeBytes(const uint8_t* buf, uint8_t len) override;
    bool readBytes(uint8_t* buf, uint8_t len) override;
    bool isBusy() const override { return BusTransport::isBusy(); }
    bool lastOpFailed() const override { return BusTransport::lastOpFailed(); }

    // SPI has no ACK-style bus signal the way I2C does, so there's no
    // real equivalent to HAL_I2C_IsDeviceReady() here. This always
    // returns true — actual presence-checking for an SPI sensor has
    // to happen at the sensor level (e.g. read a chip-ID register and
    // compare against the datasheet's expected value).
    bool checkDevice() override;

protected:
    // The actual hardware calls, supplied by Stm32HalSPITransport (or
    // any other concrete transport for a different HAL/platform).
    // halTransmit()/halTransmitReceive() return true once *issued*,
    // and must end by calling onTransferComplete(), from the SPI
    // interrupt or directly (STM32: HAL_SPI_TxCpltCallback,
    // HAL_SPI_TxRxCpltCallback and HAL_SPI_ErrorCallback, in
    // Stm32SpiItCallbacks.cpp).
    virtual bool halTransmit(uint8_t* txBuf, uint16_t len) = 0;
    virtual bool halTransmitReceive(uint8_t* txBuf, uint8_t* rxBuf, uint16_t len) = 0;
    virtual void halCsLow() = 0;
    virtual void halCsHigh() = 0;

    // Raises chip-select and, after a read, copies the received bytes
    // into the caller's buffer. Runs in the waiting thread, from
    // isBusy(), while the bus is still held.
    void onTransferLanded() override;

    void*    csPort_;
    uint16_t csPin_;

private:
    static constexpr uint8_t kRwBit = 0x80;    // bit 7 of the address: read or write, see AddressBit
    static constexpr uint8_t kMaxLen  = 1 + kMaxWriteLen; // 1 addr byte + up to 32 data bytes

    // Shared tail of all five transfer calls, entered with the bus
    // already held (beginTransfer()): chip-select low, issue whatever
    // is in txBuf_[0..xferLen_), and back out again if the hardware
    // call didn't start. A read uses full-duplex transmit+receive; a
    // write transmits only.
    bool start(bool isRead);

    uint8_t  txBuf_[kMaxLen] = {0};
    uint8_t  rxBuf_[kMaxLen] = {0};
    uint8_t  xferLen_ = 0;
    uint8_t  rxSkip_  = 0;       // leading rxBuf_ bytes that aren't data: 1 after readRegs()
                                 // (clocked in while the address went out), 0 after readBytes()
    uint8_t* destBuf_ = nullptr; // caller's buffer, filled in onTransferLanded() after a read
    bool     isRead_  = false;
    AddressBit addressBit_ = AddressBit::ReadHigh;
};
