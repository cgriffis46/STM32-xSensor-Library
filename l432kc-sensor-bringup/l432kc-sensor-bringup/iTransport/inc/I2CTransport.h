#pragma once
#include "ISensorTransport.h"
#include "BusTransport.h"

// Generic I2C transport: the five ISensorTransport transfer calls and
// checkDevice(), expressed as I2C operations. Bus arbitration, the
// transfer-completion signal and the busy/failed bookkeeping are
// inherited from BusTransport, shared with SPITransport.
//
// Hardware-call-agnostic AND platform/RTOS-agnostic: no RTOS header
// and no STM32 type appears here. Every literal STM32 call
// (HAL_I2C_Mem_Write_IT, HAL_I2C_Master_Transmit_IT, ...) lives in
// Stm32HalI2CTransport, and so does the HAL header itself. That's not
// just tidiness: many STM32Cube HAL versions typedef
// I2C_HandleTypeDef from an ANONYMOUS struct (no tag name), which
// means forward-declaring it here (`struct I2C_HandleTypeDef;`) would
// conflict with the real definition once Stm32HalI2CTransport.h
// includes it — a genuine redefinition error, not just a style
// concern. Storing the bus handle as an opaque void* avoids that
// entirely.
class I2CTransport : public ISensorTransport, public BusTransport {
public:
    // busHandle should be the bus's real HAL handle (e.g. &hi2c1),
    // passed through as void* — the concrete transport casts it back.
    // busMutex is likewise opaque — see BusTransport.
    I2CTransport(void* busHandle, uint8_t deviceAddr7bit, void* busMutex);
    ~I2CTransport() override = default;

    bool writeReg(uint8_t reg, uint8_t value) override;
    bool writeRegs(uint8_t reg, const uint8_t* buf, uint8_t len) override;
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) override;
    bool writeBytes(const uint8_t* buf, uint8_t len) override;
    bool readBytes(uint8_t* buf, uint8_t len) override;
    bool isBusy() const override { return BusTransport::isBusy(); }
    bool lastOpFailed() const override { return BusTransport::lastOpFailed(); }
    bool checkDevice() override;

protected:
    // The actual hardware calls, supplied by a concrete transport
    // (Stm32HalI2CTransport, ArduinoWireTransport, ...). Same
    // non-blocking convention as the public interface above: return
    // true once the operation has been *issued* — isBusy()/
    // lastOpFailed() cover whether it actually landed, separately.
    //
    // halMemWrite()/halMemRead() send the register address first
    // (STM32: HAL_I2C_Mem_Write_IT/HAL_I2C_Mem_Read_IT).
    // halMasterTransmit()/halMasterReceive() send or receive the
    // bytes alone (STM32: HAL_I2C_Master_Transmit_IT/
    // HAL_I2C_Master_Receive_IT). All four must end by calling
    // onTransferComplete(), from the bus interrupt or directly.
    //
    // pData for the two write calls points at this class's own copy
    // (writeBuf_), which stays valid for the whole transfer.
    virtual bool halMemWrite(uint8_t reg, uint8_t* pData, uint16_t size) = 0;
    virtual bool halMemRead(uint8_t reg, uint8_t* pData, uint16_t size) = 0;
    virtual bool halMasterTransmit(uint8_t* pData, uint16_t size) = 0;
    virtual bool halMasterReceive(uint8_t* pData, uint16_t size) = 0;
    virtual bool halIsDeviceReady(uint32_t trials, uint32_t timeout) = 0;

    uint8_t devAddr8bit_; // 7-bit address pre-shifted for HAL's 8-bit convention

private:
    uint8_t writeBuf_[kMaxWriteLen] = {0}; // every write is copied here; must outlive the IT transfer

    // Only ever called between beginTransfer() and the hardware call,
    // so writeBuf_ can't still be in use by an earlier transfer.
    void copyWrite(const uint8_t* buf, uint8_t len);
};
