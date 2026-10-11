#pragma once
// The project's own main.h, not a specific family's _hal.h: CubeMX's
// generated main.h already includes the right HAL header for
// whichever STM32 family the project targets (same approach, for the
// same reason, as Stm32HalUartTransport.h). The HAL header lives
// here, not in I2CTransport.h — see the comment there on why an
// anonymous-struct HAL typedef makes it unsafe to forward-declare.
#include "main.h"
#include "I2CTransport.h"
#include "FreeRtosTransport.h"

// The literal STM32 HAL_I2C_* calls — all the interrupt-driven (_IT)
// variants, completing through Stm32I2CItCallbacks.cpp — isolated here so
// I2CTransport's generic logic doesn't depend on which specific HAL
// function signature a given STM32 family or HAL version uses. This
// is the class you actually construct — I2CTransport itself is
// abstract. Derives from FreeRtosTransport<I2CTransport> (not
// I2CTransport directly) to get a real bus mutex and to be woken by
// the I2C interrupt rather than polling.
//
// I2C event and error interrupts must be enabled for the peripheral
// in CubeMX. Create busMutex with osMutexNew() before constructing this.
class Stm32HalI2CTransport : public FreeRtosTransport<I2CTransport> {
public:
    Stm32HalI2CTransport(I2C_HandleTypeDef* hi2c, uint8_t deviceAddr7bit, osMutexId_t busMutex)
        : FreeRtosTransport<I2CTransport>(hi2c, deviceAddr7bit, busMutex) {}

protected:
    bool halMemWrite(uint8_t reg, uint8_t* pData, uint16_t size) override;
    bool halMemRead(uint8_t reg, uint8_t* pData, uint16_t size) override;
    bool halMasterTransmit(uint8_t* pData, uint16_t size) override;
    bool halMasterReceive(uint8_t* pData, uint16_t size) override;
    bool halIsDeviceReady(uint32_t trials, uint32_t timeout) override;

private:
    // busHandle_ (inherited, protected, void*) always actually holds
    // an I2C_HandleTypeDef* here — this is the one place that's true
    // and the one place it's safe to assume, since only this class
    // constructs an I2CTransport with an I2C_HandleTypeDef* to begin with.
    I2C_HandleTypeDef* hi2c() const { return static_cast<I2C_HandleTypeDef*>(busHandle_); }
};
