#pragma once
#include <type_traits>
#include "stm32l1xx.h"
#include "SpiBlockTransport.h"
#include "Stm32L1SpiPort.h"

// SpiBlockTransport on an STM32L1 SPI, through Stm32L1SpiPort (register
// level, interrupt per byte, no HAL SPI). For the HY3131, a W5500, an SD
// card in SPI mode: any header-then-block chip.
//
// TBus picks how a thread waits:
//
//   // FreeRTOS / CMSIS-RTOS2: sleeps on a thread flag until the interrupt
//   Stm32L1SpiBlockTransport<FreeRtosTransport<SpiBlockTransport>> dmmSpi(spi2, DMM_CS_GPIO_Port, DMM_CS_Pin, spi2Mutex);
//
//   // bare metal: BusTransport's polling defaults, busMutex nullptr
//   Stm32L1SpiBlockTransport<SpiBlockTransport> dmmSpi(spi2, DMM_CS_GPIO_Port, DMM_CS_Pin, nullptr);
//
// A driver takes it as its TTransport, port first in the arguments:
//
//   xhy3131<Stm32L1SpiBlockTransport<FreeRtosTransport<SpiBlockTransport>>>
//       dmm(param, spi2, DMM_CS_GPIO_Port, DMM_CS_Pin, spi2Mutex);
//
// Every device on one SPI shares one Stm32L1SpiPort and one busMutex.
// Chip-select is written through BSRR (one store, safe from an interrupt).
// Before it goes up, the port waits for BSY to clear.
template <typename TBus>
class Stm32L1SpiBlockTransport : public TBus {
    static_assert(std::is_base_of<SpiBlockTransport, TBus>::value,
                  "TBus must be SpiBlockTransport, or FreeRtosTransport<SpiBlockTransport>");

public:
    Stm32L1SpiBlockTransport(Stm32L1SpiPort& port, GPIO_TypeDef* csPort, uint16_t csPin, void* busMutex)
        : TBus(port.instance(), csPort, csPin, busMutex), port_(port) {
        halCsHigh();   // idle deselected: safe here, in the most derived constructor
    }

protected:
    bool halTransmit(uint8_t* tx, uint16_t len) override { return port_.start(tx, nullptr, len); }
    bool halReceive(uint8_t* rx, uint16_t len) override  { return port_.start(nullptr, rx, len); }
    void halCsLow() override  { gpio()->BSRR = (uint32_t)this->csPin_ << 16; }
    void halCsHigh() override {
        port_.waitIdle();
        gpio()->BSRR = (uint32_t)this->csPin_;
    }

private:
    GPIO_TypeDef* gpio() const { return static_cast<GPIO_TypeDef*>(this->csPort_); }

    Stm32L1SpiPort& port_;
};
