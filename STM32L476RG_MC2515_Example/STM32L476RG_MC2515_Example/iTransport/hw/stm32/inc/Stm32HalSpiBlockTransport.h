#pragma once
// The project's own main.h, not a specific family's _hal.h — see
// Stm32HalI2CTransport.h.
#include "main.h"
#include "SpiBlockTransport.h"
#include "FreeRtosTransport.h"

// The literal STM32 HAL calls behind SpiBlockTransport — the class you
// construct for a W5500 (or any header-then-block SPI chip):
//
//   Stm32HalSpiBlockTransport spi(&hspi1, W5500_CS_GPIO_Port, W5500_CS_Pin, spi1Mutex);
//
// Completes through the same HAL callbacks as Stm32HalSPITransport
// (Stm32SpiItCallbacks.cpp), and like it derives from
// FreeRtosTransport<> to take a real bus mutex and to sleep until the
// SPI interrupt, rather than poll.
//
// DMA or interrupt, per call: a block of dmaThreshold bytes or more
// goes by DMA when CubeMX has linked DMA channels to the SPI handle
// (hspi->hdmatx, and hdmarx for reads), anything shorter — and
// everything, when it hasn't — by interrupt. Link the DMA channels for
// an Ethernet chip: at a useful SPI clock the interrupt-per-byte path
// cannot keep up with a 2 KB frame (STM32L432: SPI1_RX on DMA1
// channel 2, SPI1_TX on DMA1 channel 3, both "Normal" mode, byte
// width). The SPI global interrupt must be enabled either way, and
// the DMA channel interrupts too when DMA is used.
//
// Reads use HAL_SPI_Receive_*(), which on a full-duplex master clocks
// out the receive buffer's own contents as it goes; a W5500 ignores
// MOSI during the data phase of a read. It completes through
// HAL_SPI_RxCpltCallback, which Stm32SpiItCallbacks.cpp provides.
class Stm32HalSpiBlockTransport : public FreeRtosTransport<SpiBlockTransport> {
public:
    Stm32HalSpiBlockTransport(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin,
                              osMutexId_t busMutex, uint16_t dmaThreshold = 16)
        : FreeRtosTransport<SpiBlockTransport>(hspi, csPort, csPin, busMutex),
          dmaThreshold_(dmaThreshold) {
        halCsHigh(); // idle deselected — safe here, in this class's own constructor
    }

protected:
    bool halTransmit(uint8_t* tx, uint16_t len) override;
    bool halReceive(uint8_t* rx, uint16_t len) override;
    void halCsLow() override;
    void halCsHigh() override;

private:
    // Bytes left in the receive FIFO by a transmit-only call (the
    // header, on families with a FIFO such as the L4) would otherwise
    // be taken as the first bytes of the next receive.
    void flushRx();

    SPI_HandleTypeDef* hspi() const { return static_cast<SPI_HandleTypeDef*>(busHandle_); }
    GPIO_TypeDef*      csGpio() const { return static_cast<GPIO_TypeDef*>(csPort_); }

    uint16_t dmaThreshold_;
};
