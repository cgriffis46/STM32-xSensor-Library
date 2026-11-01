#pragma once
#include <cstddef>
#include <cstdint>
#include "stm32l1xx.h"   // CMSIS device header: SPI_TypeDef and the SPI_* bits (STM32L152xD etc. defined by the project)

// One STM32L1 SPI peripheral, driven at register level by its own
// interrupt: no HAL SPI driver underneath. One byte is in flight at a
// time. Each RXNE interrupt takes the byte that came in and writes the
// next one. So a transfer can't overrun, and the interrupt never
// busy-waits.
//
// Why not the HAL? The L1's SPI has no FIFO and no data-size field, so
// the register path is short. The HAL's interrupt-mode completion also
// waits for BSY inside the interrupt (SPI_EndRxTxTransaction, with
// HAL_GetTick() timeouts). Here nothing waits in the interrupt. The
// wait for BSY to clear happens in the thread, just before chip-select
// goes up (waitIdle()).
//
//   Stm32L1SpiPort spi2(SPI2);
//   spi2.configure(Stm32L1SpiPort::Mode::Mode0, 3);   // PCLK/16. Or keep CubeMX's MX_SPI2_Init().
//   NVIC_SetPriority(SPI2_IRQn, 6); NVIC_EnableIRQ(SPI2_IRQn);
//   extern "C" void SPI2_IRQHandler(void) { spi2.irq(); }
//
// The SPI interrupt must come here and NOT to HAL_SPI_IRQHandler: don't
// tick "SPI2 global interrupt" in CubeMX, or delete its handler. Clocks
// and the pins' alternate functions are the project's (CubeMX).
//
// A transfer ends with BusTransport::onTransferComplete(instance, failed).
// The bus handle is the SPI_TypeDef* (SPI1, SPI2, SPI3), not a HAL handle.
// Every transport on this port must be constructed with
// Stm32L1SpiBlockTransport, which passes it.
class Stm32L1SpiPort {
public:
    enum class Mode : uint8_t { Mode0 = 0, Mode1 = 1, Mode2 = 2, Mode3 = 3 };   // CPOL << 1 | CPHA

    explicit Stm32L1SpiPort(SPI_TypeDef* spi) : spi_(spi) {}

    Stm32L1SpiPort(const Stm32L1SpiPort&) = delete;
    Stm32L1SpiPort& operator=(const Stm32L1SpiPort&) = delete;

    // Master, 8 bit, MSB first, software NSS, enabled. baudDiv: BR[2:0],
    // SCK = PCLK / (2 << baudDiv). Optional: a peripheral that CubeMX
    // has set up already works as it is (SPE is set at the first transfer).
    void configure(Mode mode, uint8_t baudDiv);

    // Starts len bytes. tx: what to send (nullptr: the fill byte). rx:
    // where the bytes received go (nullptr: dropped). False if one is
    // in flight already or len is 0. Thread or interrupt context
    // (SpiBlockTransport starts the data phase from the interrupt).
    bool start(const uint8_t* tx, uint8_t* rx, uint16_t len);

    // From SPIx_IRQHandler.
    void irq();

    // Waits for the last byte to finish leaving the shifter (BSY clear),
    // bounded. For just before chip-select goes up.
    void waitIdle() const;

    bool active() const { return active_; }
    SPI_TypeDef* instance() const { return spi_; }

    // What a receive clocks out. 0xFF: what an SD card wants, and what
    // a chip that ignores MOSI in a read doesn't mind.
    void setFill(uint8_t b) { fill_ = b; }

    // Counted for the debug log / a console.
    uint32_t errors() const { return errors_; }

private:
    void finish(bool failed);

    SPI_TypeDef*            spi_;
    const uint8_t*          tx_ = nullptr;
    uint8_t*                rx_ = nullptr;
    volatile uint16_t       len_ = 0;
    volatile uint16_t       pos_ = 0;
    volatile bool           active_ = false;
    uint8_t                 fill_ = 0xFF;
    volatile uint32_t       errors_ = 0;
};
