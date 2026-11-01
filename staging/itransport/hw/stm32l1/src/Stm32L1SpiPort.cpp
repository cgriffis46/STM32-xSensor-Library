#include "Stm32L1SpiPort.h"
#include "BusTransport.h"

void Stm32L1SpiPort::configure(Mode mode, uint8_t baudDiv) {
    spi_->CR1 &= ~SPI_CR1_SPE;
    spi_->CR2 = 0;
    uint32_t cr1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
                 | ((uint32_t)(baudDiv & 7u) << SPI_CR1_BR_Pos);
    if ((uint8_t)mode & 2u) cr1 |= SPI_CR1_CPOL;
    if ((uint8_t)mode & 1u) cr1 |= SPI_CR1_CPHA;
    spi_->CR1 = cr1;
    spi_->CR1 = cr1 | SPI_CR1_SPE;
}

bool Stm32L1SpiPort::start(const uint8_t* tx, uint8_t* rx, uint16_t len) {
    if (len == 0 || active_) return false;

    // Whatever an earlier transfer, or the HAL, left behind: a byte in
    // DR and maybe OVR. Reading DR then SR clears OVR (RM0038 30.4.8).
    for (int i = 0; i < 4 && (spi_->SR & SPI_SR_RXNE) != 0U; ++i) (void)spi_->DR;
    if ((spi_->SR & SPI_SR_OVR) != 0U) { (void)spi_->DR; (void)spi_->SR; }
    // MODF clears by reading SR then writing CR1; with SSM/SSI set it can't recur.
    if ((spi_->SR & SPI_SR_MODF) != 0U) { spi_->CR1 = spi_->CR1 | SPI_CR1_MSTR; }

    tx_  = tx;
    rx_  = rx;
    len_ = len;
    pos_ = 0;
    active_ = true;

    spi_->CR1 = spi_->CR1 | SPI_CR1_SPE;
    spi_->CR2 = spi_->CR2 | SPI_CR2_RXNEIE | SPI_CR2_ERRIE;
    spi_->DR  = tx ? tx[0] : fill_;   // TXE is set: nothing else is in flight
    return true;
}

void Stm32L1SpiPort::irq() {
    const uint32_t sr = spi_->SR;

    if ((sr & (SPI_SR_OVR | SPI_SR_MODF)) != 0U) {
        (void)spi_->DR;
        (void)spi_->SR;
        if ((sr & SPI_SR_MODF) != 0U) spi_->CR1 = spi_->CR1 | SPI_CR1_MSTR | SPI_CR1_SPE;
        errors_ = errors_ + 1;
        if (active_) finish(true);
        else spi_->CR2 = spi_->CR2 & ~(SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
        return;
    }

    if ((sr & SPI_SR_RXNE) == 0U) return;
    const uint8_t b = (uint8_t)spi_->DR;   // clears RXNE
    if (!active_) {                         // not ours: stand down
        spi_->CR2 = spi_->CR2 & ~(SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
        return;
    }
    const uint16_t p = pos_;
    if (rx_) rx_[p] = b;
    const uint16_t next = (uint16_t)(p + 1);
    pos_ = next;
    if (next < len_) {
        spi_->DR = tx_ ? tx_[next] : fill_;
    } else {
        finish(false);
    }
}

// The port is free again BEFORE the transport is told: SpiBlockTransport
// starts its data phase from inside onTransferComplete(), through start().
void Stm32L1SpiPort::finish(bool failed) {
    spi_->CR2 = spi_->CR2 & ~(SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
    active_ = false;
    BusTransport::onTransferComplete(spi_, failed);
}

// RXNE of the last byte comes as its last bit is sampled; SCK has half
// a bit still to go, and BSY covers it. At the slowest clock
// (PCLK/256) that is 128 PCLKs; the bound is far above that.
void Stm32L1SpiPort::waitIdle() const {
    for (uint32_t i = 0; i < 10000u && (spi_->SR & SPI_SR_BSY) != 0U; ++i) {}
}
