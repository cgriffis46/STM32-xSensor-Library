#include "Stm32HalSpiBlockTransport.h"

void Stm32HalSpiBlockTransport::halCsLow()  { HAL_GPIO_WritePin(csGpio(), csPin_, GPIO_PIN_RESET); }
void Stm32HalSpiBlockTransport::halCsHigh() { HAL_GPIO_WritePin(csGpio(), csPin_, GPIO_PIN_SET); }

// Both are called from thread context for the first phase of a
// transfer and from the SPI (or DMA) interrupt for the second. The
// HAL has set the handle back to READY before it calls the completion
// callback, so starting the next transfer from there is allowed.
bool Stm32HalSpiBlockTransport::halTransmit(uint8_t* tx, uint16_t len) {
    if (len >= dmaThreshold_ && hspi()->hdmatx != nullptr) {
        return HAL_SPI_Transmit_DMA(hspi(), tx, len) == HAL_OK;
    }
    return HAL_SPI_Transmit_IT(hspi(), tx, len) == HAL_OK;
}

bool Stm32HalSpiBlockTransport::halReceive(uint8_t* rx, uint16_t len) {
    flushRx();
    // A full-duplex master's receive is a transmit-and-receive
    // underneath, so DMA needs both channels.
    if (len >= dmaThreshold_ && hspi()->hdmarx != nullptr && hspi()->hdmatx != nullptr) {
        return HAL_SPI_Receive_DMA(hspi(), rx, len) == HAL_OK;
    }
    return HAL_SPI_Receive_IT(hspi(), rx, len) == HAL_OK;
}

// At most a FIFO's worth (4 bytes on the L4); bounded anyway. An 8-bit
// read of DR takes one byte out of the FIFO.
void Stm32HalSpiBlockTransport::flushRx() {
    SPI_TypeDef* spi = hspi()->Instance;
    for (int i = 0; i < 8 && (spi->SR & SPI_SR_RXNE) != 0U; ++i) {
        (void)*reinterpret_cast<volatile uint8_t*>(&spi->DR);
    }
    __HAL_SPI_CLEAR_OVRFLAG(hspi());
}
