#include "main.h" // the HAL header for this project's STM32 family — see Stm32HalSPITransport.h
#include "SPITransport.h"

// STM32 HAL calls these (weak, overridable) from the SPI interrupt
// handler once an interrupt-driven transfer finishes or faults. Same
// reasoning as Stm32I2CItCallbacks.cpp: HAL callbacks are free C
// functions per peripheral, not per-instance, so
// BusTransport::onTransferComplete() does the work of finding which
// transport was actually waiting — and, through FreeRtosTransport,
// waking the thread that issued the transfer.
//
// These must be linked into the application as object files. Inside a
// static library the linker has no reason to pull them in over the
// HAL's own weak defaults, and no transfer would ever complete.
extern "C" {

// After HAL_SPI_Transmit_IT/_DMA (writeReg/writeRegs/writeBytes, and
// SpiBlockTransport's header and write data phase).
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
    BusTransport::onTransferComplete(hspi, /*failed=*/false);
}

// After HAL_SPI_TransmitReceive_IT (readRegs/readBytes).
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    BusTransport::onTransferComplete(hspi, /*failed=*/false);
}

// After HAL_SPI_Receive_IT/_DMA (SpiBlockTransport's read data phase).
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
    BusTransport::onTransferComplete(hspi, /*failed=*/false);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    BusTransport::onTransferComplete(hspi, /*failed=*/true);
}

} // extern "C"
