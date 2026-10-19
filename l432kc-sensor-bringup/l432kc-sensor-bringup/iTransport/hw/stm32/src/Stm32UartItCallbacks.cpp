#include "Stm32HalUartTransport.h"
#include "OneWireUartTransport.h"

// STM32 HAL calls these (weak, overridable) from the UART interrupt
// handler once a Transmit_IT/Receive_IT transfer finishes or faults.
// Same reasoning as Stm32I2CItCallbacks.cpp: HAL callbacks are free C
// functions per peripheral, not per-instance, so each transport's own
// on*() function does the work of finding which instance was actually
// waiting.
//
// Two kinds of transport sit on UARTs, and one set of callbacks has
// to serve both: Stm32HalUartTransport (a byte stream) and the 1-Wire
// transport. Each is offered every event and ignores a UART that
// isn't its own.
//
// These must be linked into the application as object files. Inside a
// static library the linker has no reason to pull them in over the
// HAL's own weak defaults, and no transfer would ever complete.
extern "C" {

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    Stm32HalUartTransport::onTxComplete(huart);
    OneWireUartTransport::onUartTxComplete(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    Stm32HalUartTransport::onRxComplete(huart);
    OneWireUartTransport::onUartRxComplete(huart);
}

// The stream transport is told about every error: all it does is
// start its receiver again if HAL has stopped it.
//
// For 1-Wire only an overrun is passed on: it is the one error after
// which HAL abandons the reception, so the transfer can never finish.
// Noise and framing flags are expected now and then on a 1-Wire line
// — a device releases the line whenever it likes relative to the
// UART's sampling points — and HAL carries on receiving after them.
// Real corruption is caught by the transport's read-back check on
// writes and by the device's own CRC on reads.
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    Stm32HalUartTransport::onError(huart);
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U) {
        OneWireUartTransport::onUartError(huart);
    }
}

} // extern "C"
