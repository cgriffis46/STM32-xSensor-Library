#include "Stm32HalOneWireTransport.h"

bool Stm32HalOneWireTransport::halSetBaud(uint32_t baud) {
    UART_HandleTypeDef* h = huart();

    if (refBrr_ == 0) {
        if (h->Init.OverSampling != UART_OVERSAMPLING_16 || h->Init.BaudRate == 0) {
            return false; // BRR isn't a plain clock/baud ratio with oversampling by 8
        }
        refBrr_  = h->Instance->BRR;
        refBaud_ = h->Init.BaudRate;
        if (refBrr_ == 0) return false; // UART not initialised yet
    }

    const uint32_t brr = static_cast<uint32_t>(
        (static_cast<uint64_t>(refBrr_) * refBaud_ + baud / 2u) / baud);

    // Only called with the UART idle. Newer families only accept a
    // write to BRR while the UART is disabled.
    __HAL_UART_DISABLE(h);
    h->Instance->BRR = brr;
    __HAL_UART_ENABLE(h);
    return true;
}

// Returns as soon as reception is armed. It finishes in the UART
// interrupt, through HAL_UART_RxCpltCallback (Stm32UartItCallbacks.cpp).
bool Stm32HalOneWireTransport::halReceive(uint8_t* rx, uint16_t n) {
    // An overrun left over from earlier would end the new reception
    // before it began.
    __HAL_UART_CLEAR_OREFLAG(huart());
    return HAL_UART_Receive_IT(huart(), rx, n) == HAL_OK;
}

// Likewise, through HAL_UART_TxCpltCallback.
bool Stm32HalOneWireTransport::halTransmit(uint8_t* tx, uint16_t n) {
    return HAL_UART_Transmit_IT(huart(), tx, n) == HAL_OK;
}

void Stm32HalOneWireTransport::halAbort() {
    HAL_UART_Abort(huart());
}
