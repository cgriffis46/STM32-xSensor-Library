#include "Stm32HalUartTransport.h"
#include <cstddef>

namespace {
    // Same small fixed registry pattern as BusTransport's —
    // HAL_UART_TxCpltCallback/HAL_UART_RxCpltCallback/
    // HAL_UART_ErrorCallback are global weak callbacks keyed by
    // UART_HandleTypeDef*, not per-instance, so this is what maps a
    // given huart back to the Stm32HalUartTransport instance waiting
    // on it. Bump kMaxUarts if a project uses more than 4 UARTs this way.
    constexpr size_t kMaxUarts = 4;
    UART_HandleTypeDef*    s_uartHandle[kMaxUarts]   = {nullptr, nullptr, nullptr, nullptr};
    Stm32HalUartTransport* s_activeOnUart[kMaxUarts] = {nullptr, nullptr, nullptr, nullptr};
}

void Stm32HalUartTransport::registerActive(UART_HandleTypeDef* huart, Stm32HalUartTransport* instance) {
    for (size_t i = 0; i < kMaxUarts; ++i) {
        if (s_uartHandle[i] == nullptr || s_uartHandle[i] == huart) {
            s_uartHandle[i]   = huart;
            s_activeOnUart[i] = instance;
            return;
        }
    }
    // registry full — bump kMaxUarts if this project uses more UARTs
}

// Only if `instance` is still the one registered: a newer transport on
// the same UART must not be unhooked by an older one going away.
void Stm32HalUartTransport::unregisterActive(UART_HandleTypeDef* huart, Stm32HalUartTransport* instance) {
    for (size_t i = 0; i < kMaxUarts; ++i) {
        if (s_uartHandle[i] == huart) {
            if (s_activeOnUart[i] == instance) s_activeOnUart[i] = nullptr;
            return;
        }
    }
}

Stm32HalUartTransport* Stm32HalUartTransport::find(UART_HandleTypeDef* huart) {
    for (size_t i = 0; i < kMaxUarts; ++i) {
        if (s_uartHandle[i] == huart) return s_activeOnUart[i];
    }
    return nullptr;
}

// Nothing to do: write() asks HAL whether a send is in flight rather
// than keeping a flag of its own for this to clear. Kept so the
// callbacks file has one entry point per HAL callback.
void Stm32HalUartTransport::onTxComplete(UART_HandleTypeDef* huart) {
    (void)huart;
}

void Stm32HalUartTransport::onRxComplete(UART_HandleTypeDef* huart) {
    Stm32HalUartTransport* t = find(huart);
    if (t != nullptr) t->handleRxComplete();
}

// ISR context, from HAL_UART_ErrorCallback. After an overrun HAL has
// already abandoned the receive, and nothing more would ever arrive;
// start it again. After a noise or framing flag HAL carries on
// receiving, and startReceiving() sees that and leaves it alone.
void Stm32HalUartTransport::onError(UART_HandleTypeDef* huart) {
    Stm32HalUartTransport* t = find(huart);
    if (t != nullptr) t->startReceiving();
}

Stm32HalUartTransport::Stm32HalUartTransport(UART_HandleTypeDef* huart)
    : huart_(huart) {
    registerActive(huart_, this);
    startReceiving(); // start listening immediately — SHTP data can arrive unsolicited
}

Stm32HalUartTransport::~Stm32HalUartTransport() {
    // Only if this is still the instance the interrupts are routed to:
    // then the receive in flight is ours and points at our rxByte_.
    if (find(huart_) == this) {
        HAL_UART_AbortReceive(huart_);
        unregisterActive(huart_, this);
    }
}

void Stm32HalUartTransport::setRxSink(iTransportRxSink& sink) {
    rxSink_ = &sink;
    startReceiving();
}

bool Stm32HalUartTransport::startReceiving() {
    // Already listening, or the UART has not been initialised yet.
    // Either way there is nothing to arm. The overrun flag below must
    // not be touched while a receive is running: on some families
    // clearing it reads the data register, which would throw away a
    // byte the interrupt was about to collect.
    if (huart_->RxState != HAL_UART_STATE_READY) {
        return huart_->RxState == HAL_UART_STATE_BUSY_RX;
    }

    // An overrun left pending would end the new receive at once.
    __HAL_UART_CLEAR_OREFLAG(huart_);
    return HAL_UART_Receive_IT(huart_, &rxByte_, 1) == HAL_OK;
}

// HAL already refuses a second send while one is in flight (HAL_BUSY),
// so its answer is the whole of the busy check. An earlier version
// kept a flag of its own beside that, set after the HAL call returned
// and cleared by the transmit-complete interrupt — and when that
// interrupt arrived before the flag was set, the flag stayed set for
// good and every later write() was refused.
bool Stm32HalUartTransport::write(const uint8_t* data, size_t len) {
    if (data == nullptr || len == 0 || len > 0xFFFFu) return false;

    return HAL_UART_Transmit_IT(
        huart_, const_cast<uint8_t*>(data), static_cast<uint16_t>(len)) == HAL_OK;
}

// ISR context. Hands the just-received byte directly to rxSink_ if
// one's been attached — this class has no idea what the sink does
// with it (a FreeRTOS stream buffer, in xBNO085's case). If no sink
// has been attached yet, the byte is silently dropped — see
// setRxSink()'s own comment for why that's the correct default rather
// than an error. Either way, immediately re-arms the next single-byte
// receive so nothing is missed while this runs.
void Stm32HalUartTransport::handleRxComplete() {
    if (rxSink_) rxSink_->onByteReceived(rxByte_);
    startReceiving();
}
