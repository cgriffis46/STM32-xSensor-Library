#pragma once
// The project's own main.h, not a specific family's _hal.h — see
// Stm32HalI2CTransport.h.
#include "main.h"
#include "OneWireUartTransport.h"
#include "FreeRtosTransport.h"

// The literal STM32 HAL_UART_* calls behind a 1-Wire bus — the
// interrupt-driven (_IT) variants, completing through
// Stm32UartItCallbacks.cpp. This is the class you actually construct;
// OneWireUartTransport itself is abstract. Derives from
// FreeRtosTransport<OneWireUartTransport> to get a real bus mutex and
// to be woken by the UART interrupts rather than polling, exactly as
// Stm32HalI2CTransport and Stm32HalSPITransport do.
//
// Wiring and CubeMX set-up for the UART:
//   - TX and RX both on the 1-Wire line, TX open drain, with the
//     usual pull-up on the line. Either an external link between the
//     two pins or the peripheral's single-wire (half-duplex) mode
//     will do: all that matters is that the UART hears what it sends.
//   - Asynchronous, 8 data bits, no parity, 1 stop bit, oversampling
//     by 16. Any baud rate; this class sets 9600 and 115200 itself.
//   - The UART's global interrupt enabled.
// busMutex: created with osMutexNew() before constructing this.
class Stm32HalOneWireTransport : public FreeRtosTransport<OneWireUartTransport> {
public:
    Stm32HalOneWireTransport(UART_HandleTypeDef* huart, osMutexId_t busMutex)
        : FreeRtosTransport<OneWireUartTransport>(huart, busMutex) {}

protected:
    // Changes the baud rate by rescaling the divider CubeMX already
    // programmed: with oversampling by 16 the BRR register is simply
    // (UART clock / baud), so the new value follows from the old one
    // without needing to know which clock feeds this UART. Leaves
    // every other setting, single-wire mode included, as it was.
    // Returns false if the UART is not oversampling by 16.
    bool halSetBaud(uint32_t baud) override;

    bool halReceive(uint8_t* rx, uint16_t n) override;
    bool halTransmit(uint8_t* tx, uint16_t n) override;
    void halAbort() override;

private:
    UART_HandleTypeDef* huart() const { return static_cast<UART_HandleTypeDef*>(busHandle_); }

    uint32_t refBrr_  = 0; // divider and the baud rate it stands for, read once from the
    uint32_t refBaud_ = 0; // UART as CubeMX configured it
};
