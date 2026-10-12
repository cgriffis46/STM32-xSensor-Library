#pragma once
#include "iTransportOneWire.h"
#include "BusTransport.h"

// 1-Wire driven from a UART, with the UART's TX (open drain) and RX
// both on the 1-Wire line — the standard technique (Maxim/Analog
// Devices application note 214), and the one DS18B20.cpp already used.
// Every 1-Wire time slot is one UART character, so the UART's own
// baud-rate timing produces the pulse widths and no bit-banging or
// microsecond delay is needed:
//
//   reset      one character 0xF0 at 9600 baud. Its start bit and four
//              low data bits hold the line low for 520 us (the reset
//              pulse). If the character reads back changed, a device
//              pulled the line low during the high half: presence.
//   write 1    0xFF at 115200 baud: only the 8.7 us start bit is low.
//   write 0    0x00 at 115200 baud: low for 78 us.
//   read       0xFF at 115200 baud, as for writing a 1; a device
//              sending 0 holds the line low a little longer, which
//              the UART samples as bit 0 of the character being 0.
//
// Everything sent is also received, since TX and RX share the line.
// So one operation is one transmit of N characters and one receive of
// the same N, started together with the interrupt-driven HAL calls,
// and it is over when BOTH have finished. Only then is the waiting
// thread signalled — the same way an I2C or SPI transfer ends, through
// BusTransport, so FreeRtosTransport<OneWireUartTransport> gives this
// a real mutex and a real wake-up exactly as it does for those.
//
// Hardware-agnostic: no HAL header here. The literal UART calls live
// in Stm32HalOneWireTransport.
class OneWireUartTransport : public iTransportOneWire, public BusTransport {
public:
    // uartHandle: the UART's real HAL handle (e.g. &huart1), opaque
    // here. busMutex: one per 1-Wire bus — see BusTransport.
    OneWireUartTransport(void* uartHandle, void* busMutex);
    ~OneWireUartTransport() override = default;

    bool reset() override;
    bool writeBytes(const uint8_t* buf, uint8_t len) override;
    bool readBytes(uint8_t* buf, uint8_t len) override;
    bool isBusy() const override { return BusTransport::isBusy(); }
    bool lastOpFailed() const override { return BusTransport::lastOpFailed(); }
    bool presence() const override { return presence_; }

    // Call these from the UART's transmit-complete, receive-complete
    // and error interrupts (STM32: HAL_UART_TxCpltCallback,
    // HAL_UART_RxCpltCallback, HAL_UART_ErrorCallback — see
    // Stm32UartItCallbacks.cpp). uartHandle is the value the transport
    // was constructed with. A UART that no 1-Wire transfer is in
    // flight on is ignored, so it is safe to call these for every UART.
    static void onUartTxComplete(void* uartHandle);
    static void onUartRxComplete(void* uartHandle);
    static void onUartError(void* uartHandle);

    static constexpr uint32_t kResetBaud = 9600;
    static constexpr uint32_t kDataBaud  = 115200;

protected:
    // The actual hardware calls, supplied by a concrete transport.
    //
    // halSetBaud(): change the UART's baud rate. Only ever called
    // with the UART idle, from the thread issuing an operation.
    // halReceive()/halTransmit(): start receiving / sending n
    // characters and return at once (STM32: HAL_UART_Receive_IT,
    // HAL_UART_Transmit_IT); each ends by calling the matching on*()
    // function above. halReceive() is always called first.
    // halAbort(): stop anything still running after a fault.
    virtual bool halSetBaud(uint32_t baud) = 0;
    virtual bool halReceive(uint8_t* rx, uint16_t n) = 0;
    virtual bool halTransmit(uint8_t* tx, uint16_t n) = 0;
    virtual void halAbort() = 0;

    void HandleBusEvent(uint8_t event) override; // interrupt context
    void onTransferLanded() override;            // waiting thread, bus still held

private:
    enum Op : uint8_t { OpReset, OpWrite, OpRead };
    enum Event : uint8_t { EvTxDone, EvRxDone, EvError };

    static constexpr uint8_t  kResetChar = 0xF0;
    static constexpr uint8_t  kSlotOne   = 0xFF; // write 1, and every read slot
    static constexpr uint8_t  kSlotZero  = 0x00; // write 0
    static constexpr uint16_t kMaxSlots  = 8u * kMaxLen;

    // Shared tail of the three operations, entered with the bus held
    // and txSlots_[0..n) filled in.
    bool start(Op op, uint32_t baud, uint16_t n);

    uint8_t  txSlots_[kMaxSlots] = {0};
    uint8_t  rxSlots_[kMaxSlots] = {0};
    uint16_t slots_ = 0;
    Op       op_ = OpReset;
    uint8_t* destBuf_ = nullptr;  // caller's buffer for readBytes()
    uint8_t  destLen_ = 0;
    uint32_t baud_ = 0;           // what the UART is set to now; 0 = not known yet
    bool     presence_ = false;

    // Written by the UART interrupts.
    volatile bool txDone_ = false;
    volatile bool rxDone_ = false;
    volatile bool signalled_ = false;
};
