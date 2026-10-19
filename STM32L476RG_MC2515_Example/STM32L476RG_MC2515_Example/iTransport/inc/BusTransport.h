#pragma once
#include <cstddef>
#include <cstdint>

// Everything an interrupt-driven bus transport needs that is the same
// whatever the bus: arbitration, the transfer-completion signal, and
// the busy/failed bookkeeping. I2CTransport, SPITransport and
// OneWireUartTransport all derive from this, alongside the interface
// they present to a driver (ISensorTransport or iTransportOneWire),
// and add only what is particular to their bus.
//
// Deliberately NOT derived from any driver-facing interface itself:
// the machinery here is the same for a register bus and for 1-Wire,
// whose interfaces differ. Each bus class forwards its interface's
// isBusy()/lastOpFailed() to the two functions of the same name here.
//
// The life of one transfer:
//
//   1. beginTransfer()   takes the bus (the mutex, where there is one)
//                        and registers this instance as the one the
//                        bus interrupt should signal.
//   2. the hardware call starts the transfer and returns at once.
//      endIssue() gives the bus straight back if it didn't start.
//   3. onTransferComplete(), called from the bus interrupt, finds the
//      registered instance and calls its SignalTransferComplete() —
//      unless ContinueTransfer() started a further part of it.
//   4. isBusy(), called by whoever is waiting, sees the signal via
//      WaitForTransfer(), calls onTransferLanded(), and releases the bus.
//
// No RTOS header is included here. ObtainMutex()/ReleaseMutex() and
// SignalTransferComplete()/WaitForTransfer() are where a platform's
// own primitives go — see FreeRtosTransport<TBus>, which supplies the
// CMSIS-RTOS2 versions for I2C and SPI alike. The defaults below are a
// working, if less efficient, fallback for a target with no RTOS: a
// flag set by the interrupt and polled by isBusy().
//
// Bus handles are opaque (void*) for the reason given in I2CTransport.h:
// STM32Cube HAL handle types can't safely be forward-declared.
class BusTransport {
public:
    virtual ~BusTransport();

    // The interrupt holds a pointer to whichever instance is mid-transfer.
    BusTransport(const BusTransport&) = delete;
    BusTransport& operator=(const BusTransport&) = delete;

    // True while this instance's transfer is still in flight. Waits
    // (bounded) for it to finish — see WaitForTransfer().
    bool isBusy() const;

    // Valid once isBusy() == false.
    bool lastOpFailed() const;

    // Call this from wherever this platform learns a transfer
    // finished: the HAL completion and error callbacks on STM32 (see
    // Stm32I2CItCallbacks.cpp / Stm32SpiItCallbacks.cpp), or directly
    // after a blocking call returns (see ArduinoWireTransport).
    // busHandle is the same value the transport was constructed with,
    // e.g. &hi2c1 or &hspi1. Safe to call from interrupt context.
    static void onTransferComplete(void* busHandle, bool failed);

    // For a bus where one transfer ends in more than one interrupt
    // (1-Wire over a UART: transmit-complete AND receive-complete).
    // Hands `event` to HandleBusEvent() of whichever instance is
    // mid-transfer on busHandle; that instance decides when the
    // transfer as a whole is over. Safe to call from interrupt context.
    static void onBusEvent(void* busHandle, uint8_t event);

protected:
    // busHandle: the bus's real HAL handle, passed through opaquely.
    // busMutex: likewise opaque. It must be the SAME value for every
    // transport sharing this physical bus — create it once per bus
    // and pass it to each device's transport.
    BusTransport(void* busHandle, void* busMutex);

    // Acquires the bus mutex, waiting up to timeoutTicks. Default
    // reports success immediately: on a target with no RTOS there is
    // one thread, and beginTransfer() already refuses to start while
    // another device's transfer is in flight on the same bus.
    virtual bool ObtainMutex(uint32_t timeoutTicks) { (void)timeoutTicks; return true; }

    // Releases whatever ObtainMutex() acquired. If you override one
    // of this pair, override both.
    virtual void ReleaseMutex() {}

    // Called from the bus interrupt. The default records the outcome
    // on this instance for takeCompletion() to find. An override that
    // wakes a waiting thread must still call this first: the record
    // on the instance is what says WHICH transfer finished, and a
    // wake-up alone can't.
    virtual void SignalTransferComplete(bool failed);

    // Waits up to timeoutTicks for this instance's transfer to finish.
    // Returns true, with failed_ set, once it has. Default polls
    // takeCompletion(), calling YieldTick() between polls.
    virtual bool WaitForTransfer(uint32_t timeoutTicks);

    // Called between polls in the default WaitForTransfer(). No-op by
    // default (a tight spin) — override to yield or delay instead,
    // e.g. Arduino's delay(1).
    virtual void YieldTick() {}

    // Interrupt context, via onBusEvent(). An override records the
    // event and calls SignalTransferComplete() once the transfer is
    // complete. Unused by I2C and SPI, which finish in one interrupt.
    virtual void HandleBusEvent(uint8_t event) { (void)event; }

    // Interrupt context, via onTransferComplete(), before anything is
    // signalled. For a transfer made of more than one hardware call
    // under one chip-select (SpiBlockTransport: a header, then a block
    // of data): start the next part and return true, and the transfer
    // is not over yet. Return false to let it finish now, setting
    // `failed` if starting the next part went wrong. Default: every
    // transfer is a single call.
    virtual bool ContinueTransfer(bool& failed) { (void)failed; return false; }

    // Called from isBusy(), in the waiting thread, once the transfer
    // has finished and before the bus is released. SPI uses it to
    // raise chip-select and copy the received bytes out; 1-Wire to
    // turn the received bit slots back into bytes.
    virtual void onTransferLanded() {}

    // True, once, after SignalTransferComplete() has run for this
    // instance's transfer; sets failed_ from what the interrupt reported.
    bool takeCompletion();

    bool beginTransfer();
    bool endIssue(bool issued);

    void* busHandle_;
    void* busMutex_;         // opaque — see ObtainMutex()/ReleaseMutex() above
    bool  failed_ = false;   // outcome of the last transfer, or of checkDevice()

    static constexpr uint32_t kMutexTimeoutTicks    = 100; // bus-contention bound
    static constexpr uint32_t kTransferTimeoutTicks = 50;  // how long one isBusy() call waits;
                                                           // meaning depends on the WaitForTransfer()
                                                           // in use (RTOS ticks, or poll iterations)

private:
    // One slot per physical bus, I2C and SPI together. Bump this if a
    // project has more. A transport on a bus past the limit refuses
    // every transfer rather than start one whose completion could
    // never be delivered.
    static constexpr size_t kMaxBuses = 8;
    static constexpr size_t kNoSlot   = kMaxBuses;

    size_t slot_;            // this bus's slot, claimed once in the constructor
    bool   busy_ = false;

    // Written by the interrupt, read by the waiting thread.
    volatile bool transferNotified_       = false;
    volatile bool transferNotifiedFailed_ = false;
};
