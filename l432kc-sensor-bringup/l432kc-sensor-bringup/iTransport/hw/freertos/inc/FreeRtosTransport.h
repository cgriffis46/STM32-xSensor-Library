#pragma once
#include <type_traits>
#include "cmsis_os2.h"
#include "BusTransport.h"

// The CMSIS-RTOS2 versions of BusTransport's four platform hooks,
// written once for every bus:
//
//   FreeRtosTransport<I2CTransport>    FreeRtosTransport<SPITransport>
//   FreeRtosTransport<OneWireUartTransport>
//   FreeRtosTransport<SpiBlockTransport>
//
//   ObtainMutex()/ReleaseMutex()   -> osMutexAcquire()/osMutexRelease()
//   SignalTransferComplete()       -> osThreadFlagsSet(), from the bus interrupt
//   WaitForTransfer()              -> osThreadFlagsWait()
//
// So the thread that issued a transfer sleeps inside isBusy() and is
// woken by the interrupt that finishes it, instead of polling. (Under
// FreeRTOS, CMSIS thread flags ARE task notifications: osThreadFlagsSet()
// is xTaskNotifyFromISR() underneath.)
//
// Still abstract — the hal*() hardware calls are left for a
// further-derived class (Stm32HalI2CTransport, Stm32HalSPITransport,
// Stm32HalOneWireTransport, Stm32HalSpiBlockTransport).
// A target with no RTOS derives from I2CTransport/SPITransport
// directly instead and gets BusTransport's polling defaults — see
// ArduinoWireTransport.
//
// busMutex (the last constructor argument of each bus class) must
// be a real osMutexId_t here: one per physical bus, created with
// osMutexNew() BEFORE the transport is constructed. A null handle is
// refused by osMutexAcquire(), so every transfer would be refused.
//
// The bus interrupt must be allowed to call FreeRTOS: its priority
// has to be numerically at or above
// configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (CubeMX arranges this
// when the interrupt is marked as using FreeRTOS functions).
template <typename TBus>
class FreeRtosTransport : public TBus {
    static_assert(std::is_base_of<BusTransport, TBus>::value,
                  "FreeRtosTransport<TBus> requires TBus to derive from BusTransport "
                  "(I2CTransport, SPITransport, SpiBlockTransport or OneWireUartTransport).");

public:
    using TBus::TBus;

protected:
    // Also notes which thread is about to issue a transfer, so the
    // interrupt knows who to wake.
    bool ObtainMutex(uint32_t timeoutTicks) override {
        if (osMutexAcquire(static_cast<osMutexId_t>(this->busMutex_), timeoutTicks) != osOK) {
            return false;
        }
        waitingTask_ = osThreadGetId();
        return true;
    }

    void ReleaseMutex() override {
        osMutexRelease(static_cast<osMutexId_t>(this->busMutex_));
    }

    // Interrupt context. Record the outcome on THIS transport first,
    // then wake the thread.
    void SignalTransferComplete(bool failed) override {
        TBus::SignalTransferComplete(failed);
        if (waitingTask_ != nullptr) {
            osThreadFlagsSet(waitingTask_, kFlagWake);
        }
    }

    // The flag only wakes the thread; whether THIS transfer finished
    // is read from the transport itself (takeCompletion()). A thread's
    // flags are shared by everything that thread uses, so a wake-up
    // meant for another transport, or one left over from earlier,
    // must not be mistaken for this transfer landing — it just means
    // "look again", and isBusy() reports still busy.
    bool WaitForTransfer(uint32_t timeoutTicks) override {
        if (this->takeCompletion()) return true; // landed before we got here
        osThreadFlagsWait(kFlagWake, osFlagsWaitAny, timeoutTicks);
        return this->takeCompletion();
    }

private:
    // A high bit, out of the way of flags an application is likely
    // to have chosen for its own use on the same thread.
    static constexpr uint32_t kFlagWake = 0x40000000u;

    osThreadId_t waitingTask_ = nullptr;
};
