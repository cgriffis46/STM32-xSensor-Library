#include "BusTransport.h"
#include "DebugLog.h"

namespace {
    // Maps a bus handle to whichever transport currently has a transfer
    // in flight on it — at most one per bus at a time. Slots are
    // claimed by handle when a transport is constructed and never
    // given back, so the interrupt-side lookup below only ever reads.
    struct BusSlot {
        bool          used   = false;
        void*         handle = nullptr;
        BusTransport* active = nullptr;
    };
    constexpr size_t kSlots = 8; // == BusTransport::kMaxBuses
    BusSlot s_bus[kSlots];
}

BusTransport::BusTransport(void* busHandle, void* busMutex)
    : busHandle_(busHandle), busMutex_(busMutex), slot_(kNoSlot) {
    static_assert(kSlots == kMaxBuses, "registry size and kMaxBuses must match");
    for (size_t i = 0; i < kSlots; ++i) {
        if (s_bus[i].used && s_bus[i].handle == busHandle) { slot_ = i; return; }
    }
    for (size_t i = 0; i < kSlots; ++i) {
        if (!s_bus[i].used) {
            s_bus[i].used   = true;
            s_bus[i].handle = busHandle;
            slot_ = i;
            return;
        }
    }
    // No slot left: slot_ stays kNoSlot and beginTransfer() refuses.
}

BusTransport::~BusTransport() {
    // Don't leave the interrupt a pointer to a destroyed object.
    if (slot_ != kNoSlot && s_bus[slot_].active == this) {
        s_bus[slot_].active = nullptr;
    }
}

// Finds the instance currently mid-transfer on this bus and signals it,
// or lets it start the next part of a transfer made of several.
void BusTransport::onTransferComplete(void* busHandle, bool failed) {
    DBG_PULSE(dbg::kPinBusIrq);
    for (size_t i = 0; i < kSlots; ++i) {
        if (s_bus[i].used && s_bus[i].handle == busHandle) {
            BusTransport* active = s_bus[i].active;
            if (active != nullptr && !active->ContinueTransfer(failed)) {
                if (failed) DBG_FAULT("bus", "xfer-fail", (int32_t)i);
                active->SignalTransferComplete(failed);
            } else if (active == nullptr) {
                DBG_FAULT("bus", "irq-idle", (int32_t)i);   // an interrupt with no transfer of ours in flight
            }
            return;
        }
    }
    DBG_FAULT("bus", "irq-unknown");   // a handle no transport was made with
}

void BusTransport::onBusEvent(void* busHandle, uint8_t event) {
    for (size_t i = 0; i < kSlots; ++i) {
        if (s_bus[i].used && s_bus[i].handle == busHandle) {
            BusTransport* active = s_bus[i].active;
            if (active != nullptr) active->HandleBusEvent(event);
            return;
        }
    }
}

void BusTransport::SignalTransferComplete(bool failed) {
    transferNotifiedFailed_ = failed; // outcome first, so whoever sees the flag sees the right outcome
    transferNotified_       = true;
}

bool BusTransport::takeCompletion() {
    if (!transferNotified_) return false;
    failed_           = transferNotifiedFailed_;
    transferNotified_ = false;
    return true;
}

bool BusTransport::WaitForTransfer(uint32_t timeoutTicks) {
    for (uint32_t i = 0; i < timeoutTicks; ++i) {
        if (takeCompletion()) return true;
        YieldTick();
    }
    return takeCompletion(); // one last check after the final yield
}

// Everything a transfer needs before the hardware call: this instance
// idle, the bus free and held, and this instance registered as the
// one onTransferComplete() should signal.
bool BusTransport::beginTransfer() {
    if (busy_) return false;           // this instance already has an outstanding transfer
    if (slot_ == kNoSlot) {             // more buses than kMaxBuses
        DBG_FAULT("bus", "no-slot");
        return false;
    }

    if (!ObtainMutex(kMutexTimeoutTicks)) {
        DBG_TRACE("bus", "mutex-busy", (int32_t)slot_);
        return false; // bus busy elsewhere; caller's sleep()-then-retry handles this
    }

    // Another device's transfer still in flight on this bus. With a
    // real mutex this can't happen (that device still holds it); with
    // the no-op default it is what keeps two devices on one bus apart.
    if (s_bus[slot_].active != nullptr) {
        DBG_TRACE("bus", "in-use", (int32_t)slot_);
        ReleaseMutex();
        return false;
    }

    failed_           = false;
    transferNotified_ = false;
    s_bus[slot_].active = this;
    return true;
}

// The hardware call either started (stay busy until isBusy() sees it
// land) or didn't (hand the bus straight back).
bool BusTransport::endIssue(bool issued) {
    if (!issued) {
        DBG_TRACE("bus", "not-issued", (int32_t)slot_);
        s_bus[slot_].active = nullptr;
        ReleaseMutex();
        return false;
    }
    busy_ = true;
    DBG_PIN(dbg::kPinTransfer, true);
    DBG_TRACE("bus", "start", (int32_t)slot_);
    return true;
}

bool BusTransport::isBusy() const {
    if (!busy_) return false;

    auto* self = const_cast<BusTransport*>(this);

    // Bounded wait for the signal from onTransferComplete() — default
    // is a poll yielding via YieldTick(); an overridden (RTOS) version
    // blocks properly instead. Either way this also sets failed_.
    if (!self->WaitForTransfer(kTransferTimeoutTicks)) {
        return true; // not yet — still busy as far as the caller's concerned
    }

    DBG_PIN(dbg::kPinTransfer, false);
    DBG_TRACE("bus", "done", (int32_t)slot_, failed_ ? 1 : 0);
    self->onTransferLanded();
    self->busy_ = false;
    s_bus[slot_].active = nullptr;
    self->ReleaseMutex();
    return false;
}

bool BusTransport::lastOpFailed() const { return failed_; }
