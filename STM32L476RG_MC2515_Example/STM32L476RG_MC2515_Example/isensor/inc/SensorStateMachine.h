#pragma once
#include <cstdint>
#include <utility>
#include "DebugLog.h"

// The part of a non-blocking sensor driver that is the same for every
// sensor: which state it is in and when it got there, the sleep()
// hook, and the two halves of a bus operation.
//
//   template <typename TTransport>
//   class bmp280 : public SensorStateMachine<TTransport, bmp280_state_t> { ... };
//
// TTransport is the bus the driver talks through, and this class
// inherits from it, so the driver does too. TState is the driver's
// own state enum; the states themselves, and the switch in main()
// that walks them, stay in the driver.
//
// A driver built on this never waits on the bus. Each bus operation
// is two states:
//
//   case xxx_read_id:                       // issue it
//       issued(this->readRegs(reg, &id, 1), xxx_wait_id, nowMs);
//       break;
//   case xxx_wait_id:                       // wait for it to land
//       if (landed(nowMs)) { ...use id...; enter(xxx_next, nowMs); }
//       break;
//
// issued() and landed() carry the timeout and the failure path, so a
// state cannot wait for ever: anything that goes wrong ends in the
// driver's error state, and errorCleared() there says when it is safe
// to start again.
//
// Works with any transport that has isBusy() and lastOpFailed() —
// ISensorTransport (I2C, SPI) and iTransportOneWire alike. Which of
// those a particular driver needs is for the driver to assert.
//
// A driver on a stream transport (iTransport: a sensor that just
// keeps sending, like the PM2.5) has no bus operations to issue or
// wait for, and uses only the state, timing and sleep() parts.
// issued(), landed(), finished() and errorCleared() are compiled only
// where a driver calls them, so the transport need not have isBusy().
//
// Debugging on hardware (itransport's DebugLog.h, ITRANSPORT_DEBUG):
// every state change is logged as "<tag> st <from> <to>" with a pulse on
// dbg::kPinState, every failure as "<tag> fail <state>" (with the cause,
// "bus-err" or "timeout", just before it) and a pulse on dbg::kPinFault.
// The numbers are the driver's state enum. setDebugTag() names the
// driver in the log ("ssm" until it is set); a no-op when the log is off.
//
// Not to be confused with SensorBase, the older base class in this
// folder. SensorBase holds a reference to an ISensorTransport and
// runs one fixed sequence of states that a sensor customises through
// virtual functions. Here the transport is inherited rather than
// referenced, it need not be an ISensorTransport, and every driver
// keeps its own states.
template <typename TTransport, typename TState>
class SensorStateMachine : protected TTransport {
public:
    TState state() const { return _state; }

    // The name this driver has in the debug log. A string literal, or
    // anything that outlives the driver.
    void setDebugTag(const char* tag) {
#if ITRANSPORT_DEBUG
        _dbgTag = tag;
#else
        (void)tag;
#endif
    }

protected:
    // initState: where main() starts. errorState: where fail() goes.
    // busTimeoutMs: how long one bus operation may take to issue, or
    // to land. Everything after that goes to TTransport's constructor.
    template <typename... TArgs>
    SensorStateMachine(TState initState, TState errorState, uint32_t busTimeoutMs, TArgs&&... transportArgs)
        : TTransport(std::forward<TArgs>(transportArgs)...),
          _state(initState), last_update(0), _errorState(errorState), _busTimeoutMs(busTimeoutMs) {}

    virtual ~SensorStateMachine() {}

    // Called whenever a state has nothing to do for ms. An empty stub
    // for bare metal, where main() just returns and is called again.
    // Override it for an OS with a call that gives up the CPU.
    virtual void sleep(uint32_t ms) { (void)ms; }

    void enter(TState next, uint32_t nowMs) {
        if (next != _state) {
            DBG_EVENT(_dbgTag, "st", (int32_t)_state, (int32_t)next);
            DBG_PULSE(dbg::kPinState);
        }
        _state = next;
        last_update = nowMs;
    }

    // Goes to the error state. Every failure, from whichever helper,
    // comes through here, and onFail() is called first.
    void fail(uint32_t nowMs) {
        DBG_FAULT(_dbgTag, "fail", (int32_t)_state);
        DBG_PULSE(dbg::kPinFault);
        onFail();
        enter(_errorState, nowMs);
    }

    // Called on every failure, before the error state is entered. A
    // driver overrides it to mark its readings as no longer valid.
    virtual void onFail() {}

    // True once ms have passed since the current state was entered.
    // Unsigned subtraction gives the right answer across the 49.7 day
    // rollover of a 32 bit ms counter; now > last_update + ms does not.
    bool elapsed(uint32_t nowMs, uint32_t ms) const { return (nowMs - last_update) >= ms; }

    // Sleep for what is left of ms counted from `since`, so an OS
    // thread wakes when there is something to do.
    void sleepRemaining(uint32_t since, uint32_t nowMs, uint32_t ms) {
        const uint32_t waited = nowMs - since;
        if (waited < ms) sleep(ms - waited);
    }

    // The same, counted from when the current state was entered.
    void sleepRemaining(uint32_t nowMs, uint32_t ms) { sleepRemaining(last_update, nowMs, ms); }

    // Body of every state that issues a bus operation. `started` is
    // what the transport returned. false may only mean the bus is in
    // use, so the state is left as it is to try again — until the bus
    // timeout, when it fails.
    void issued(bool started, TState waitState, uint32_t nowMs) {
        if (started) {
            enter(waitState, nowMs);
        } else if (elapsed(nowMs, _busTimeoutMs)) {
            DBG_FAULT(_dbgTag, "issue-timeout", (int32_t)_state);
            fail(nowMs);
        } else {
            sleep(1);
        }
    }

    // Body of every state that waits for one. True once the operation
    // has landed and the transport reports no error. Fails on an
    // error, or if it has not landed within the bus timeout.
    bool landed(uint32_t nowMs) {
        if (!finished(nowMs)) return false;
        if (this->lastOpFailed()) {
            DBG_FAULT(_dbgTag, "bus-err", (int32_t)_state);
            fail(nowMs);
            return false;
        }
        return true;
    }

    // As landed(), but true whatever the outcome, for the few
    // operations where an error is expected and is not a fault — a
    // chip that resets before it acknowledges the reset command, say.
    // The caller looks at lastOpFailed() itself. Still fails if the
    // operation has not finished within the bus timeout.
    bool finished(uint32_t nowMs) {
        if (this->isBusy()) {
            if (elapsed(nowMs, _busTimeoutMs)) {
                DBG_FAULT(_dbgTag, "land-timeout", (int32_t)_state);
                fail(nowMs);
            } else {
                sleep(1);
            }
            return false;
        }
        return true;
    }

    // Body of the error state. True once backoffMs have passed AND
    // nothing is in flight any more. An operation the driver gave up
    // on may still be running; the transport has to be asked until it
    // lands, or it stays busy and keeps hold of the bus.
    bool errorCleared(uint32_t nowMs, uint32_t backoffMs) {
        if (this->isBusy()) {
            sleep(1);
            return false;
        }
        if (!elapsed(nowMs, backoffMs)) {
            sleepRemaining(nowMs, backoffMs);
            return false;
        }
        return true;
    }

    TState   _state;
    uint32_t last_update; // when the current state was entered

private:
    TState   _errorState;
    uint32_t _busTimeoutMs;
#if ITRANSPORT_DEBUG
    const char* _dbgTag = "ssm";
#endif
};
