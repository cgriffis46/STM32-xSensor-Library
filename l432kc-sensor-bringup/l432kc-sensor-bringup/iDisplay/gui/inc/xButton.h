#pragma once
#include <stdint.h>
#include "xGuiEvent.h"

namespace idisplay {

// How a button behaves. Three kinds:
//
//   plain        (longPressMs == 0)  Pressed as soon as it goes down.
//   long press   (longPressMs > 0, repeatMs == 0)  One event per push:
//                Pressed when it is let go before longPressMs, or Held
//                once it has been down for longPressMs (and nothing when
//                it is then let go). Pressed comes on release so that a
//                long press is never also a short one. Use it for Enter
//                with three buttons: short chooses, long goes back.
//   auto-repeat  (longPressMs > 0, repeatMs > 0)  Pressed as soon as it
//                goes down, then Repeat after longPressMs and every
//                repeatMs after that while it stays down. Use it for Up
//                and Down to run through a menu or a character set.
//
// reportReleases adds Released when it is let go, after a plain or
// auto-repeat press, or after Held.
//
// Held and Repeat need a clock: something must call xButton::onTick()
// every few ms (xGuiButtonGroup's timer, in hw/freertos).
struct xButtonConfig {
    uint16_t debounceMs = 50;
    uint16_t longPressMs = 0;
    uint16_t repeatMs = 0;
    bool reportReleases = false;

    static xButtonConfig plain(uint16_t debounceMs = 50) {
        xButtonConfig c; c.debounceMs = debounceMs; return c;
    }
    static xButtonConfig longPress(uint16_t longPressMs = 600, uint16_t debounceMs = 50) {
        xButtonConfig c; c.debounceMs = debounceMs; c.longPressMs = longPressMs; return c;
    }
    static xButtonConfig autoRepeat(uint16_t delayMs = 500, uint16_t repeatMs = 150, uint16_t debounceMs = 50) {
        xButtonConfig c; c.debounceMs = debounceMs; c.longPressMs = delayMs; c.repeatMs = repeatMs; return c;
    }
};

// One push button: turns the edges its pin interrupt sees, and the time
// it is held, into key events. No RTOS and no HAL here: the caller reads
// the pin and posts the event if there is one (xGuiButton in
// hw/freertos does that).
//
// Debouncing, for an interrupt on both edges:
//  - a press is taken only if the button has been up for debounceMs,
//    so the bounces of a release, and of the press itself, are not
//    seen as presses;
//  - a release counts only once the press it ends has lasted
//    debounceMs. A release inside that is taken as bounce (so a tap
//    shorter than debounceMs is a press with no release; on a long
//    press button, where the press is reported at release, it is lost).
// The GUI's one-slot event queue does the rest: while a key is waiting
// to be handled, further events are dropped rather than queued up.
//
// onLevel() runs in the pin interrupt and onTick() in a timer task. The
// interrupt can run in the middle of onTick() but not the other way
// round, and the two share state in a way that stays right either way:
// see onTick() in xButton.cpp.
class xButton {
public:
    explicit xButton(xKey key, uint16_t debounceMs = 50, bool reportReleases = false)
        : key_(key) {
        cfg_.debounceMs = debounceMs;
        cfg_.reportReleases = reportReleases;
    }
    xButton(xKey key, const xButtonConfig& config) : key_(key), cfg_(config) {}

    xKey key() const { return key_; }
    const xButtonConfig& config() const { return cfg_; }
    bool isDown() const { return down_; }

    // Call on every edge (or every poll) with the button's state now.
    // True when ev holds an event to post. Safe in an interrupt.
    bool onLevel(bool down, uint32_t nowMs, xGuiEvent& ev);

    // Call every few ms from one thread (a timer). True when ev holds a
    // Held or Repeat event to post. Does nothing on a plain button.
    bool onTick(uint32_t nowMs, xGuiEvent& ev);

private:
    bool deferred() const { return cfg_.longPressMs > 0 && cfg_.repeatMs == 0; }

    xKey key_;
    xButtonConfig cfg_;

    // Written by onLevel() (the interrupt).
    volatile bool down_ = false;
    volatile bool pressActive_ = false;      // a press was taken and not yet released
    volatile bool everReleased_ = false;     // releasedAt_ is meaningful
    volatile uint32_t releasedAt_ = 0;
    volatile uint32_t pressedAt_ = 0;
    volatile uint32_t pressSeq_ = 0;         // counts presses taken; 0 = none yet
    volatile uint32_t clickSeq_ = 0;         // the press reported as a short press at its release

    // Written by onTick() (the timer).
    volatile uint32_t heldSeq_ = 0;          // the press that reached longPressMs
    uint32_t nextRepeatMs_ = 0;              // when the next Repeat is due, counted from the press
};

} // namespace idisplay
