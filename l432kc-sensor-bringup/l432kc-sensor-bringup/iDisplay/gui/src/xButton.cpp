#include "xButton.h"

namespace idisplay {

bool xButton::onLevel(bool down, uint32_t nowMs, xGuiEvent& ev) {
    if (down == down_) return false;     // no edge: a bounce the pin settled from
    down_ = down;

    if (down) {
        // Unsigned subtraction keeps this right across the tick rollover.
        const bool settled = !everReleased_ || (uint32_t)(nowMs - releasedAt_) >= cfg_.debounceMs;
        if (!settled) return false;
        // pressedAt_ before pressSeq_: onTick() reads the sequence on
        // both sides of the time, so it never pairs a new count with an
        // old time.
        pressedAt_ = nowMs;
        pressSeq_ = pressSeq_ + 1;
        if (pressSeq_ == 0) pressSeq_ = 1;   // 0 means "none"
        pressActive_ = true;
        if (deferred()) return false;        // decided at the release, or by onTick()
        ev = xGuiEvent::keyEvent(key_, xKeyAction::Pressed);
        return true;
    }

    releasedAt_ = nowMs;
    everReleased_ = true;
    // A release inside debounceMs of the press is its bounce. Should it
    // have been a real tap that short, the next press is still taken
    // (the button has been up long enough); only this release is lost.
    if (!pressActive_ || (uint32_t)(nowMs - pressedAt_) < cfg_.debounceMs) return false;
    pressActive_ = false;

    const uint32_t seq = pressSeq_;
    if (deferred() && heldSeq_ != seq) {
        // Let go before longPressMs: a short press. clickSeq_ tells an
        // onTick() this interrupt cut into not to report Held as well.
        clickSeq_ = seq;
        ev = xGuiEvent::keyEvent(key_, xKeyAction::Pressed);
        return true;
    }
    if (!cfg_.reportReleases) return false;
    ev = xGuiEvent::keyEvent(key_, xKeyAction::Released);
    return true;
}

// The interrupt may run anywhere in here. What keeps a long press button
// to exactly one event per push is the order of two writes:
//
//   onTick():   heldSeq_ = seq, then report Held only if clickSeq_ != seq
//   release:    report Pressed only if heldSeq_ != seq, then clickSeq_ = seq
//               (in that order, as seen from here: the interrupt runs
//               whole)
//
// If the release runs before heldSeq_ is written, it reports Pressed and
// sets clickSeq_, and onTick() then sees clickSeq_ and stays quiet. If it
// runs after, it sees heldSeq_ and stays quiet, and onTick() reports Held.
bool xButton::onTick(uint32_t nowMs, xGuiEvent& ev) {
    if (cfg_.longPressMs == 0) return false;

    const uint32_t seq = pressSeq_;
    const uint32_t at = pressedAt_;
    const bool held = pressActive_ && down_;
    if (seq == 0 || seq != pressSeq_ || !held) return false;    // none, or a new press came in while reading

    const uint32_t downFor = nowMs - at;
    if (downFor < cfg_.longPressMs) return false;

    if (heldSeq_ != seq) {
        heldSeq_ = seq;
        nextRepeatMs_ = (uint32_t)cfg_.longPressMs + cfg_.repeatMs;
        if (clickSeq_ == seq) return false;      // already reported as a short press
        ev = xGuiEvent::keyEvent(key_, cfg_.repeatMs ? xKeyAction::Repeat : xKeyAction::Held);
        return true;
    }

    if (cfg_.repeatMs == 0 || downFor < nextRepeatMs_) return false;
    // Counted from now, not from when it was due: a tick that came late
    // (or a Repeat the full queue dropped) does not make a burst.
    nextRepeatMs_ = downFor + cfg_.repeatMs;
    ev = xGuiEvent::keyEvent(key_, xKeyAction::Repeat);
    return true;
}

} // namespace idisplay
