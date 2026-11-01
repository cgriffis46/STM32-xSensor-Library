#pragma once
#include <stdint.h>
#include <atomic>

namespace idisplay {

// What a key (or a chord of keys) did.
//   Pressed  a short press. For a key that repeats, sent as it goes down;
//            otherwise sent when it is let go, so a long press is never
//            also a short one.
//   Held     down for the long press time (keys that don't repeat).
//   Repeat   still down: step again (keys that repeat).
struct KeyEvent {
    enum class Action : uint8_t { Pressed, Held, Repeat };
    uint16_t keys = 0;          // the key's bit, or a chord's bits
    Action action = Action::Pressed;
};

// Up to 16 keys read together as one bit mask (a GPIO port's input
// register, inverted for active-low keys), sampled at a steady rate by
// sample(). Turns that into KeyEvents:
//
//   * a mask counts once it has been the same for debounce samples;
//   * one key at a time, or one of the chords given; other combinations
//     are ignored (two keys pressed by mistake do nothing);
//   * long press after longSamples; for keys in the repeat mask, Repeat
//     after longSamples and then every repeatSamples.
//
// Events go to a one-deep mailbox: poll() takes the newest, and a newer
// one replaces one not yet taken. sample() from a timer or interrupt,
// poll() from the application's thread.
//
// The defaults are the EEVblog 121GW's stock firmware's, sampled every
// 10 ms: debounce 2 samples, long press 50 (0.5 s), repeat every 31.
class KeyScanner {
public:
    struct Config {
        uint8_t  debounce = 2;
        uint16_t longSamples = 50;
        uint16_t repeatSamples = 31;
        uint16_t repeatMask = 0;
        const uint16_t* chords = nullptr;
        uint8_t  chordCount = 0;
    };

    KeyScanner() {}
    explicit KeyScanner(const Config& c) : cfg_(c) {}

    void setRepeatMask(uint16_t m) { cfg_.repeatMask = m; }

    // down: one bit per key, 1 = pressed.
    void sample(uint16_t down) {
        if (down != last_) { last_ = down; stable_ = 0; return; }
        if (stable_ < 0xFF) stable_++;
        if (stable_ < cfg_.debounce) return;

        if (down == 0) {
            if (active_ && !longDone_ && !repeats(active_)) post(active_, KeyEvent::Action::Pressed);
            active_ = 0;
            return;
        }
        if (!valid(down)) { active_ = 0; return; }   // a combination that means nothing: cancel
        if (down != active_) {
            active_ = down;
            held_ = 0;
            longDone_ = false;
            if (repeats(down)) post(down, KeyEvent::Action::Pressed);
            return;
        }
        held_++;
        if (repeats(down)) {
            if (held_ >= cfg_.longSamples && (held_ - cfg_.longSamples) % cfg_.repeatSamples == 0)
                post(down, KeyEvent::Action::Repeat);
        } else if (!longDone_ && held_ >= cfg_.longSamples) {
            longDone_ = true;
            post(down, KeyEvent::Action::Held);
        }
    }

    bool poll(KeyEvent* out) {
        const uint32_t v = box_.exchange(0, std::memory_order_acq_rel);
        if (!v) return false;
        out->keys = (uint16_t)(v & 0xFFFF);
        out->action = (KeyEvent::Action)((v >> 16) & 0x3);
        return true;
    }

private:
    bool repeats(uint16_t m) const { return (m & cfg_.repeatMask) == m && !(m & (m - 1)); }
    bool valid(uint16_t m) const {
        if (!(m & (m - 1))) return true;
        for (uint8_t i = 0; i < cfg_.chordCount; i++) if (cfg_.chords[i] == m) return true;
        return false;
    }
    void post(uint16_t keys, KeyEvent::Action a) {
        box_.store((uint32_t)keys | ((uint32_t)a << 16) | (1u << 31), std::memory_order_release);
    }

    Config   cfg_;
    uint16_t last_ = 0, active_ = 0;
    uint8_t  stable_ = 0;
    uint16_t held_ = 0;
    bool     longDone_ = false;
    std::atomic<uint32_t> box_{0};
};

} // namespace idisplay
