#pragma once
#include <stdint.h>

namespace idisplay {

// Something that beeps. The application asks for a pattern; how the sound
// is made is the implementation's.
class iBeeper {
public:
    virtual ~iBeeper() = default;
    // count beeps of onMs, offMs apart (0: as long as onMs). A new pattern
    // replaces one still sounding.
    virtual void beep(uint16_t onMs, uint8_t count = 1, uint16_t offMs = 0) = 0;
    virtual void stop() = 0;
    virtual bool beeping() const = 0;

    // The 121GW's stock sounds: a key taken, a key taken that changes a
    // lot, and no.
    void ok()      { beep(100); }
    void excited() { beep(200); }
    void no()      { beep(50, 2, 50); }
};

// A buzzer that sounds while a pin is high: a self-oscillating piezo on a
// transistor, as on the 121GW (PE7, Q8, BZ2). Non-blocking: tick(nowMs),
// from a 1 ms timer or any loop, switches the pin at the pattern's edges.
//
// TPin: void set(bool on). (BuzzerPinL1 on the STM32L1, a test's fake.)
template <typename TPin>
class Buzzer : public iBeeper {
public:
    template <typename... TArgs>
    explicit Buzzer(TArgs&&... pinArgs) : pin_(static_cast<TArgs&&>(pinArgs)...) { pin_.set(false); }

    void beep(uint16_t onMs, uint8_t count = 1, uint16_t offMs = 0) override {
        if (count == 0 || onMs == 0) { stop(); return; }
        onMs_ = onMs;
        offMs_ = offMs ? offMs : onMs;
        left_ = count;
        started_ = false;   // starts at the next tick, which knows the time
        active_ = true;
    }

    void stop() override { active_ = false; on_ = false; pin_.set(false); }
    bool beeping() const override { return active_; }

    void tick(uint32_t nowMs) {
        if (!active_) return;
        if (!started_) { started_ = true; on_ = true; edge_ = nowMs; pin_.set(true); return; }
        const uint32_t t = nowMs - edge_;
        if (on_ && t >= onMs_) {
            on_ = false;
            pin_.set(false);
            edge_ = nowMs;
            if (--left_ == 0) active_ = false;
        } else if (!on_ && t >= offMs_) {
            on_ = true;
            pin_.set(true);
            edge_ = nowMs;
        }
    }

    TPin& pin() { return pin_; }

private:
    TPin pin_;
    uint16_t onMs_ = 0, offMs_ = 0;
    uint8_t  left_ = 0;
    uint32_t edge_ = 0;
    volatile bool active_ = false;
    bool started_ = false, on_ = false;
};

} // namespace idisplay
