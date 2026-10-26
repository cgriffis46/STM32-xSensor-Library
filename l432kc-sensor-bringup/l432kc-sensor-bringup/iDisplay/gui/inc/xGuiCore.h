#pragma once
#include <stdint.h>
#include "xScreen.h"
#include "xGuiEvent.h"
#include "iTextSurface.h"

namespace idisplay {

// The GUI without the RTOS: a stack of screens, and what each event
// does to it. xGui (hw/freertos) owns one of these, takes events off
// its queue, hands them to handle(), and draws and flushes when
// handle() says so. A bare-metal loop can do the same.
//
// The home screen is always at the bottom of the stack.
class xGuiCore : public xNavigator {
public:
    static constexpr uint8_t kMaxDepth = 8;

    xGuiCore(iTextSurface& surface, xScreen& home);

    // Opens the home screen. Call once, before the first event.
    void start();

    // Does what ev asks. True when the display needs drawing again,
    // which is after every event the GUI acted on.
    bool handle(const xGuiEvent& ev);

    // Clears the surface and has the top screen draw on it.
    void render();

    // Back to the home screen when nobody has pressed a key for a while:
    // after `refreshes` Refresh events in a row with no other event
    // between them (a key, or a screen opened from another thread), the
    // stack is emptied as by home(). Counted in refreshes, like xHmi's
    // timeouts, so it needs xGuiTicker: refreshes = timeout / the
    // ticker's period (60 s at 500 ms: 120). 0 (the default) turns it
    // off. Already at home, nothing happens: the home screen keeps its
    // own state (xHmi has its own edit and unlock timeouts).
    void setIdleHome(uint32_t refreshes) { idleHome_ = refreshes; idle_ = 0; }
    uint32_t idleRefreshes() const { return idle_; }

    xScreen& top() const { return *stack_[depth_ - 1]; }
    uint8_t depth() const { return depth_; }

    // xNavigator. Push onto a full stack replaces the top screen; pop on
    // the home screen does nothing.
    void show(xScreen& s) override;
    void push(xScreen& s) override;
    void pop() override;
    void home() override;

private:
    iTextSurface& surface_;
    xScreen& home_;
    xScreen* stack_[kMaxDepth];
    uint8_t depth_;
    uint32_t idleHome_ = 0;
    uint32_t idle_ = 0;     // refreshes since the last other event
};

} // namespace idisplay
