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
};

} // namespace idisplay
