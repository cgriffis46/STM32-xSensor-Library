#pragma once
#include <stdint.h>

namespace idisplay {

class xScreen;

// The keys a GUI understands. A board with three buttons uses Up, Down
// and Enter; Back, Left and Right are there for boards with more.
enum class xKey : uint8_t { Up, Down, Enter, Back, Left, Right };

// What a button did (see xButtonConfig for which a button sends):
//   Pressed   a press: as it goes down, or, on a long press button, a
//             short press, sent when it is let go
//   Released  let go (only when the button reports releases)
//   Held      a long press button has been down for its long press time
//   Repeat    an auto-repeat button is still down: step again
enum class xKeyAction : uint8_t { Pressed, Released, Held, Repeat };

enum class xGuiEventType : uint8_t {
    Key,        // a button: key, action
    Refresh,    // new data to show: the top screen's onRefresh(arg), then a redraw
    Show,       // screen replaces the top of the stack
    Push,       // screen goes on top of the stack; Pop comes back
    Pop,        // back to the screen underneath
    Home        // back to the home screen, emptying the stack
};

// Everything that reaches the GUI goes through its event queue as one
// of these: button presses, new data, and screen changes asked for
// from other threads. Plain data, copied into the queue.
struct xGuiEvent {
    xGuiEventType type;
    xKey key;
    xKeyAction action;
    xScreen* screen;
    uint32_t arg;

    static xGuiEvent keyEvent(xKey k, xKeyAction a) { return make(xGuiEventType::Key, k, a, nullptr, 0); }
    static xGuiEvent refresh(uint32_t arg = 0) { return make(xGuiEventType::Refresh, xKey::Up, xKeyAction::Pressed, nullptr, arg); }
    static xGuiEvent show(xScreen& s) { return make(xGuiEventType::Show, xKey::Up, xKeyAction::Pressed, &s, 0); }
    static xGuiEvent push(xScreen& s) { return make(xGuiEventType::Push, xKey::Up, xKeyAction::Pressed, &s, 0); }
    static xGuiEvent pop() { return make(xGuiEventType::Pop, xKey::Up, xKeyAction::Pressed, nullptr, 0); }
    static xGuiEvent home() { return make(xGuiEventType::Home, xKey::Up, xKeyAction::Pressed, nullptr, 0); }

private:
    static xGuiEvent make(xGuiEventType t, xKey k, xKeyAction a, xScreen* s, uint32_t arg) {
        xGuiEvent e;
        e.type = t; e.key = k; e.action = a; e.screen = s; e.arg = arg;
        return e;
    }
};

} // namespace idisplay
