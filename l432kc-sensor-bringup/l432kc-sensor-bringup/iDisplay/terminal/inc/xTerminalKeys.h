#pragma once
#include <stdint.h>
#include "iTransport.h"
#include "xGuiEvent.h"

namespace idisplay {

// A terminal's keyboard as the GUI's keys: attach it to the UART the
// xTerminal draws on and it turns what the terminal sends into key
// presses.
//
//   Arrow keys          Up, Down, Left, Right (ESC [ A..D, or ESC O A..D
//                       in application cursor mode; modifiers ignored)
//   Enter               Enter (CR, LF or CR LF: one press)
//   Backspace, Delete   Back (0x08, 0x7F), and Esc pressed twice
//   Ctrl-L              redraw: the terminal is cleared and sent again
//
// A lone Esc can't be told from the start of an arrow key until the next
// byte, so Esc alone does nothing; press it twice. A held key repeats in
// the terminal, which sends it again: each comes as Pressed (a terminal
// sends no releases). Other bytes are ignored.
//
//   static xTerminalKeys keys(
//       [](xKey k, xKeyAction a, void* g) { static_cast<xGui*>(g)->post(xGuiEvent::keyEvent(k, a)); }, &gui,
//       [](void* t) { static_cast<xTerminalBase*>(t)->redraw(); }, &term);
//   uart.setRxSink(keys);
//
// onByteReceived() runs where the transport calls it (often the UART
// interrupt), and so do the callbacks: post with no timeout there. The
// redraw callback should only ask for a redraw (xTerminalBase::redraw()
// sets a flag) and then post a refresh, so the GUI flushes.
class xTerminalKeys : public iTransportRxSink {
public:
    typedef void (*KeyFn)(xKey key, xKeyAction action, void* ctx);
    typedef void (*RedrawFn)(void* ctx);

    xTerminalKeys(KeyFn key, void* keyCtx, RedrawFn redraw = nullptr, void* redrawCtx = nullptr)
        : key_(key), keyCtx_(keyCtx), redraw_(redraw), redrawCtx_(redrawCtx) {}

    void onByteReceived(uint8_t byte) override;

private:
    enum class State : uint8_t { Ground, Esc, Csi, Ss3 };
    void press(xKey k) { if (key_) key_(k, xKeyAction::Pressed, keyCtx_); }
    bool arrow(uint8_t final);
    void ground(uint8_t byte);

    KeyFn key_;
    void* keyCtx_;
    RedrawFn redraw_;
    void* redrawCtx_;
    State state_ = State::Ground;
    bool lastCr_ = false;
};

} // namespace idisplay
