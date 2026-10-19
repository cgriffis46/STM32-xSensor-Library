#include "xTerminalKeys.h"

namespace idisplay {

bool xTerminalKeys::arrow(uint8_t final) {
    switch (final) {
    case 'A': press(xKey::Up); return true;
    case 'B': press(xKey::Down); return true;
    case 'C': press(xKey::Right); return true;
    case 'D': press(xKey::Left); return true;
    default: return false;
    }
}

void xTerminalKeys::ground(uint8_t b) {
    const bool cr = b == '\r';
    if (b == '\n' && lastCr_) { lastCr_ = false; return; }    // CR LF: one Enter
    lastCr_ = cr;
    if (cr || b == '\n') press(xKey::Enter);
    else if (b == 0x08 || b == 0x7F) press(xKey::Back);
    else if (b == 0x0C) { if (redraw_) redraw_(redrawCtx_); }
    else if (b == 0x1B) state_ = State::Esc;
}

void xTerminalKeys::onByteReceived(uint8_t b) {
    switch (state_) {
    case State::Ground:
        ground(b);
        return;
    case State::Esc:
        lastCr_ = false;
        if (b == '[') { state_ = State::Csi; return; }
        if (b == 'O') { state_ = State::Ss3; return; }
        state_ = State::Ground;
        if (b == 0x1B) { press(xKey::Back); return; }          // Esc Esc
        ground(b);                                              // Esc, then something else
        return;
    case State::Csi:
        // Parameters (0x30-0x3F) and intermediates (0x20-0x2F) up to the
        // final byte (0x40-0x7E); anything else ends it.
        if (b >= 0x20 && b <= 0x3F) return;
        state_ = State::Ground;
        if (b >= 0x40 && b <= 0x7E) arrow(b);
        else ground(b);
        return;
    case State::Ss3:
        state_ = State::Ground;
        if (!arrow(b)) ground(b);
        return;
    }
}

} // namespace idisplay
