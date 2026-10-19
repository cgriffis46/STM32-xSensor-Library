#pragma once
#include <stdint.h>
#include "iTextSurface.h"
#include "iDisplayDevice.h"

namespace idisplay {

// One GUI on two displays at once: the panel and a serial terminal
// (xTerminal), say. Everything drawn goes to both, and a flush flushes
// both. Give it to xGui in place of a display:
//
//   static xDisplayMirror both(lcd, term);
//   static xGui gui(both, hmi);
//
// Make them the same size (an xTerminal<20, 4> next to a 20x4 LCD); a
// bigger one shows the smaller one's area only (the mirror is the
// smaller of the two in each direction). Character surface only:
// graphics() is nullptr, so a screen that draws pixels falls back to
// text. Inverse shows where each display can show it, and the edit
// cursor where each puts it.
//
// main() runs both. idle() once each is idle or failed, so a display
// that failed (an unplugged terminal) doesn't hold up the other;
// failed() only when both have.
class xDisplayMirror : public iTextSurface, public iDisplayDevice {
public:
    xDisplayMirror(iDisplayDevice& a, iDisplayDevice& b);

    iTextSurface& text() override { return *this; }
    void clear() override;
    void showEditCursor(uint8_t col, uint8_t row) override;
    void requestFlush() override { a_.requestFlush(); b_.requestFlush(); }
    void main(uint32_t nowMs) override { a_.main(nowMs); b_.main(nowMs); }
    bool idle() const override {
        return (a_.idle() || a_.failed()) && (b_.idle() || b_.failed());
    }
    bool failed() const override { return a_.failed() && b_.failed(); }

protected:
    void drawCell(uint8_t col, uint8_t row, char c, bool inverse) override;

private:
    static void put(iTextSurface& s, uint8_t col, uint8_t row, char c, bool inverse);

    iDisplayDevice& a_;
    iDisplayDevice& b_;
};

} // namespace idisplay
