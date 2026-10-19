#pragma once
#include <stdint.h>
#include "xScreen.h"
#include "xLogLines.h"

namespace idisplay {

// The log on a screen of the GUI (the panel's or the terminal's): the
// newest lines at the bottom, one a row, cut at the right edge. Up goes
// back to older lines, Down forward again; Left and Right move sideways
// by half the width, for lines wider than the display; Back goes back
// (pop). While it shows the newest line it follows new ones; scrolled
// back, it stays where it is as lines come (until they push the ones it
// shows out of xLogLines).
//
//   static xLogScreen logScreen(logLines);
//   ... a menu entry pushes it; xGuiTicker's refreshes redraw it.
class xLogScreen : public xScreen {
public:
    explicit xLogScreen(const xLogLinesBase& log) : log_(log) {}

    void onEnter(xNavigator& nav) override;
    void onKey(xKey key, xKeyAction action, xNavigator& nav) override;
    void render(iTextSurface& s) override;

    // Lines back from the newest at the bottom row, and the first column.
    uint16_t back() const { return back_; }
    uint8_t shift() const { return shift_; }

    // Draws the last rows of log, back lines up from the newest, into a
    // rectangle (what render() and xTerminal's pane do).
    static void draw(const xLogLinesBase& log, iTextSurface& s, uint8_t col, uint8_t row, uint8_t width,
                     uint8_t height, uint16_t back = 0, uint8_t shift = 0);

private:
    const xLogLinesBase& log_;
    uint16_t back_ = 0;
    uint8_t shift_ = 0;
    uint8_t rows_ = 1, cols_ = 1;   // from the last render
    uint32_t seen_ = 0;             // log total at the last render
};

} // namespace idisplay
