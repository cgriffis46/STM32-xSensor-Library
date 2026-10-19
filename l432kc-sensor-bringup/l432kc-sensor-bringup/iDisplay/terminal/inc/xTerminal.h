#pragma once
#include <stddef.h>
#include <stdint.h>
#include "iTextSurface.h"
#include "iDisplayDevice.h"
#include "iTransport.h"
#include "xLogLines.h"

namespace idisplay {

// A serial terminal (PuTTY, screen, minicom, Tera Term: anything VT100)
// as a display: the same screens, menus and HMI as the panel, drawn over
// a UART (any iTransport).
//
//   static xTerminal<20, 4> term(uart, xTerminalConfig());
//   static xGui gui(term, hmi);
//
// The surface is cols x rows character cells, the size of the panel, so
// a screen laid out for a 20x4 LCD looks the same; give it a frame to
// see where it ends. Below it there can be a pane of log lines (an
// xLogLines, fed from the debug log's tap): LOG_ROWS rows LOG_COLS wide
// (at least as wide as the framed surface). The terminal draws that pane
// itself, on each flush.
//
// It sends only what changed since the last flush: the cursor moved
// (ESC[r;cH) to each run of changed cells, inverse on and off (ESC[7m,
// ESC[27m), the characters. The degree sign goes out as UTF-8 (C2 B0),
// so set the terminal to UTF-8. Characters outside printable ASCII show
// as '?'. The first flush, and the first after a failure or redraw(),
// clears the terminal (ESC[2J) and hides its cursor (ESC[?25l), then
// sends everything. Inverse is shown, so showEditCursor() is ignored, as
// on an OLED.
//
// Sending: two 128 byte buffers in turn (the transport may send from one
// after write() returns; it is not touched again until the other has
// been accepted). While the UART is busy main() calls config.sleep (give
// it osDelay on an RTOS, or nothing to spin), and after stuckMs of
// refusals the terminal fails: failed() for backoffMs, then everything
// again. idle() means everything has been handed to the transport; the
// last buffer may still be going out.
//
// A terminal that was closed and opened again shows nothing until the
// next change: redraw() (Ctrl-L, see xTerminalKeys) sends it all again.
struct xTerminalConfig {
    bool frame = true;                      // a box of + - | around the surface
    void (*sleep)(uint32_t ms) = nullptr;   // while the UART is busy
    uint32_t stuckMs = 1000;
    uint32_t backoffMs = 2000;
    const xLogLinesBase* log = nullptr;     // the log pane's lines (LOG_ROWS > 0)
};

class xTerminalBase : public iTextSurface, public iDisplayDevice {
public:
    static constexpr size_t kTxBuffer = 128;

    iTextSurface& text() override { return *this; }
    void clear() override;
    void requestFlush() override { flushWanted_ = true; }
    void main(uint32_t nowMs) override;
    bool idle() const override { return !flushWanted_ && !scanning_ && txLen_ == 0; }
    bool failed() const override { return failed_; }

    // Clears the terminal and sends everything at the next flush. Safe
    // from an interrupt or another thread (it only sets a flag).
    void redraw() { redrawWanted_ = true; }

    // Bytes handed to the transport, and flushes that failed.
    uint32_t bytesSent() const { return bytesSent_; }
    uint32_t failures() const { return failures_; }

    // The whole terminal area, frame and log pane included.
    uint8_t width() const { return w_; }
    uint8_t height() const { return h_; }

protected:
    xTerminalBase(iTransport& out, const xTerminalConfig& config, uint8_t cols, uint8_t rows,
                  uint8_t logRows, uint8_t logCols, uint8_t* want, uint8_t* sent);
    void drawCell(uint8_t col, uint8_t row, char c, bool inverse) override;

private:
    static constexpr uint8_t kInverse = 0x80;
    static constexpr uint8_t kUnknown = 0xFF;   // in sent_: repaint it

    void ensureInit();
    void drawFrame();
    void drawLog();
    void startFlush();
    void scan();
    void put(const char* s);
    void putNumber(uint8_t v);

    iTransport& out_;
    xTerminalConfig config_;
    uint8_t w_, h_;         // the whole area
    uint8_t x0_, y0_;       // the surface's top left within it
    uint8_t logRows_, logTop_;
    uint8_t* want_;         // w_ * h_: char | kInverse
    uint8_t* sent_;

    uint8_t tx_[2][kTxBuffer];
    uint8_t txNext_ = 0;
    size_t txLen_ = 0;
    uint16_t pos_ = 0;      // scan position
    int16_t at_ = -1;       // where the terminal's cursor is (-1 unknown)
    bool attrInverse_ = false;
    bool init_ = false;
    bool started_ = false;
    bool flushWanted_ = false;
    bool scanning_ = false;
    volatile bool redrawWanted_ = false;
    bool failed_ = false;
    bool refusing_ = false;
    uint32_t refusedSince_ = 0;
    uint32_t failedAt_ = 0;
    uint32_t bytesSent_ = 0;
    uint32_t failures_ = 0;
};

// The size is fixed at compile time, so the buffers are members: 2 bytes
// a cell of the whole area (a framed 20x4 with a 6 x 40 log pane: 2 * 40
// * 13 = 1040 bytes) plus 256 for sending.
template <uint8_t COLS, uint8_t ROWS, uint8_t LOG_ROWS = 0, uint8_t LOG_COLS = 0>
class xTerminal : public xTerminalBase {
    static constexpr unsigned kFramedW = COLS + 2u;
    static constexpr unsigned kW = LOG_ROWS && LOG_COLS > kFramedW ? LOG_COLS : kFramedW;
    static constexpr unsigned kH = ROWS + 2u + (LOG_ROWS ? LOG_ROWS + 1u : 0u);
    static_assert(COLS > 0 && ROWS > 0 && kW <= 250 && kH <= 250, "xTerminal: size");
public:
    xTerminal(iTransport& out, const xTerminalConfig& config)
        : xTerminalBase(out, config, COLS, ROWS, LOG_ROWS, LOG_COLS, want_, sent_) {}
private:
    uint8_t want_[kW * kH];
    uint8_t sent_[kW * kH];
};

} // namespace idisplay
