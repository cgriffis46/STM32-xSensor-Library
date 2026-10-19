#pragma once
#include <stdint.h>
#include "iTextSurface.h"

namespace idisplay {

// A time of day on the display, at a fixed place: xTimeHHMM shows
// "HH:MM", xTimeHHMMSS "HH:MM:SS" (24 hour, zero padded). It holds the
// time it was last given; update() writes it to its place on the
// surface. Until a time is set it shows dashes ("--:--"), so a clock
// that has not been set yet (no RTC, no SNTP) does not look right.
//
// Like everything drawn, update() belongs in a screen's render(), in the
// GUI task. A thread that knows the time sets it and posts a refresh:
//
//   xTimeHHMM clock(11, 0);                     // top right of a 16x2 LCD
//
//   void render(iTextSurface& s) override {     // the home screen
//       ...
//       clock.update(s);
//   }
//
//   clock.setFromSecondsOfDay(unixSeconds % 86400);     // data thread
//   gui.post(xGuiEvent::refresh());
//
// The HD44780 driver sends only the characters that changed and the
// SSD1306 only the pages, so updating every second costs a digit or two.
// set() is a few byte stores; between threads the worst a torn read can
// do is show one stale digit until the next refresh.
class xTimeHHMM {
public:
    static constexpr uint8_t kWidth = 5;            // "HH:MM"

    explicit xTimeHHMM(uint8_t col = 0, uint8_t row = 0) : col_(col), row_(row) {}

    void setPosition(uint8_t col, uint8_t row) { col_ = col; row_ = row; }
    uint8_t col() const { return col_; }
    uint8_t row() const { return row_; }

    // False, and nothing changes, if hours > 23 or minutes > 59.
    bool set(uint8_t hours, uint8_t minutes);
    // From seconds since midnight; whole days are dropped.
    void setFromSecondsOfDay(uint32_t seconds);
    // Back to dashes.
    void invalidate() { valid_ = false; }

    bool valid() const { return valid_; }
    uint8_t hours() const { return h_; }
    uint8_t minutes() const { return m_; }

    // The character between the fields: ':' unless set otherwise; 0 for
    // none ("HHMM", one character narrower). A screen can blink it by
    // setting ' ' and ':' on alternate seconds.
    void setSeparator(char c) { sep_ = c; }
    char separator() const { return sep_; }

    // Characters update() writes.
    uint8_t width() const { return (uint8_t)(fields() * 2 + (sep_ ? fields() - 1 : 0)); }

    // Writes the time at its place. The cursor is left after it.
    void update(iTextSurface& s) const;
    // Writes it at the cursor instead, like the fields do.
    void render(iTextSurface& s) const;

protected:
    xTimeHHMM(uint8_t col, uint8_t row, uint8_t fields) : col_(col), row_(row), fields_(fields) {}
    uint8_t fields() const { return fields_; }
    uint8_t field(uint8_t i) const { return i == 0 ? h_ : i == 1 ? m_ : s_; }

    uint8_t col_, row_;
    uint8_t fields_ = 2;
    uint8_t h_ = 0, m_ = 0, s_ = 0;
    char sep_ = ':';
    bool valid_ = false;
};

// Hours, minutes and seconds: "HH:MM:SS".
class xTimeHHMMSS : public xTimeHHMM {
public:
    static constexpr uint8_t kWidth = 8;            // "HH:MM:SS"

    explicit xTimeHHMMSS(uint8_t col = 0, uint8_t row = 0) : xTimeHHMM(col, row, 3) {}

    using xTimeHHMM::set;                           // set(h, m): seconds 0
    // False, and nothing changes, if any is out of range.
    bool set(uint8_t hours, uint8_t minutes, uint8_t seconds);
    uint8_t seconds() const { return s_; }
};

} // namespace idisplay
