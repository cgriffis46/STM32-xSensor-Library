#include "iTextSurface.h"

namespace idisplay {

void iTextSurface::putChar(char c) {
    if (c == '\n') {
        col_ = 0;
        if (row_ < 255) ++row_;
        return;
    }
    if (col_ < cols_ && row_ < rows_) drawCell(col_, row_, c, inverse_);
    if (col_ < 255) ++col_;
}

void iTextSurface::print(const char* s) {
    if (s == nullptr) return;
    while (*s) putChar(*s++);
}

void iTextSurface::print(int32_t v) {
    char buf[12];
    uint8_t n = 0;
    // Work in unsigned so INT32_MIN does not overflow when negated.
    uint32_t u = v < 0 ? 0u - (uint32_t)v : (uint32_t)v;
    do {
        buf[n++] = (char)('0' + u % 10);
        u /= 10;
    } while (u != 0);
    if (v < 0) putChar('-');
    while (n > 0) putChar(buf[--n]);
}

void iTextSurface::print(float v, uint8_t decimals) {
    if (v != v) {   // NAN
        print("--");
        return;
    }
    if (decimals > 6) decimals = 6;
    uint32_t scale = 1;
    for (uint8_t i = 0; i < decimals; ++i) scale *= 10;
    const bool neg = v < 0;
    if (neg) v = -v;
    // Rounded to the last decimal shown. Beyond what 32 bits hold,
    // there is nothing useful to show on a small display anyway.
    const float scaled = v * (float)scale + 0.5f;
    if (scaled >= 4294967040.0f) {
        print(neg ? "-ovf" : "ovf");
        return;
    }
    const uint32_t fixed = (uint32_t)scaled;
    const uint32_t whole = fixed / scale;
    uint32_t frac = fixed % scale;
    if (neg && fixed != 0) putChar('-');
    // whole may be above INT32_MAX; print it digit by digit.
    char buf[11];
    uint8_t n = 0;
    uint32_t w = whole;
    do {
        buf[n++] = (char)('0' + w % 10);
        w /= 10;
    } while (w != 0);
    while (n > 0) putChar(buf[--n]);
    if (decimals == 0) return;
    putChar('.');
    for (uint32_t d = scale / 10; d > 0; d /= 10) {
        putChar((char)('0' + frac / d));
        frac %= d;
    }
}

void iTextSurface::padToEndOfRow() {
    while (col_ < cols_) putChar(' ');
}

} // namespace idisplay
