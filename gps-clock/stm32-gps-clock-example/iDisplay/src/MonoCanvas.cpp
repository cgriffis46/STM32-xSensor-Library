#include "MonoCanvas.h"
#include <string.h>
#include "Font5x7.h"

namespace idisplay {

MonoCanvas::MonoCanvas(uint8_t* buf, uint16_t width, uint16_t height)
    : iTextSurface((uint8_t)(width / kCellWidth), (uint8_t)(height / kCellHeight)),
      buf_(buf), width_(width), height_(height) {}

void MonoCanvas::clear() {
    memset(buf_, 0, bufferSize());
    homeCursor();
}

void MonoCanvas::fill(bool on) {
    memset(buf_, on ? 0xFF : 0x00, bufferSize());
}

void MonoCanvas::drawPixel(int16_t x, int16_t y, bool on) {
    if (x < 0 || y < 0 || x >= (int16_t)width_ || y >= (int16_t)height_) return;
    uint8_t& b = buf_[(y / 8) * width_ + x];
    const uint8_t bit = (uint8_t)(1u << (y & 7));
    if (on) b |= bit; else b &= (uint8_t)~bit;
}

bool MonoCanvas::getPixel(int16_t x, int16_t y) const {
    if (x < 0 || y < 0 || x >= (int16_t)width_ || y >= (int16_t)height_) return false;
    return (buf_[(y / 8) * width_ + x] >> (y & 7)) & 1;
}

void MonoCanvas::drawHLine(int16_t x, int16_t y, int16_t w, bool on) {
    for (int16_t i = 0; i < w; ++i) drawPixel(x + i, y, on);
}

void MonoCanvas::drawVLine(int16_t x, int16_t y, int16_t h, bool on) {
    for (int16_t i = 0; i < h; ++i) drawPixel(x, y + i, on);
}

void MonoCanvas::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, bool on) {
    if (w <= 0 || h <= 0) return;
    drawHLine(x, y, w, on);
    drawHLine(x, y + h - 1, w, on);
    drawVLine(x, y, h, on);
    drawVLine(x + w - 1, y, h, on);
}

void MonoCanvas::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, bool on) {
    for (int16_t j = 0; j < h; ++j) drawHLine(x, y + j, w, on);
}

void MonoCanvas::invertRect(int16_t x, int16_t y, int16_t w, int16_t h) {
    for (int16_t j = 0; j < h; ++j)
        for (int16_t i = 0; i < w; ++i) drawPixel(x + i, y + j, !getPixel(x + i, y + j));
}

void MonoCanvas::drawChar(int16_t x, int16_t y, char c, bool inverse) {
    const uint8_t* g = fontGlyph(c);
    for (uint8_t col = 0; col < kCellWidth; ++col) {
        // Bit 7 of the column is the blank row under the glyph.
        const uint8_t bits = col < kFontWidth ? (uint8_t)(g[col] & 0x7F) : 0;
        for (uint8_t row = 0; row < kCellHeight; ++row) {
            const bool lit = (bits >> row) & 1;
            drawPixel(x + col, y + row, lit != inverse);
        }
    }
}

int16_t MonoCanvas::drawText(int16_t x, int16_t y, const char* s, bool inverse) {
    if (s == nullptr) return x;
    while (*s) {
        drawChar(x, y, *s++, inverse);
        x += kCellWidth;
    }
    return x;
}

void MonoCanvas::drawCell(uint8_t col, uint8_t row, char c, bool inverse) {
    drawChar((int16_t)(col * kCellWidth), (int16_t)(row * kCellHeight), c, inverse);
}

} // namespace idisplay
