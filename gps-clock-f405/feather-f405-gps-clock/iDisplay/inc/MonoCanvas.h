#pragma once
#include <stdint.h>
#include "iTextSurface.h"

namespace idisplay {

// A one-bit-per-pixel drawing surface over a buffer the caller owns.
//
// The buffer is laid out the way the SSD1306 (and SH1106, ST7565, ...)
// take it: the screen is cut into pages 8 pixels high, each page is
// `width` bytes, one per column, and bit 0 of a byte is the top pixel
// of that column in the page. Byte (y / 8) * width + x, bit y % 8.
// A display driver can send it to the chip a page at a time unchanged.
//
// As an iTextSurface it is a grid of 6x8 cells in the 5x7 font, so
// menus written for a character LCD draw here too. Pixel drawing and
// text at any pixel position are there for screens that want them.
//
// Coordinates outside the canvas are clipped. Nothing here touches a
// bus.
class MonoCanvas : public iTextSurface {
public:
    static constexpr uint8_t kCellWidth = 6;
    static constexpr uint8_t kCellHeight = 8;

    // buf must hold width * height / 8 bytes; height a multiple of 8.
    // The buffer is not cleared here (a driver that owns it as a member
    // calls clear() once it is constructed).
    MonoCanvas(uint8_t* buf, uint16_t width, uint16_t height);

    uint16_t width() const { return width_; }
    uint16_t height() const { return height_; }
    uint8_t pages() const { return (uint8_t)(height_ / 8); }
    uint8_t* buffer() { return buf_; }
    const uint8_t* buffer() const { return buf_; }
    uint16_t bufferSize() const { return (uint16_t)(width_ * (height_ / 8)); }

    void clear() override;
    MonoCanvas* graphics() override { return this; }

    void fill(bool on);
    void drawPixel(int16_t x, int16_t y, bool on);
    bool getPixel(int16_t x, int16_t y) const;
    void drawHLine(int16_t x, int16_t y, int16_t w, bool on);
    void drawVLine(int16_t x, int16_t y, int16_t h, bool on);
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, bool on);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, bool on);
    void invertRect(int16_t x, int16_t y, int16_t w, int16_t h);

    // One character in a 6x8 cell whose top left corner is x,y. inverse
    // draws it light on dark, the blank column and row included.
    void drawChar(int16_t x, int16_t y, char c, bool inverse);
    // Draws s from x,y and returns the x just past the last character.
    int16_t drawText(int16_t x, int16_t y, const char* s, bool inverse);

protected:
    void drawCell(uint8_t col, uint8_t row, char c, bool inverse) override;

private:
    uint8_t* buf_;
    uint16_t width_, height_;
};

} // namespace idisplay
