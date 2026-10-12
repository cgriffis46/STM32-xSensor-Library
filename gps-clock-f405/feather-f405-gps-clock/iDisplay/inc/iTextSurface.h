#pragma once
#include <stdint.h>

namespace idisplay {

class MonoCanvas;

// The degree sign, on every surface: the 5x7 font draws it, and a
// character LCD driver sends its own ROM's. print("21.5\x7F") or
// putChar(kDegreeChar).
static constexpr char kDegreeChar = '\x7F';

// iTextSurface is what a screen draws on: a grid of character cells,
// cols() wide and rows() high, with a cursor. A graphic display
// (MonoCanvas, 6x8 pixel cells) and a character LCD both provide one,
// so the same menus and fields run on either.
//
// Nothing here talks to a bus. Drawing only changes a buffer in RAM;
// the display device sends it when asked (iDisplayDevice::requestFlush).
//
// Text that runs past the end of a row is clipped, not wrapped. '\n'
// moves the cursor to the start of the next row.
class iTextSurface {
public:
    virtual ~iTextSurface() {}

    uint8_t cols() const { return cols_; }
    uint8_t rows() const { return rows_; }

    // Blanks the whole surface, puts the cursor at 0,0 and turns
    // inverse off.
    virtual void clear() = 0;

    // The pixel canvas behind this surface, or nullptr on a character
    // display. A screen can draw graphics when it is there and fall
    // back to text when it is not; there is no RTTI to ask otherwise.
    virtual MonoCanvas* graphics() { return nullptr; }

    // Marks the cell being edited (a text field's cursor). A display
    // that can draw inverse shows that already and ignores this; a
    // character LCD, which cannot, puts its blinking cursor there.
    // clear() takes the mark away.
    virtual void showEditCursor(uint8_t col, uint8_t row) { (void)col; (void)row; }

    void setCursor(uint8_t col, uint8_t row) { col_ = col; row_ = row; }
    uint8_t cursorCol() const { return col_; }
    uint8_t cursorRow() const { return row_; }

    // Inverse video for the characters written from now on. A display
    // that cannot show it (most character LCDs) ignores it.
    void setInverse(bool on) { inverse_ = on; }
    bool inverse() const { return inverse_; }

    void putChar(char c);
    void print(const char* s);
    void print(int32_t v);
    // Fixed point, no printf: newlib-nano leaves out %f. NAN prints as
    // "--", so a reading that has not arrived yet is plain to see.
    void print(float v, uint8_t decimals);
    void printAt(uint8_t col, uint8_t row, const char* s) { setCursor(col, row); print(s); }
    // Spaces from the cursor to the end of its row, in the current
    // inverse setting: a highlighted bar for a selected menu line.
    void padToEndOfRow();

protected:
    iTextSurface(uint8_t cols, uint8_t rows) : cols_(cols), rows_(rows) {}

    // Draws one character in one cell. col < cols(), row < rows().
    virtual void drawCell(uint8_t col, uint8_t row, char c, bool inverse) = 0;

    void homeCursor() { col_ = 0; row_ = 0; inverse_ = false; }

private:
    uint8_t cols_, rows_;
    uint8_t col_ = 0, row_ = 0;
    bool inverse_ = false;
};

} // namespace idisplay
