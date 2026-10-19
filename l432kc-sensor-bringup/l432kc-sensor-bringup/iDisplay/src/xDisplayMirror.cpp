#include "xDisplayMirror.h"

namespace idisplay {

static uint8_t smaller(uint8_t x, uint8_t y) { return x < y ? x : y; }

xDisplayMirror::xDisplayMirror(iDisplayDevice& a, iDisplayDevice& b)
    : iTextSurface(smaller(a.text().cols(), b.text().cols()), smaller(a.text().rows(), b.text().rows())),
      a_(a), b_(b) {}

void xDisplayMirror::clear() {
    a_.text().clear();
    b_.text().clear();
    homeCursor();
}

void xDisplayMirror::showEditCursor(uint8_t col, uint8_t row) {
    a_.text().showEditCursor(col, row);
    b_.text().showEditCursor(col, row);
}

void xDisplayMirror::put(iTextSurface& s, uint8_t col, uint8_t row, char c, bool inverse) {
    s.setCursor(col, row);
    s.setInverse(inverse);
    s.putChar(c);
    s.setInverse(false);
}

void xDisplayMirror::drawCell(uint8_t col, uint8_t row, char c, bool inverse) {
    put(a_.text(), col, row, c, inverse);
    put(b_.text(), col, row, c, inverse);
}

} // namespace idisplay
