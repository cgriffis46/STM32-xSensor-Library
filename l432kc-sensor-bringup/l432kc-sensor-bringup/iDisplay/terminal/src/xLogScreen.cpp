#include "xLogScreen.h"

namespace idisplay {

void xLogScreen::draw(const xLogLinesBase& log, iTextSurface& s, uint8_t col, uint8_t row, uint8_t width,
                      uint8_t height, uint16_t back, uint8_t shift) {
    char line[256];
    for (uint8_t r = 0; r < height; ++r) {
        const uint32_t which = (uint32_t)back + (height - 1u - r);
        const size_t n = which <= 0xFFFF ? log.copy((uint16_t)which, line, sizeof line) : 0;
        s.setCursor(col, (uint8_t)(row + r));
        for (uint8_t x = 0; x < width; ++x) {
            const size_t i = (size_t)shift + x;
            s.putChar(i < n ? line[i] : ' ');
        }
    }
}

void xLogScreen::onEnter(xNavigator& nav) {
    (void)nav;
    back_ = 0;
    shift_ = 0;
    seen_ = log_.total();
}

void xLogScreen::onKey(xKey key, xKeyAction action, xNavigator& nav) {
    if (action != xKeyAction::Pressed && action != xKeyAction::Repeat) return;
    const uint16_t count = log_.count();
    const uint16_t most = count > rows_ ? (uint16_t)(count - rows_) : 0;
    const uint8_t half = (uint8_t)(cols_ / 2 ? cols_ / 2 : 1);
    switch (key) {
    case xKey::Up: if (back_ < most) ++back_; break;
    case xKey::Down: if (back_ > 0) --back_; break;
    case xKey::Right: if (shift_ + cols_ < log_.width()) shift_ = (uint8_t)(shift_ + half); break;
    case xKey::Left: shift_ = shift_ > half ? (uint8_t)(shift_ - half) : 0; break;
    case xKey::Back: if (action == xKeyAction::Pressed) nav.pop(); break;
    default: break;
    }
}

void xLogScreen::render(iTextSurface& s) {
    rows_ = s.rows();
    cols_ = s.cols();
    // Scrolled back: keep the same lines in view as new ones come.
    const uint32_t total = log_.total();
    if (back_ > 0) {
        const uint32_t moved = total - seen_;
        const uint32_t b = back_ + moved;
        const uint16_t count = log_.count();
        const uint16_t most = count > rows_ ? (uint16_t)(count - rows_) : 0;
        back_ = (uint16_t)(b > most ? most : b);
    }
    seen_ = total;
    draw(log_, s, 0, 0, cols_, rows_, back_, shift_);
}

} // namespace idisplay
