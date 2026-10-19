#include "xTime.h"

namespace idisplay {

bool xTimeHHMM::set(uint8_t hours, uint8_t minutes) {
    if (hours > 23 || minutes > 59) return false;
    h_ = hours;
    m_ = minutes;
    s_ = 0;
    valid_ = true;
    return true;
}

void xTimeHHMM::setFromSecondsOfDay(uint32_t seconds) {
    seconds %= 86400u;
    h_ = (uint8_t)(seconds / 3600u);
    m_ = (uint8_t)(seconds / 60u % 60u);
    s_ = (uint8_t)(seconds % 60u);
    valid_ = true;
}

void xTimeHHMM::update(iTextSurface& s) const {
    s.setCursor(col_, row_);
    render(s);
}

void xTimeHHMM::render(iTextSurface& s) const {
    for (uint8_t i = 0; i < fields_; ++i) {
        if (i > 0 && sep_) s.putChar(sep_);
        const uint8_t v = field(i);
        s.putChar(valid_ ? (char)('0' + v / 10) : '-');
        s.putChar(valid_ ? (char)('0' + v % 10) : '-');
    }
}

bool xTimeHHMMSS::set(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    if (hours > 23 || minutes > 59 || seconds > 59) return false;
    h_ = hours;
    m_ = minutes;
    s_ = seconds;
    valid_ = true;
    return true;
}

} // namespace idisplay
