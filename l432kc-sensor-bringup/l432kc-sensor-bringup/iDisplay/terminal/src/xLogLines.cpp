#include "xLogLines.h"
#include <string.h>

namespace idisplay {

// The line being built lives in row lines_, which no reader looks at;
// commit() copies it into the ring under the lock.
void xLogLinesBase::append(const char* text, size_t len) {
    if (text == nullptr) return;
    char* building = text_ + (size_t)lines_ * width_;
    uint8_t& n = len_[lines_];
    for (size_t i = 0; i < len; ++i) {
        const char c = text[i];
        if (c == '\r') continue;
        if (c == '\n') { commit(); continue; }
        if (n < width_) building[n++] = c;
    }
}

void xLogLinesBase::commit() {
    if (lock_) lock_(lockCtx_);
    memcpy(text_ + (size_t)head_ * width_, text_ + (size_t)lines_ * width_, len_[lines_]);
    len_[head_] = len_[lines_];
    head_ = (uint16_t)((head_ + 1) % lines_);
    ++total_;
    if (unlock_) unlock_(lockCtx_);
    len_[lines_] = 0;
}

uint16_t xLogLinesBase::count() const {
    const uint32_t t = total();
    return t < lines_ ? (uint16_t)t : lines_;
}

uint32_t xLogLinesBase::total() const {
    if (lock_) lock_(lockCtx_);
    const uint32_t t = total_;
    if (unlock_) unlock_(lockCtx_);
    return t;
}

size_t xLogLinesBase::copy(uint16_t back, char* out, size_t cap) const {
    if (out == nullptr || cap == 0) return 0;
    out[0] = 0;
    if (lock_) lock_(lockCtx_);
    size_t n = 0;
    const uint32_t have = total_ < lines_ ? total_ : lines_;
    if (back < have) {
        const uint16_t at = (uint16_t)((head_ + lines_ - 1u - back) % lines_);
        n = len_[at] < cap - 1 ? len_[at] : cap - 1;
        memcpy(out, text_ + (size_t)at * width_, n);
        out[n] = 0;
    }
    if (unlock_) unlock_(lockCtx_);
    return n;
}

} // namespace idisplay
