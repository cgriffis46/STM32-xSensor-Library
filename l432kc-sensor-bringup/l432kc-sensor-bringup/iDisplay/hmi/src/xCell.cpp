#include "xCell.h"
#include <math.h>
#include <string.h>
#include "xFormat.h"

namespace idisplay {

xLabel xLabel::of(const char* caption, const float& v, uint8_t decimals, const char* unit) {
    xLabel l(caption);
    l.source_ = Source::Float; l.ptr_ = &v; l.decimals_ = decimals; l.unit_ = unit;
    return l;
}

xLabel xLabel::of(const char* caption, const int32_t& v, const char* unit) {
    xLabel l(caption);
    l.source_ = Source::Int; l.ptr_ = &v; l.unit_ = unit;
    return l;
}

xLabel xLabel::of(const char* caption, const bool& v, const char* on, const char* off) {
    xLabel l(caption);
    l.source_ = Source::Bool; l.ptr_ = &v; l.on_ = on; l.off_ = off;
    return l;
}

xLabel xLabel::fn(const char* caption, Getter get, const void* ctx, uint8_t decimals, const char* unit) {
    xLabel l(caption);
    l.source_ = Source::Getter; l.get_ = get; l.ptr_ = ctx; l.decimals_ = decimals; l.unit_ = unit;
    return l;
}

xLabel xLabel::tag(const char* caption, iTagSource& source, const char* name, uint8_t decimals, const char* unit) {
    xLabel l(caption);
    l.source_ = Source::Tag; l.tags_ = &source; l.name_ = name; l.decimals_ = decimals; l.unit_ = unit;
    return l;
}

xLabel xLabel::tagState(const char* caption, iTagSource& source, const char* name, const char* on, const char* off) {
    xLabel l = tag(caption, source, name, 0, nullptr);
    l.on_ = on; l.off_ = off;
    return l;
}

xTagValue xLabel::value() {
    switch (source_) {
    case Source::Float: return xTagValue::num(*static_cast<const float*>(ptr_));
    case Source::Int: return xTagValue::num(*static_cast<const int32_t*>(ptr_));
    case Source::Bool: return xTagValue::boolean(*static_cast<const bool*>(ptr_));
    case Source::Getter: return get_ ? xTagValue::num(get_(ptr_)) : xTagValue::none();
    case Source::Tag: {
        if (handle_ < 0) handle_ = tags_->find(name_);
        xTagValue v;
        if (handle_ < 0 || !tags_->read(handle_, v)) return xTagValue::none();
        return v;
    }
    default: return xTagValue::none();
    }
}

static size_t copyText(const char* t, char* out, size_t cap) {
    const size_t n = strlen(t);
    if (n + 1 > cap) return kFormatTooLong;
    memcpy(out, t, n + 1);
    return n;
}

size_t xLabel::valueText(char* out, size_t cap) {
    if (out == nullptr || cap == 0) return kFormatTooLong;
    out[0] = 0;
    if (source_ == Source::None) return 0;
    const xTagValue v = value();
    switch (v.kind) {
    case xTagValue::Kind::None:
        return copyText("--", out, cap);
    case xTagValue::Kind::Bool:
        if (on_ || off_) return copyText(v.b ? (on_ ? on_ : "") : (off_ ? off_ : ""), out, cap);
        return copyText(v.b ? "1" : "0", out, cap);
    case xTagValue::Kind::Number:
        if (on_ || off_)        // a tag shown as on/off text
            return isnan(v.number) ? copyText("--", out, cap)
                                   : copyText(v.number != 0 ? (on_ ? on_ : "") : (off_ ? off_ : ""), out, cap);
        {
            const size_t n = formatNumber(v.number, decimals_, out, cap);
            return n == 0 ? copyText("--", out, cap) : n;      // NaN, inf
        }
    }
    return 0;
}

void xLabel::draw(iTextSurface& s, uint8_t col, uint8_t row, uint8_t width) {
    char buf[24];
    size_t n = valueText(buf, sizeof buf);
    const size_t unit = unit_ && source_ != Source::None ? strlen(unit_) : 0;
    const size_t room = unit < width ? width - unit : 0;
    if (n == kFormatTooLong || n > room) {
        n = room < sizeof buf ? room : sizeof buf - 1;
        memset(buf, '#', n);
        buf[n] = 0;
    }
    s.setCursor(col, row);
    // The caption, cut to leave the value and a space before it.
    const size_t capRoom = room > n ? room - n - (n ? 1 : 0) : 0;
    size_t used = 0;
    for (const char* c = caption_; c && *c && used < capRoom; ++c, ++used) s.putChar(*c);
    while (used + n < room) { s.putChar(' '); ++used; }
    s.print(buf);
    for (size_t i = 0; i < unit && used + n + i < width; ++i) s.putChar(unit_[i]);
}

void xCell::scroll(int by) {
    const int last = count_ > labelRows() ? count_ - labelRows() : 0;
    int f = (int)first_ + by;
    first_ = (uint8_t)(f < 0 ? 0 : f > last ? last : f);
}

void xCell::render(iTextSurface& s) {
    uint8_t r = row_;
    if (title_ && height_ > 0) {
        s.setCursor(col_, r);
        s.setInverse(true);
        uint8_t used = 0;
        for (const char* c = title_; *c && used < width_; ++c, ++used) s.putChar(*c);
        for (; used < width_; ++used) s.putChar(' ');
        s.setInverse(false);
        ++r;
    }
    if (first_ > 0) scroll(0);           // the labels may have changed
    for (uint8_t i = 0; i < labelRows() && first_ + i < count_; ++i)
        labels_[first_ + i].draw(s, col_, (uint8_t)(r + i), width_);
}

void xCellScreen::onKey(xKey key, xKeyAction action, xNavigator& nav) {
    if (action != xKeyAction::Pressed && action != xKeyAction::Repeat) return;
    if (key == xKey::Back) { if (action == xKeyAction::Pressed) nav.pop(); return; }
    if (key != xKey::Up && key != xKey::Down) return;
    for (uint8_t i = 0; i < count_; ++i) cells_[i]->scroll(key == xKey::Up ? -1 : 1);   // a cell that fits stays put
}

void xCellScreen::render(iTextSurface& s) {
    for (uint8_t i = 0; i < count_; ++i) cells_[i]->render(s);
}

} // namespace idisplay
