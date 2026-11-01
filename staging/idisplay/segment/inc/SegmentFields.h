#pragma once
#include <stdint.h>
#include <math.h>
#include "iTagSource.h"
#include "SegmentFrame.h"
#include "SevenSeg.h"

namespace idisplay {

// A tag as a field uses it: the name and, once found, the handle. A tag
// that isn't there yet is looked for again on each render, so tags
// registered after the display starts are picked up (as xHmi does).
class TagRef {
public:
    explicit TagRef(const char* name = nullptr) : name_(name) {}
    bool used() const { return name_ != nullptr; }
    const char* name() const { return name_; }
    // False: no such tag, or it can't be read now.
    bool read(iTagSource& src, xTagValue& out) {
        if (!name_) return false;
        if (h_ < 0) h_ = src.find(name_);
        if (h_ < 0) return false;
        return src.read(h_, out) && out.kind != xTagValue::Kind::None;
    }
    bool number(iTagSource& src, double& out) {
        xTagValue v;
        if (!read(src, v)) return false;
        out = v.kind == xTagValue::Kind::Bool ? (v.b ? 1.0 : 0.0) : v.number;
        return true;
    }
    bool truthy(iTagSource& src) {
        double v;
        return number(src, v) && v != 0.0 && !isnan(v);
    }
private:
    const char* name_;
    int32_t h_ = -1;
};

// Something on the glass that shows tags.
class iSegmentField {
public:
    virtual ~iSegmentField() = default;
    virtual void render(iTagSource& src, SegmentFrame& fr) = 0;
};

// One segment: an icon, a unit, a label. On while its tag is true
// (non-zero), while it equals a value or lies in lo..hi (an enum tag: the
// unit, the function, a range), or always (no tag: a fixed label). Off if
// the tag can't be read.
class IconField : public iSegmentField {
public:
    enum class When : uint8_t { True, Equals, Always };

    IconField(SegRef seg, const char* tag) : seg_(seg), tag_(tag), when_(When::True) {}
    IconField(SegRef seg, const char* tag, double equals) : seg_(seg), tag_(tag), when_(When::Equals), k_(equals), hi_(equals) {}
    IconField(SegRef seg, const char* tag, double lo, double hi) : seg_(seg), tag_(tag), when_(When::Equals), k_(lo), hi_(hi) {}
    static IconField always(SegRef seg) { IconField f(seg, nullptr); f.when_ = When::Always; return f; }

    void render(iTagSource& src, SegmentFrame& fr) override {
        bool on = false;
        if (when_ == When::Always) on = true;
        else if (when_ == When::True) on = tag_.truthy(src);
        else { double v; on = tag_.number(src, v) && v >= k_ && v <= hi_; }
        fr.set(seg_, on);
    }

private:
    SegRef seg_;
    TagRef tag_;
    When   when_;
    double k_ = 0, hi_ = 0;
};

// A number on 7-segment digits: right aligned, rounded half away from
// zero to its decimals, a leading zero before the point, a minus segment.
//
//   value     the number
//   decimals  a tag giving the decimals (or the fixed number given)
//   overload  a tag: true shows olText ("  OL " by default)
//
// Too many digits: olText too. Unknown tag, failed read, NaN: dashes.
//
// points[i] is the point between digit i and digit i + 1 (digits left to
// right); a glass without one there gives {0xFF, 0xFF}.
class NumberField : public iSegmentField {
public:
    static constexpr uint8_t kMaxDigits = 8;

    NumberField(const SevenSegDigit* digits, uint8_t n, const SegRef* points, SegRef minus,
                const char* valueTag, const char* decimalsTag, uint8_t fixedDecimals = 0,
                const char* overloadTag = nullptr)
        : digits_(digits), n_(n > kMaxDigits ? kMaxDigits : n), points_(points), minus_(minus),
          value_(valueTag), decimals_(decimalsTag), fixed_(fixedDecimals), overload_(overloadTag) {}

    void setOverloadText(const char* t) { olText_ = t; }

    void render(iTagSource& src, SegmentFrame& fr) override {
        char text[kMaxDigits + 1];
        int8_t point = -1;
        bool negative = false;
        double v;
        uint8_t dec = fixed_;
        double d;
        if (decimals_.used() && decimals_.number(src, d) && d >= 0 && d < n_) dec = (uint8_t)d;

        if (overload_.used() && overload_.truthy(src)) {
            fill(text, olText_);
        } else if (!value_.number(src, v) || isnan(v) || isinf(v)) {
            for (uint8_t i = 0; i < n_; i++) text[i] = '-';
            text[n_] = 0;
        } else {
            double scaled = fabs(v);
            for (uint8_t i = 0; i < dec; i++) scaled *= 10.0;
            scaled = floor(scaled + 0.5);
            double limit = 1.0;
            for (uint8_t i = 0; i < n_; i++) limit *= 10.0;
            if (scaled >= limit) {
                fill(text, olText_);
            } else {
                uint32_t x = (uint32_t)scaled;
                for (int8_t i = (int8_t)n_ - 1; i >= 0; i--) {
                    const bool needed = x != 0 || (int8_t)(n_ - 1 - i) <= dec;
                    text[i] = needed ? (char)('0' + x % 10) : ' ';
                    x /= 10;
                }
                text[n_] = 0;
                if (dec > 0) point = (int8_t)(n_ - 1 - dec);
                negative = v < 0.0 && scaled != 0.0;
            }
        }
        for (uint8_t i = 0; i < n_; i++) seven::draw(fr, digits_[i], seven::glyph(text[i]));
        if (points_)
            for (uint8_t i = 0; i + 1 < n_; i++) fr.set(points_[i], (int8_t)i == point);
        fr.set(minus_, negative);
    }

private:
    void fill(char* text, const char* s) const {
        uint8_t i = 0;
        for (; i < n_ && s && s[i]; i++) text[i] = s[i];
        for (; i < n_; i++) text[i] = ' ';
        text[n_] = 0;
    }

    const SevenSegDigit* digits_;
    uint8_t n_;
    const SegRef* points_;
    SegRef minus_;
    TagRef value_, decimals_;
    uint8_t fixed_;
    TagRef overload_;
    const char* olText_ = "  OL ";
};

// Words chosen by a tag's value, on 7-segment digits: a setup menu, a
// status ("OPEn", "Auto"), a mode name.
class ChoiceField : public iSegmentField {
public:
    struct Choice { double value; const char* text; };

    ChoiceField(const SevenSegDigit* digits, uint8_t n, const char* tag, const Choice* choices, uint8_t count,
                const char* otherwise = "")
        : digits_(digits), n_(n), tag_(tag), choices_(choices), count_(count), otherwise_(otherwise) {}

    void render(iTagSource& src, SegmentFrame& fr) override {
        const char* t = otherwise_;
        double v;
        if (tag_.number(src, v))
            for (uint8_t i = 0; i < count_; i++) if (choices_[i].value == v) { t = choices_[i].text; break; }
        bool ended = false;
        for (uint8_t i = 0; i < n_; i++) {
            if (!t || !t[i]) ended = true;
            seven::draw(fr, digits_[i], ended ? 0 : seven::glyph(t[i]));
        }
    }

private:
    const SevenSegDigit* digits_;
    uint8_t n_;
    TagRef tag_;
    const Choice* choices_;
    uint8_t count_;
    const char* otherwise_;
};

// A bargraph: |value| / full of the bars lit, from the first, rounded
// down, all of them at or past full. The sign segments say which side.
// Unknown: none lit.
class BarField : public iSegmentField {
public:
    BarField(const SegRef* bars, uint8_t n, const char* valueTag, double full,
             SegRef positive = SegRef{0xFF, 0xFF}, SegRef negative = SegRef{0xFF, 0xFF}, const char* enableTag = nullptr)
        : bars_(bars), n_(n), value_(valueTag), full_(full), pos_(positive), neg_(negative), enable_(enableTag) {}

    void render(iTagSource& src, SegmentFrame& fr) override {
        double v = 0;
        const bool on = (!enable_.used() || enable_.truthy(src)) && value_.number(src, v) && !isnan(v) && full_ > 0;
        uint8_t lit = 0;
        if (on) {
            double k = fabs(v) / full_ * n_;
            lit = k >= n_ ? n_ : (uint8_t)k;
        }
        for (uint8_t i = 0; i < n_; i++) fr.set(bars_[i], i < lit);
        fr.set(pos_, on && v >= 0);
        fr.set(neg_, on && v < 0);
    }

private:
    const SegRef* bars_;
    uint8_t n_;
    TagRef value_;
    double full_;
    SegRef pos_, neg_;
    TagRef enable_;
};

} // namespace idisplay
