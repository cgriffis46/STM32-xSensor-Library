#include "xHmi.h"
#include <math.h>
#include <string.h>
#include "xTime.h"
#include "xFormat.h"

namespace idisplay {

xHmi::xHmi(const xHmiLayout& layout, iTagSource& source) : layout_(layout), source_(source) {
    for (uint16_t i = 0; i < kMaxTags; ++i) handles_[i] = -1;
}

bool xHmi::setPin(xHmiLevel level, const char* digits) {
    const uint8_t l = (uint8_t)level;
    if (l == 0 || l > 2) return false;
    pinSet_[l] = false;
    if (digits == nullptr) return false;
    for (uint8_t i = 0; i < kPinDigits; ++i) {
        if (digits[i] < '0' || digits[i] > '9') return false;
        pins_[l][i] = digits[i];
    }
    if (digits[kPinDigits] != 0) return false;
    pinSet_[l] = true;
    return true;
}

bool xHmi::showScreen(uint8_t index) {
    if (index >= layout_.screenCount) return false;
    screen_ = index;
    mode_ = Mode::View;
    return true;
}

bool xHmi::showScreen(const char* id) {
    if (id == nullptr) return false;
    for (uint8_t i = 0; i < layout_.screenCount; ++i)
        if (strcmp(layout_.screens[i].id, id) == 0) return showScreen(i);
    return false;
}

void xHmi::rebind() {
    for (uint16_t i = 0; i < layout_.tagCount && i < kMaxTags; ++i) handles_[i] = source_.find(layout_.tags[i]);
}

uint16_t xHmi::missingCount() const {
    uint16_t n = 0;
    for (uint16_t i = 0; i < layout_.tagCount; ++i)
        if (i >= kMaxTags || handles_[i] < 0) ++n;
    return n;
}

const char* xHmi::missing(uint16_t which) const {
    for (uint16_t i = 0; i < layout_.tagCount; ++i)
        if ((i >= kMaxTags || handles_[i] < 0) && which-- == 0) return layout_.tags[i];
    return nullptr;
}

void xHmi::onEnter(xNavigator& nav) {
    (void)nav;
    screen_ = 0;
    mode_ = Mode::View;
    unlocked_ = xHmiLevel::None;
    message_ = nullptr;
    messageLeft_ = 0;
    rebind();
}

bool xHmi::bound(uint16_t tag) {
    if (tag >= layout_.tagCount || tag >= kMaxTags) return false;
    if (handles_[tag] < 0) handles_[tag] = source_.find(layout_.tags[tag]);
    return handles_[tag] >= 0;
}

bool xHmi::readTag(uint16_t tag, xTagValue& v) {
    v = xTagValue::none();
    return bound(tag) && source_.read(handles_[tag], v) && v.kind != xTagValue::Kind::None;
}

// xFormat.h's, here so that a build compiling xHmi.cpp needs no new file.
size_t formatNumber(double v, uint8_t decimals, char* out, size_t cap) {
    if (cap == 0) return kFormatTooLong;
    out[0] = 0;
    if (!isfinite(v)) {
        if (cap < 3) return kFormatTooLong;
        out[0] = '-'; out[1] = '-'; out[2] = 0;
        return 0;
    }
    if (decimals > 6) decimals = 6;
    double p = 1;
    for (uint8_t i = 0; i < decimals; ++i) p *= 10;
    const double a = fabs(v) * p;
    if (a >= 1e15) return kFormatTooLong;
    const uint64_t u = (uint64_t)floor(a + 0.5);    // half away from zero; below 2^53, so exact

    char digits[24];
    size_t n = 0;
    uint64_t r = u;
    do { digits[n++] = (char)('0' + r % 10); r /= 10; } while (r != 0);
    while (n < (size_t)decimals + 1) digits[n++] = '0';     // 0.05, not .05

    const bool neg = v < 0 && u != 0;                       // no "-0.0"
    const size_t len = n + (neg ? 1 : 0) + (decimals ? 1 : 0);
    if (len + 1 > cap) return kFormatTooLong;
    size_t k = 0;
    if (neg) out[k++] = '-';
    while (n > 0) {
        if (decimals && n == decimals) out[k++] = '.';
        out[k++] = digits[--n];
    }
    out[k] = 0;
    return k;
}

size_t xHmi::formatNumber(double v, uint8_t decimals, char* out, size_t cap) {
    return idisplay::formatNumber(v, decimals, out, cap);
}

uint32_t xHmi::ticks(uint32_t ms) const {
    const uint32_t r = layout_.refreshMs ? layout_.refreshMs : 1;
    const uint32_t t = ms / r + (ms % r ? 1 : 0);
    return t ? t : 1;
}

uint16_t xHmi::fieldCount() const {
    if (screen_ >= layout_.screenCount) return 0;
    const xHmiScreen& sc = layout_.screens[screen_];
    uint16_t n = 0;
    for (uint16_t i = 0; i < sc.itemCount; ++i) if (sc.items[i].edit) ++n;
    return n;
}

const xHmiItem* xHmi::field(uint16_t which) const {
    if (screen_ >= layout_.screenCount) return nullptr;
    const xHmiScreen& sc = layout_.screens[screen_];
    for (uint16_t i = 0; i < sc.itemCount; ++i)
        if (sc.items[i].edit && which-- == 0) return &sc.items[i];
    return nullptr;
}

void xHmi::say(const char* text) {
    message_ = text;
    messageLeft_ = ticks(kMessageMs);
}

static int32_t lastStep(const xHmiItem& it) {
    const xHmiEdit& e = *it.edit;
    return (int32_t)floor((e.max - e.min) / e.step + 1e-9);
}

static bool isOn(const xTagValue& v) {
    return v.kind == xTagValue::Kind::Bool ? v.b : (v.number != 0 && !isnan(v.number));
}

static double asNumber(const xTagValue& v) {
    return v.kind == xTagValue::Kind::Bool ? (v.b ? 1 : 0) : v.number;
}

xTagValue xHmi::pendingValue() const {
    const xHmiItem* it = field(pick_);
    if (it == nullptr) return xTagValue::none();
    switch (it->kind) {
    case xHmiKind::State:
        return xTagValue::boolean(pending_ != 0);
    case xHmiKind::Choice:
        return pending_ >= 0 && pending_ < it->choiceCount ? xTagValue::num(it->choices[pending_].value) : xTagValue::none();
    default: {
        const xHmiEdit& e = *it->edit;
        double v = e.min + (double)pending_ * e.step;
        double p = 1;
        for (uint8_t i = 0; i < it->decimals && i < 6; ++i) p *= 10;
        const double r = floor(fabs(v) * p + 0.5) / p;    // what the field shows
        v = v < 0 ? -r : r;
        if (v < e.min) v = e.min;
        if (v > e.max) v = e.max;
        return xTagValue::num(v);
    }
    }
}

void xHmi::startEdit() {
    const xHmiItem* it = field(pick_);
    if (it == nullptr) return;
    const xHmiLevel need = it->edit->level;
    if (need != xHmiLevel::None && (uint8_t)unlocked_ < (uint8_t)need) {
        bool any = false;
        for (uint8_t l = (uint8_t)need; l <= 2; ++l) any = any || pinSet_[l];
        if (lockout_ > 0 || !any) { say("Locked"); return; }
        need_ = need;
        for (uint8_t i = 0; i < kPinDigits; ++i) digits_[i] = 0;
        digit_ = 0;
        mode_ = Mode::Pin;
        return;
    }
    xTagValue v;
    if (!readTag(it->tag, v) || (v.kind == xTagValue::Kind::Number && !isfinite(v.number) && it->kind != xHmiKind::State)) {
        say("No value");
        return;
    }
    switch (it->kind) {
    case xHmiKind::State:
        original_ = isOn(v) ? 1 : 0;
        break;
    case xHmiKind::Choice:
        original_ = -1;                     // a value that is none of the choices
        for (uint8_t i = 0; i < it->choiceCount; ++i)
            if (it->choices[i].value == asNumber(v)) { original_ = i; break; }
        break;
    default: {
        const xHmiEdit& e = *it->edit;
        double c = asNumber(v);
        if (c < e.min) c = e.min;
        if (c > e.max) c = e.max;
        int32_t k = (int32_t)floor((c - e.min) / e.step + 0.5);
        const int32_t last = lastStep(*it);
        original_ = k < 0 ? 0 : k > last ? last : k;
        break;
    }
    }
    pending_ = original_ < 0 ? 0 : original_;
    mode_ = Mode::Edit;
}

void xHmi::step(int32_t by) {
    const xHmiItem* it = field(pick_);
    if (it == nullptr) return;
    int32_t last;
    switch (it->kind) {
    case xHmiKind::State: pending_ = pending_ ? 0 : 1; return;
    case xHmiKind::Choice: last = (int32_t)it->choiceCount - 1; break;
    default: last = lastStep(*it); break;
    }
    int32_t n = pending_ + by;
    pending_ = n < 0 ? 0 : n > last ? last : n;
}

void xHmi::finishEdit() {
    if (pending_ == original_) { mode_ = Mode::Pick; return; }
    yes_ = false;
    mode_ = Mode::Confirm;
}

xHmiLevel xHmi::checkPin(const char* digits) const {
    if (digits == nullptr) return xHmiLevel::None;
    for (uint8_t i = 0; i < kPinDigits; ++i) if (digits[i] < '0' || digits[i] > '9') return xHmiLevel::None;
    if (digits[kPinDigits] != 0) return xHmiLevel::None;
    for (uint8_t l = 2; l >= 1; --l)
        if (pinSet_[l] && memcmp(pins_[l], digits, kPinDigits) == 0) return (xHmiLevel)l;
    return xHmiLevel::None;
}

bool xHmi::pinSet(xHmiLevel level) const {
    const uint8_t l = (uint8_t)level;
    return l >= 1 && l <= 2 && pinSet_[l];
}

bool xHmi::fieldAccepts(const xHmiItem& it, double x, xTagValue& out) {
    if (it.edit == nullptr || !isfinite(x)) return false;
    switch (it.kind) {
    case xHmiKind::State:
        if (x != 0 && x != 1) return false;
        out = xTagValue::boolean(x != 0);
        return true;
    case xHmiKind::Choice:
        for (uint8_t i = 0; i < it.choiceCount; ++i)
            if (it.choices[i].value == x) { out = xTagValue::num(x); return true; }
        return false;
    case xHmiKind::Value: {
        const xHmiEdit& e = *it.edit;
        if (x < e.min || x > e.max) return false;
        const double k = (x - e.min) / e.step;
        if (fabs(k - floor(k + 0.5)) > 1e-6) return false;
        double p = 1;
        for (uint8_t i = 0; i < it.decimals && i < 6; ++i) p *= 10;
        if (fabs(x * p - floor(x * p + 0.5)) > 1e-6 * (fabs(x * p) > 1 ? fabs(x * p) : 1)) return false;
        out = xTagValue::num(x);
        return true;
    }
    default:
        return false;
    }
}

void xHmi::enterDigit() {
    if (++digit_ < kPinDigits) return;
    xHmiLevel matched = xHmiLevel::None;
    for (uint8_t l = 2; l >= 1 && matched == xHmiLevel::None; --l) {
        if (!pinSet_[l]) continue;
        bool same = true;
        for (uint8_t i = 0; i < kPinDigits; ++i) same = same && pins_[l][i] == (char)('0' + digits_[i]);
        if (same) matched = (xHmiLevel)l;
    }
    mode_ = Mode::Pick;
    if (matched != xHmiLevel::None && (uint8_t)matched >= (uint8_t)need_) {
        unlocked_ = matched;
        wrong_ = 0;
        startEdit();
        return;
    }
    if (++wrong_ >= kPinTries) {
        wrong_ = 0;
        lockout_ = ticks(kLockoutMs);
        say("Locked");
    } else {
        say("Wrong PIN");
    }
}

void xHmi::onKey(xKey key, xKeyAction action, xNavigator& nav) {
    (void)nav;
    idle_ = 0;
    if (action != xKeyAction::Pressed && action != xKeyAction::Repeat) return;
    const bool press = action == xKeyAction::Pressed;
    repeats_ = press ? 0 : (uint8_t)(repeats_ < 255 ? repeats_ + 1 : 255);
    if (message_) {                         // a key only clears the message
        if (press) { message_ = nullptr; messageLeft_ = 0; }
        return;
    }
    if ((uint8_t)key >= kHmiKeys || screen_ >= layout_.screenCount) return;
    switch (mode_) {
    case Mode::View:
        if (!press) return;
        if (key == xKey::Enter && fieldCount() > 0) { pick_ = 0; mode_ = Mode::Pick; return; }
        {
            const uint8_t to = layout_.screens[screen_].keys[(uint8_t)key];
            if (to != kHmiNoScreen) showScreen(to);
        }
        return;
    case Mode::Pick:
        if (!press) return;
        if (key == xKey::Up && pick_ > 0) --pick_;
        else if (key == xKey::Down && pick_ + 1 < fieldCount()) ++pick_;
        else if (key == xKey::Back) mode_ = Mode::View;
        else if (key == xKey::Enter) startEdit();
        return;
    case Mode::Edit: {
        const xHmiItem* it = field(pick_);
        if (key == xKey::Up || key == xKey::Down) {
            if (it == nullptr || (it->kind == xHmiKind::State && !press)) return;
            int32_t by = it->kind == xHmiKind::Value && repeats_ >= 10 ? 10 : 1;
            const bool up = key == xKey::Up;
            if (it->kind == xHmiKind::Choice) by = up ? -1 : 1;    // Up: the one above in the list
            else by = up ? by : -by;
            step(by);
        } else if (press && key == xKey::Enter) {
            finishEdit();
        } else if (press && key == xKey::Back) {
            mode_ = Mode::Pick;
        }
        return;
    }
    case Mode::Confirm:
        if (!press) return;
        if (key == xKey::Up || key == xKey::Down) yes_ = !yes_;
        else if (key == xKey::Back) mode_ = Mode::Edit;
        else if (key == xKey::Enter) {
            if (!yes_) { mode_ = Mode::Edit; return; }
            const xHmiItem* it = field(pick_);
            if (it == nullptr) { mode_ = Mode::Pick; return; }
            xTagValue before;
            if (!readTag(it->tag, before)) before = xTagValue::none();
            const xTagValue after = pendingValue();
            const bool ok = bound(it->tag) && source_.write(handles_[it->tag], after);
            if (audit_) audit_(layout_.tags[it->tag], before, after, ok, unlocked_, auditCtx_);
            mode_ = Mode::Pick;
            say(ok ? "Written" : "Refused");
        }
        return;
    case Mode::Pin:
        if (key == xKey::Up) digits_[digit_] = (uint8_t)((digits_[digit_] + 1) % 10);
        else if (key == xKey::Down) digits_[digit_] = (uint8_t)((digits_[digit_] + 9) % 10);
        else if (press && key == xKey::Enter) enterDigit();
        else if (press && key == xKey::Back) {
            if (digit_ > 0) --digit_;
            else mode_ = Mode::Pick;
        }
        return;
    }
}

void xHmi::onRefresh(uint32_t arg) {
    (void)arg;
    if (idle_ < 0xFFFFFFFFu) ++idle_;
    if (lockout_ > 0) --lockout_;
    if (messageLeft_ > 0 && --messageLeft_ == 0) message_ = nullptr;
    if (mode_ != Mode::View && idle_ >= ticks(layout_.editTimeoutMs)) mode_ = Mode::View;
    if (unlocked_ != xHmiLevel::None && idle_ >= ticks(layout_.unlockTimeoutMs)) unlocked_ = xHmiLevel::None;
}

static void fill(iTextSurface& s, char c, uint8_t n) {
    for (uint8_t i = 0; i < n; ++i) s.putChar(c);
}

size_t xHmi::itemText(const xHmiItem& it, bool known, const xTagValue& v, char* out, size_t cap) {
    size_t n = 0;
    const char* text = "--";
    if (!known || v.kind == xTagValue::Kind::None) {
        // "--"
    } else if (it.kind == xHmiKind::Value) {
        if (v.kind == xTagValue::Kind::Bool) text = v.b ? "1" : "0";
        else {
            n = formatNumber(v.number, it.decimals, out, cap);
            text = n == kTooLong ? nullptr : out;           // NaN: out holds "--"
        }
    } else if (it.kind == xHmiKind::State) {
        text = isOn(v) ? it.on : it.off;
        if (text == nullptr) text = "";
    } else if (it.kind == xHmiKind::Choice) {
        for (uint8_t i = 0; i < it.choiceCount; ++i)
            if (it.choices[i].value == asNumber(v)) { text = it.choices[i].label; break; }
    }
    if (it.kind == xHmiKind::Value) {
        // Too long for its width (or for out): '#' in every cell.
        const size_t len = text ? strlen(text) : (size_t)-1;
        if (len > it.width || len + 1 > cap) {
            n = it.width < cap ? it.width : cap - 1;
            for (size_t i = 0; i < n; ++i) out[i] = '#';
            out[n] = 0;
            return n;
        }
    }
    n = 0;
    while (text[n] && n + 1 < cap && (it.kind == xHmiKind::Value || n < it.width)) { out[n] = text[n]; ++n; }
    out[n] = 0;
    return n;
}

void xHmi::drawItem(iTextSurface& s, const xHmiItem& it, bool picked) {
    s.setCursor(it.col, it.row);
    switch (it.kind) {
    case xHmiKind::Text:
        s.print(it.text);
        return;
    case xHmiKind::Time: {
        xTimeHHMMSS hms(it.col, it.row);
        xTimeHHMM hm(it.col, it.row);
        xTimeHHMM& t = it.seconds ? hms : hm;
        xTagValue v;
        if (readTag(it.tag, v) && v.kind == xTagValue::Kind::Number && isfinite(v.number) && v.number >= 0)
            t.setFromSecondsOfDay((uint32_t)fmod(floor(v.number), 86400.0));
        t.update(s);
        return;
    }
    default:
        break;
    }
    xTagValue v;
    bool known;
    if (picked && mode_ == Mode::Edit) {
        v = pendingValue();
        known = v.kind != xTagValue::Kind::None;
    } else {
        known = readTag(it.tag, v);
    }
    char buf[24];
    const size_t n = itemText(it, known, v, buf, sizeof buf);
    const uint8_t pad = n < it.width ? (uint8_t)(it.width - n) : 0;
    const bool right = it.kind == xHmiKind::Value && it.align == xHmiAlign::Right;
    s.setInverse(picked);
    if (right) fill(s, ' ', pad);
    s.print(buf);
    if (!right) fill(s, ' ', pad);
    s.setInverse(false);
    if (it.kind == xHmiKind::Value && it.unit) s.print(it.unit);
}

void xHmi::drawStatus(iTextSurface& s, const char* a, const char* b, bool inverse) {
    s.setCursor(0, (uint8_t)(s.rows() - 1));
    s.setInverse(inverse);
    s.print(a);
    s.print(b);
    s.padToEndOfRow();
    s.setInverse(false);
}

static const char* levelName(xHmiLevel l) {
    return l == xHmiLevel::Engineer ? "engineer" : "operator";
}

void xHmi::drawConfirm(iTextSurface& s) {
    const xHmiItem* it = field(pick_);
    if (it == nullptr) return;
    const uint8_t rows = s.rows();
    const char* unit = it->kind == xHmiKind::Value && it->unit ? it->unit : "";
    char buf[24];
    uint8_t r = 0;
    if (rows >= 3) { s.setCursor(0, 0); s.print(it->edit->label); r = 1; }
    const xTagValue next = pendingValue();
    itemText(*it, true, next, buf, sizeof buf);
    s.setCursor(0, r);
    s.print("Write "); s.print(buf); s.print(unit); s.print("?");
    if (rows >= 4) {
        xTagValue now;
        const bool known = readTag(it->tag, now);
        itemText(*it, known, now, buf, sizeof buf);
        s.setCursor(0, 2);
        s.print("was "); s.print(buf); s.print(unit);
    }
    const uint8_t last = (uint8_t)(rows - 1);
    // ">No  Yes" / " No >Yes": the marker shows on a character LCD,
    // which can't show inverse; an OLED shows both.
    s.setCursor(0, last);
    s.putChar(yes_ ? ' ' : '>');
    s.setInverse(!yes_); s.print("No"); s.setInverse(false);
    s.print(" ");
    s.putChar(yes_ ? '>' : ' ');
    s.setInverse(yes_); s.print("Yes"); s.setInverse(false);
    s.showEditCursor(yes_ ? 5 : 1, last);
}

void xHmi::drawPin(iTextSurface& s) {
    s.setCursor(0, 0);
    s.print("PIN, "); s.print(levelName(need_));
    s.setCursor(0, 1);
    for (uint8_t i = 0; i < kPinDigits; ++i) {
        if (i < digit_) s.putChar('*');
        else if (i == digit_) { s.setInverse(true); s.putChar((char)('0' + digits_[i])); s.setInverse(false); }
        else s.putChar('_');
    }
    s.showEditCursor(digit_, 1);
}

void xHmi::render(iTextSurface& s) {
    if (screen_ >= layout_.screenCount) return;
    if (mode_ == Mode::Confirm) drawConfirm(s);
    else if (mode_ == Mode::Pin) drawPin(s);
    else {
        const xHmiScreen& sc = layout_.screens[screen_];
        const xHmiItem* picked = mode_ == Mode::Pick || mode_ == Mode::Edit ? field(pick_) : nullptr;
        for (uint16_t i = 0; i < sc.itemCount; ++i) drawItem(s, sc.items[i], &sc.items[i] == picked);
        if (picked) {
            drawStatus(s, mode_ == Mode::Edit ? "Edit: " : "", picked->edit->label, false);
            s.showEditCursor((uint8_t)(picked->col + (mode_ == Mode::Edit && picked->width ? picked->width - 1 : 0)), picked->row);
        }
    }
    if (message_) drawStatus(s, message_, "", true);
}

} // namespace idisplay
