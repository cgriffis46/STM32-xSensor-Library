#include "xShell.h"
#include <math.h>
#include <string.h>
#include "xFormat.h"

namespace idisplay {

static const char* const kHelp[] = {
    "help               this list",
    "tags               the HMI's tags and what may be set",
    "get <tag>          a tag's value",
    "set <tag> <value>  an editable field (number, on/off, choice)",
    "pin <dddd>         unlock a level",
    "lock               lock again",
    "log [n]            the last n log lines",
};
static const uint16_t kHelpLines = sizeof kHelp / sizeof kHelp[0];

static const char* levelName(xHmiLevel l) {
    return l == xHmiLevel::Engineer ? "engineer" : l == xHmiLevel::Operator ? "operator" : "none";
}

static bool sameText(const char* a, const char* b) {      // ignoring case
    if (a == nullptr || b == nullptr) return false;
    for (; *a && *b; ++a, ++b) {
        char x = *a, y = *b;
        if (x >= 'A' && x <= 'Z') x = (char)(x + 32);
        if (y >= 'A' && y <= 'Z') y = (char)(y + 32);
        if (x != y) return false;
    }
    return *a == *b;
}

// A decimal number: an optional sign, digits, an optional point and
// digits. No exponent, nothing after it.
static bool parseNumber(const char* s, double& out) {
    if (s == nullptr) return false;
    bool neg = false;
    if (*s == '-' || *s == '+') neg = *s++ == '-';
    double v = 0, scale = 1;
    bool digits = false, point = false;
    for (; *s; ++s) {
        if (*s >= '0' && *s <= '9') {
            digits = true;
            if (point) { scale /= 10; v += (*s - '0') * scale; }
            else v = v * 10 + (*s - '0');
        } else if (*s == '.' && !point) {
            point = true;
        } else {
            return false;
        }
    }
    if (!digits) return false;
    out = neg ? -v : v;
    return true;
}

static void append(char* out, size_t cap, const char* s) {
    size_t n = strlen(out);
    while (s && *s && n + 1 < cap) out[n++] = *s++;
    out[n] = 0;
}

static void appendNumber(char* out, size_t cap, double v, uint8_t decimals) {
    char buf[24];
    const size_t n = formatNumber(v, decimals, buf, sizeof buf);
    append(out, cap, n == kFormatTooLong ? "#" : buf);
}

// The fewest decimals (up to 6) that show v exactly enough.
static uint8_t decimalsFor(double v) {
    double p = 1;
    for (uint8_t d = 0; d < 6; ++d, p *= 10)
        if (fabs(v * p - floor(v * p + 0.5)) < 1e-6) return d;
    return 6;
}

void xShell::begin() {
    io_.setRxSink(*this);
    emit("\r\n");
    if (name_) emit(name_);
    emitLine(name_ ? " shell: help for the commands" : "shell: help for the commands");
    prompt();
}

void xShell::onByteReceived(uint8_t b) {
    const uint16_t head = rxHead_.load(std::memory_order_relaxed);
    const uint16_t next = (uint16_t)((head + 1) % kRx);
    if (next == rxTail_.load(std::memory_order_acquire)) {
        rxDropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    rx_[head] = b;
    rxHead_.store(next, std::memory_order_release);
}

void xShell::poll(uint32_t nowMs) {
    if (lockout_ && nowMs - lockoutAt_ >= xHmi::kLockoutMs) lockout_ = false;
    if (unlocked_ != xHmiLevel::None && nowMs - lastCommand_ >= hmi_.layout().unlockTimeoutMs) {
        unlocked_ = xHmiLevel::None;
    }
    for (;;) {
        const uint16_t tail = rxTail_.load(std::memory_order_relaxed);
        if (tail == rxHead_.load(std::memory_order_acquire)) break;
        const uint8_t b = rx_[tail];
        rxTail_.store((uint16_t)((tail + 1) % kRx), std::memory_order_release);
        input(b, nowMs);
    }
    listMore();
    send();
}

void xShell::prompt() {
    emit(unlocked_ == xHmiLevel::Engineer ? "eng> " : unlocked_ == xHmiLevel::Operator ? "op> " : "> ");
}

void xShell::input(uint8_t b, uint32_t nowMs) {
    if (esc_ == 1) { esc_ = b == '[' || b == 'O' ? 2 : 0; return; }      // arrows and the like: ignored
    if (esc_ == 2) { if (b >= 0x40 && b <= 0x7E) esc_ = 0; return; }
    if (b == '\n' && lastCr_) { lastCr_ = false; return; }
    lastCr_ = b == '\r';
    if (b == '\r' || b == '\n') {
        emit("\r\n");
        line_[lineLen_] = 0;
        lineLen_ = 0;
        if (listing_ != Listing::None) { listing_ = Listing::None; emitLine("(stopped)"); }
        execute(line_, nowMs);
        if (listing_ == Listing::None) prompt();
        return;
    }
    if (b == 0x1B) { esc_ = 1; return; }
    if (b == 0x03) {                                    // Ctrl-C
        lineLen_ = 0;
        listing_ = Listing::None;
        emit("^C\r\n");
        prompt();
        return;
    }
    if (b == 0x08 || b == 0x7F) {
        if (lineLen_ > 0) { --lineLen_; emit("\b \b"); }
        return;
    }
    if (b < 0x20 || b > 0x7E || lineLen_ >= kLine) return;
    line_[lineLen_++] = (char)b;
    // A PIN is echoed as stars.
    const bool secret = lineLen_ > 4 && strncmp(line_, "pin ", 4) == 0;
    const char echo[2] = {secret ? '*' : (char)b, 0};
    emit(echo);
}

// The degree sign (kDegreeChar, in units) goes out as UTF-8.
void xShell::emit(const char* s) {
    size_t n = 0;
    for (const char* p = s; *p; ++p) n += *p == kDegreeChar ? 2 : 1;
    if (n > room()) { ++dropped_; return; }
    for (; *s; ++s) {
        if (*s == kDegreeChar) {
            out_[(outHead_ + outCount_++) % kOut] = '\xC2';
            out_[(outHead_ + outCount_++) % kOut] = '\xB0';
        } else {
            out_[(outHead_ + outCount_++) % kOut] = *s;
        }
    }
}

void xShell::emitLine(const char* a, const char* b, const char* c, const char* d) {
    char line[160] = "";
    append(line, sizeof line, a);
    append(line, sizeof line, b);
    append(line, sizeof line, c);
    append(line, sizeof line, d);
    append(line, sizeof line, "\r\n");
    emit(line);
}

void xShell::send() {
    for (;;) {
        if (txLen_ == 0) {
            while (txLen_ < kTx && outCount_ > 0) {
                tx_[txNext_][txLen_++] = (uint8_t)out_[outHead_];
                outHead_ = (outHead_ + 1) % kOut;
                --outCount_;
            }
            if (txLen_ == 0) return;
        }
        if (!io_.write(tx_[txNext_], txLen_)) return;     // the other buffer is still going out
        txNext_ ^= 1;
        txLen_ = 0;
    }
}

// Writes the next lines of a listing while there is room for them.
void xShell::listMore() {
    while (listing_ != Listing::None && room() >= 160 + 8) {
        if (listAt_ >= listEnd_) {
            listing_ = Listing::None;
            prompt();
            return;
        }
        const uint16_t i = listAt_++;
        char line[160];
        switch (listing_) {
        case Listing::Help:
            emitLine(kHelp[i]);
            break;
        case Listing::Tags:
            if (tagLine(i, line, sizeof line)) emitLine(line);
            break;
        case Listing::Log:
            if (log_) {
                char text[256];
                log_->copy((uint16_t)(listEnd_ - 1 - i), text, sizeof text);
                text[150] = 0;
                emitLine(text);
            }
            break;
        default:
            break;
        }
    }
}

int32_t xShell::tagIndex(const char* name) const {
    const xHmiLayout& l = hmi_.layout();
    for (uint16_t i = 0; i < l.tagCount; ++i)
        if (strcmp(l.tags[i], name) == 0) return i;
    return -1;
}

static const xHmiItem* findItem(const xHmiLayout& l, uint16_t tag, bool editable) {
    for (uint8_t s = 0; s < l.screenCount; ++s)
        for (uint16_t i = 0; i < l.screens[s].itemCount; ++i) {
            const xHmiItem& it = l.screens[s].items[i];
            if (it.tag != tag) continue;
            if (it.kind != xHmiKind::Value && it.kind != xHmiKind::State && it.kind != xHmiKind::Choice) continue;
            if (!editable || it.edit) return &it;
        }
    return nullptr;
}

const xHmiItem* xShell::shownBy(uint16_t tag) const {
    const xHmiItem* f = findItem(hmi_.layout(), tag, true);
    return f ? f : findItem(hmi_.layout(), tag, false);
}

const xHmiItem* xShell::field(uint16_t tag) const { return findItem(hmi_.layout(), tag, true); }

// "name = value unit", and for an editable field what it may be set to.
bool xShell::tagLine(uint16_t tag, char* out, size_t cap) {
    const xHmiLayout& l = hmi_.layout();
    if (tag >= l.tagCount) return false;
    out[0] = 0;
    append(out, cap, l.tags[tag]);
    append(out, cap, " = ");
    iTagSource& src = hmi_.source();
    const int32_t h = src.find(l.tags[tag]);
    xTagValue v;
    const bool known = h >= 0 && src.read(h, v) && v.kind != xTagValue::Kind::None;
    const xHmiItem* it = shownBy(tag);
    char buf[24];
    if (it) {
        xHmiItem wide = *it;
        wide.width = 20;                    // the whole value, not cut to the panel's field
        xHmi::itemText(wide, known, v, buf, sizeof buf);
        append(out, cap, buf);
        if (it->kind == xHmiKind::Value && it->unit) append(out, cap, it->unit);
    } else if (!known) {
        append(out, cap, "--");
    } else if (v.kind == xTagValue::Kind::Bool) {
        append(out, cap, v.b ? "1" : "0");
    } else {
        appendNumber(out, cap, v.number, decimalsFor(v.number));
    }
    const xHmiItem* f = it && it->edit ? it : nullptr;
    if (f == nullptr) return true;
    append(out, cap, "  [");
    if (f->kind == xHmiKind::Value) {
        const uint8_t d = f->decimals;
        appendNumber(out, cap, f->edit->min, d);
        append(out, cap, "..");
        appendNumber(out, cap, f->edit->max, d);
        append(out, cap, " step ");
        appendNumber(out, cap, f->edit->step, d);
    } else if (f->kind == xHmiKind::State) {
        append(out, cap, f->on ? f->on : "1");
        append(out, cap, "|");
        append(out, cap, f->off ? f->off : "0");
    } else {
        for (uint8_t i = 0; i < f->choiceCount; ++i) {
            if (i) append(out, cap, "|");
            append(out, cap, f->choices[i].label);
        }
    }
    if (f->edit->level != xHmiLevel::None) { append(out, cap, ", "); append(out, cap, levelName(f->edit->level)); }
    append(out, cap, "]");
    return true;
}

void xShell::execute(char* line, uint32_t nowMs) {
    lastCommand_ = nowMs;
    char* word[3] = {nullptr, nullptr, nullptr};
    uint8_t words = 0;
    for (char* p = line; *p;) {
        while (*p == ' ' || *p == '\t') *p++ = 0;
        if (!*p) break;
        if (words == 3) { emitLine("too many words"); return; }
        word[words++] = p;
        while (*p && *p != ' ' && *p != '\t') ++p;
    }
    if (words == 0) return;
    const char* cmd = word[0];
    if (strcmp(cmd, "help") == 0) {
        listing_ = Listing::Help; listAt_ = 0; listEnd_ = kHelpLines;
    } else if (strcmp(cmd, "tags") == 0) {
        listing_ = Listing::Tags; listAt_ = 0; listEnd_ = hmi_.layout().tagCount;
    } else if (strcmp(cmd, "get") == 0 && words == 2) {
        get(word[1]);
    } else if (strcmp(cmd, "set") == 0 && words == 3) {
        set(word[1], word[2], nowMs);
    } else if (strcmp(cmd, "pin") == 0 && words == 2) {
        pin(word[1], nowMs);
    } else if (strcmp(cmd, "lock") == 0 && words == 1) {
        unlocked_ = xHmiLevel::None;
        emitLine("locked");
    } else if (strcmp(cmd, "log") == 0 && words <= 2) {
        log(word[1]);
    } else {
        emitLine("unknown command (help)");
    }
}

void xShell::get(const char* name) {
    const int32_t t = tagIndex(name);
    char line[160];
    if (t >= 0) {
        tagLine((uint16_t)t, line, sizeof line);
        emitLine(line);
        return;
    }
    // A tag the HMI doesn't show.
    iTagSource& src = hmi_.source();
    const int32_t h = src.find(name);
    if (h < 0) { emitLine("no tag ", name); return; }
    xTagValue v;
    line[0] = 0;
    append(line, sizeof line, name);
    append(line, sizeof line, " = ");
    if (!src.read(h, v) || v.kind == xTagValue::Kind::None) append(line, sizeof line, "--");
    else if (v.kind == xTagValue::Kind::Bool) append(line, sizeof line, v.b ? "1" : "0");
    else appendNumber(line, sizeof line, v.number, decimalsFor(v.number));
    emitLine(line);
}

void xShell::set(const char* name, const char* text, uint32_t nowMs) {
    (void)nowMs;
    const int32_t t = tagIndex(name);
    const xHmiItem* f = t >= 0 ? field((uint16_t)t) : nullptr;
    if (f == nullptr) { emitLine(name, ": not an editable field"); return; }
    const xHmiLevel need = f->edit->level;
    if (need != xHmiLevel::None && (uint8_t)unlocked_ < (uint8_t)need) {
        emitLine(lockout_ ? "locked out" : "locked: pin first (", lockout_ ? "" : levelName(need), lockout_ ? "" : ")");
        return;
    }
    double x = NAN;
    bool parsed = false;
    if (f->kind == xHmiKind::State) {
        if ((f->on && sameText(text, f->on)) || sameText(text, "on") || sameText(text, "true")) { x = 1; parsed = true; }
        else if ((f->off && sameText(text, f->off)) || sameText(text, "off") || sameText(text, "false")) { x = 0; parsed = true; }
        else parsed = parseNumber(text, x);
    } else if (f->kind == xHmiKind::Choice) {
        for (uint8_t i = 0; i < f->choiceCount && !parsed; ++i)
            if (sameText(text, f->choices[i].label)) { x = f->choices[i].value; parsed = true; }
        if (!parsed) parsed = parseNumber(text, x);
    } else {
        parsed = parseNumber(text, x);
    }
    xTagValue after;
    if (!parsed || !xHmi::fieldAccepts(*f, x, after)) {
        char line[160];
        tagLine((uint16_t)t, line, sizeof line);
        emitLine("not allowed: ", line);
        return;
    }
    iTagSource& src = hmi_.source();
    const int32_t h = src.find(name);
    xTagValue before;
    if (h < 0 || !src.read(h, before)) before = xTagValue::none();
    const bool ok = h >= 0 && src.write(h, after);
    if (hmi_.audit()) hmi_.audit()(name, before, after, ok, unlocked_, hmi_.auditContext());
    if (!ok) { emitLine("refused"); return; }
    char line[160];
    tagLine((uint16_t)t, line, sizeof line);
    emitLine("written: ", line);
}

void xShell::pin(const char* digits, uint32_t nowMs) {
    if (lockout_) { emitLine("locked out"); return; }
    const xHmiLevel l = hmi_.checkPin(digits);
    if (l != xHmiLevel::None) {
        unlocked_ = l;
        wrong_ = 0;
        emitLine("unlocked: ", levelName(l));
        return;
    }
    if (++wrong_ >= xHmi::kPinTries) {
        wrong_ = 0;
        lockout_ = true;
        lockoutAt_ = nowMs;
        unlocked_ = xHmiLevel::None;
        emitLine("wrong PIN: locked out for 60 s");
    } else {
        emitLine("wrong PIN");
    }
}

void xShell::log(const char* n) {
    if (log_ == nullptr) { emitLine("no log"); return; }
    double want = kLogDefault;
    if (n && (!parseNumber(n, want) || want < 1 || want != floor(want))) { emitLine("log [n]: n a whole number"); return; }
    const uint16_t have = log_->count();
    listing_ = Listing::Log;
    listAt_ = 0;
    listEnd_ = (uint16_t)(want < have ? want : have);
}

} // namespace idisplay
