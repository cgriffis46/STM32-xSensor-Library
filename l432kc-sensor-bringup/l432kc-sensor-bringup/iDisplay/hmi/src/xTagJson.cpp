#include "xTagJson.h"
#include <math.h>
#include <string.h>
#include "xFormat.h"

namespace idisplay {

namespace {

// Appends to a fixed buffer; once anything doesn't fit, everything after
// is refused and ok() stays false.
struct Out {
    char* p;
    size_t cap, n;
    bool good;
    Out(char* buf, size_t c) : p(buf), cap(c), n(0), good(buf != nullptr && c > 0) {}
    void put(char c) {
        if (!good) return;
        if (n + 1 >= cap) { good = false; return; }   // keep room for the 0
        p[n++] = c;
    }
    void put(const char* s) { while (*s) put(*s++); }
    void put(const char* s, size_t len) { for (size_t i = 0; i < len; ++i) put(s[i]); }
};

int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// One name from [s, e), percent-decoded into buf. False if too long or
// a % escape is broken.
bool decodeName(const char* s, const char* e, char* buf, size_t cap) {
    size_t n = 0;
    while (s < e) {
        char c = *s++;
        if (c == '%') {
            if (e - s < 2) return false;
            const int hi = hexVal(s[0]), lo = hexVal(s[1]);
            if (hi < 0 || lo < 0) return false;
            c = static_cast<char>(hi * 16 + lo);
            s += 2;
        }
        if (n + 1 >= cap) return false;
        buf[n++] = c;
    }
    buf[n] = 0;
    return true;
}

void putString(Out& o, const char* s) {
    static const char kHex[] = "0123456789abcdef";
    o.put('"');
    for (; *s; ++s) {
        const unsigned char c = static_cast<unsigned char>(*s);
        if (c == '"' || c == '\\') { o.put('\\'); o.put(static_cast<char>(c)); }
        else if (c < 0x20) { o.put("\\u00"); o.put(kHex[c >> 4]); o.put(kHex[c & 15]); }
        else o.put(static_cast<char>(c));
    }
    o.put('"');
}

} // namespace

size_t formatJsonNumber(double v, char* out, size_t cap) {
    if (out == nullptr || cap == 0) return 0;
    if (!isfinite(v) || fabs(v) >= 1e15) {
        if (cap < 5) return 0;
        memcpy(out, "null", 5);
        return 4;
    }
    // Decimals for 6 significant digits, 0..6 (formatNumber's limit): a
    // value under 1e-6 in size comes out 0.
    int dec = 0;
    if (v != 0) {
        const int mag = static_cast<int>(floor(log10(fabs(v))));   // 1234.5 -> 3
        dec = 5 - mag;
        if (dec < 0) dec = 0;
        if (dec > 6) dec = 6;
    }
    char tmp[32];
    size_t n = formatNumber(v, static_cast<uint8_t>(dec), tmp, sizeof tmp);
    if (n == 0 || n == kFormatTooLong) {
        if (cap < 5) return 0;
        memcpy(out, "null", 5);
        return 4;
    }
    // No trailing zeros, nor a bare point: JSON allows them, but they say
    // nothing. formatNumber never writes "-0".
    if (memchr(tmp, '.', n) != nullptr) {
        while (n > 0 && tmp[n - 1] == '0') --n;
        if (n > 0 && tmp[n - 1] == '.') --n;
    }
    if (n + 1 > cap) return 0;
    memcpy(out, tmp, n);
    out[n] = 0;
    return n;
}

size_t writeTagsJson(iTagSource& source, const char* names, char* out, size_t cap) {
    Out o(out, cap);
    o.put("{\"tags\":[");
    size_t answered = 0;
    const char* s = names ? names : "";
    while (*s && answered < kTagJsonMaxNames) {
        const char* e = strchr(s, ',');
        if (e == nullptr) e = s + strlen(s);
        if (e > s) {                                     // empty items (a,,b) are skipped
            char name[kTagJsonMaxName + 1];
            const bool decoded = decodeName(s, e, name, sizeof name);
            if (answered) o.put(',');
            o.put("{\"name\":");
            if (decoded) putString(o, name);
            else {
                // Answer with the name as it came, so the page can say which.
                char raw[kTagJsonMaxName + 1];
                const size_t len = static_cast<size_t>(e - s) < kTagJsonMaxName ? static_cast<size_t>(e - s) : kTagJsonMaxName;
                memcpy(raw, s, len);
                raw[len] = 0;
                putString(o, raw);
            }
            const int32_t h = decoded ? source.find(name) : -1;
            if (h < 0) {
                o.put(",\"error\":\"not found\"}");
            } else {
                xTagValue v;
                o.put(",\"value\":");
                if (!source.read(h, v)) o.put("null");
                else if (v.kind == xTagValue::Kind::Bool) o.put(v.b ? "true" : "false");
                else if (v.kind == xTagValue::Kind::Number) {
                    char num[32];
                    const size_t n = formatJsonNumber(v.number, num, sizeof num);
                    if (n) o.put(num, n); else o.put("null");
                } else o.put("null");
                o.put('}');
            }
            ++answered;
        }
        s = *e ? e + 1 : e;
    }
    o.put("]}");
    if (!o.good) {
        if (out && cap) out[0] = 0;
        return 0;
    }
    out[o.n] = 0;
    return o.n;
}

} // namespace idisplay
