#include "DnsClient.h"
#include <cstring>

namespace {

// Waits after each try: three tries, then the lookup fails (10 s).
constexpr uint32_t kRetryMs[] = {2000, 3000, 5000};
constexpr uint8_t  kTries = sizeof kRetryMs / sizeof kRetryMs[0];

constexpr uint16_t kTypeA = 1, kClassIn = 1;

uint16_t be16(const uint8_t* p) { return static_cast<uint16_t>((p[0] << 8) | p[1]); }
void put16(uint8_t* p, uint16_t v) { p[0] = static_cast<uint8_t>(v >> 8); p[1] = static_cast<uint8_t>(v); }

char lower(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c; }

// Steps over a (possibly compressed) name at p[at]; false if it runs
// off the end.
bool skipName(const uint8_t* p, size_t len, size_t& at) {
    for (int labels = 0; labels < 128; ++labels) {
        if (at >= len) return false;
        const uint8_t n = p[at];
        if ((n & 0xC0) == 0xC0) {   // pointer: the name ends here
            if (at + 2 > len) return false;
            at += 2;
            return true;
        }
        if (n & 0xC0) return false; // reserved label types
        ++at;
        if (n == 0) return true;
        at += n;
    }
    return false;
}

} // namespace

bool DnsClient::begin(const char* name, const IpAddress& server, uint16_t id, uint32_t nowMs) {
    busy_ = false;
    addr_ = IpAddress();
    error_ = Error::None;
    if (server.isZero()) { error_ = Error::NoServer; return false; }

    size_t n = name ? std::strlen(name) : 0;
    if (n > 0 && name[n - 1] == '.') --n;  // "example.com." is fine
    if (n == 0 || n > kMaxName) { error_ = Error::BadName; return false; }
    size_t label = 0;
    for (size_t i = 0; i < n; ++i) {
        if (name[i] == '.') {
            if (label == 0) { error_ = Error::BadName; return false; }
            label = 0;
        } else if (++label > 63) {
            error_ = Error::BadName;
            return false;
        }
    }
    if (label == 0) { error_ = Error::BadName; return false; }

    std::memcpy(name_, name, n);
    name_[n] = 0;
    server_ = server;
    id_ = id;
    tries_ = 0;
    busy_ = true;
    txPending_ = true;
    nextTx_ = nowMs;
    txDelay_ = 0;
    return true;
}

DnsClient::Action DnsClient::poll(uint32_t nowMs, uint8_t* buf, size_t cap) {
    Action a;
    if (!busy_ || !txPending_ || (nowMs - nextTx_) < txDelay_) return a;
    if (tries_ >= kTries) {
        busy_ = false;
        error_ = Error::Timeout;
        a.event = Event::Failed;
        return a;
    }
    a.len = build(buf, cap);
    if (a.len == 0) return a;  // buffer too small: try again next poll
    a.dst = server_;
    nextTx_ = nowMs;
    txDelay_ = kRetryMs[tries_++];
    return a;
}

uint32_t DnsClient::nextWakeMs(uint32_t nowMs) const {
    if (!busy_ || !txPending_) return 0xFFFFFFFFu;
    const uint32_t gone = nowMs - nextTx_;
    return gone >= txDelay_ ? 0 : txDelay_ - gone;
}

size_t DnsClient::build(uint8_t* b, size_t cap) const {
    const size_t n = std::strlen(name_);
    const size_t len = 12 + n + 2 + 4;
    if (b == nullptr || cap < len) return 0;
    std::memset(b, 0, 12);
    put16(b, id_);
    b[2] = 0x01;            // RD: recursion desired
    put16(b + 4, 1);        // one question
    // "pool.ntp.org" -> 4 pool 3 ntp 3 org 0
    uint8_t* o = b + 12;
    const char* s = name_;
    while (*s) {
        const char* dot = std::strchr(s, '.');
        const size_t l = dot ? static_cast<size_t>(dot - s) : std::strlen(s);
        *o++ = static_cast<uint8_t>(l);
        std::memcpy(o, s, l);
        o += l;
        s += l;
        if (*s == '.') ++s;
    }
    *o++ = 0;
    put16(o, kTypeA);
    put16(o + 2, kClassIn);
    return len;
}

// The question in the reply must be ours (case-insensitively, as DNS
// names are). Leaves at just past it.
bool DnsClient::questionMatches(const uint8_t* p, size_t len, size_t& at) const {
    const char* s = name_;
    for (;;) {
        if (at >= len) return false;
        const uint8_t n = p[at++];
        if (n == 0) break;
        if (n > 63 || at + n > len) return false;   // compression isn't used in a question
        for (uint8_t i = 0; i < n; ++i) {
            if (*s == 0 || lower(static_cast<char>(p[at + i])) != lower(*s)) return false;
            ++s;
        }
        at += n;
        if (*s == '.') ++s;
        else if (*s != 0) return false;
    }
    if (*s != 0 || at + 4 > len) return false;
    if (be16(p + at) != kTypeA || be16(p + at + 2) != kClassIn) return false;
    at += 4;
    return true;
}

DnsClient::Action DnsClient::receive(const uint8_t* p, size_t len, uint32_t nowMs) {
    (void)nowMs;
    Action a;
    if (!busy_ || p == nullptr || len < 12) return a;
    if (be16(p) != id_) return a;
    const uint16_t flags = be16(p + 2);
    if (!(flags & 0x8000) || ((flags >> 11) & 0xF) != 0) return a;   // not a response to a standard query
    if (be16(p + 4) != 1) return a;
    size_t at = 12;
    if (!questionMatches(p, len, at)) return a;

    const uint8_t rcode = flags & 0xF;
    if (rcode != 0) {
        busy_ = false;
        error_ = rcode == 3 ? Error::NotFound : Error::ServerFailure;
        a.event = Event::Failed;
        return a;
    }
    const uint16_t answers = be16(p + 6);
    for (uint16_t i = 0; i < answers; ++i) {
        if (!skipName(p, len, at) || at + 10 > len) break;
        const uint16_t type = be16(p + at), cls = be16(p + at + 2), rdlen = be16(p + at + 8);
        at += 10;
        if (at + rdlen > len) break;
        if (type == kTypeA && cls == kClassIn && rdlen == 4) {
            addr_ = IpAddress(p[at], p[at + 1], p[at + 2], p[at + 3]);
            busy_ = false;
            a.event = Event::Resolved;
            return a;
        }
        at += rdlen;    // a CNAME, say: its A record follows
    }
    busy_ = false;
    error_ = Error::NotFound;   // answered, but no IPv4 address
    a.event = Event::Failed;
    return a;
}
