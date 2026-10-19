#include "DebugLog.h"

// Always compiled, whatever ITRANSPORT_DEBUG is, so an application that
// turns the hooks on links against a library that was built without
// them (it then gets the drivers' hooks, not the library's own).

namespace dbg {
namespace {

static_assert((ITRANSPORT_DEBUG_RING & (ITRANSPORT_DEBUG_RING - 1)) == 0 && ITRANSPORT_DEBUG_RING >= 128,
              "ITRANSPORT_DEBUG_RING must be a power of two, 128 or more");

const size_t kRing = ITRANSPORT_DEBUG_RING;
const size_t kMaxLine = 112;   // longer lines are cut short
const size_t kTxBuf = 128;     // what one write() hands the UART

struct State {
    iTransport* out = nullptr;
    Port        port = {nullptr, nullptr, nullptr, nullptr};
    // The ring. Producers (any context) and poll() both change it only
    // under the port's lock.
    char     ring[kRing];
    uint32_t head = 0, tail = 0;
    uint32_t seq = 0, dropped = 0, sent = 0;
    // poll()'s two buffers: one may still be going out while the other
    // is filled.
    char     tx[2][kTxBuf];
    size_t   txLen = 0;   // bytes waiting in tx[txNext], not yet taken by write()
    int      txNext = 0;
};

State s;

struct Lock {
    uint32_t saved = 0;
    Lock() { if (s.port.lock) saved = s.port.lock(); }
    ~Lock() { if (s.port.unlock) s.port.unlock(saved); }
};

// Appends text, up to the buffer's end.
void put(char* b, size_t& n, const char* t) {
    while (*t && n < kMaxLine - 2) b[n++] = *t++;
}

void putNumber(char* b, size_t& n, int64_t v) {
    char d[20];
    int k = 0;
    const bool neg = v < 0;
    uint64_t u = neg ? (uint64_t)(-v) : (uint64_t)v;
    do { d[k++] = (char)('0' + u % 10); u /= 10; } while (u);
    if (neg && n < kMaxLine - 2) b[n++] = '-';
    while (k > 0 && n < kMaxLine - 2) b[n++] = d[--k];
}

} // namespace

void begin(iTransport* out, const Port& port) {
    s.out = out;
    s.port = port;
}

void reset() {
    s.out = nullptr;
    s.port = Port{nullptr, nullptr, nullptr, nullptr};
    s.head = s.tail = s.seq = s.dropped = s.sent = 0;
    s.txLen = 0;
    s.txNext = 0;
}

void line(const char* tag, const char* what, const int32_t* values, size_t count) {
    if (s.out == nullptr) return;   // no debug UART: nothing to keep lines for

    // Everything but the sequence number is formatted outside the lock.
    const uint32_t ms = s.port.nowMs ? s.port.nowMs() : 0;
    char body[kMaxLine];
    size_t n = 0;
    body[n++] = ' ';
    putNumber(body, n, ms);
    body[n++] = ' ';
    put(body, n, tag ? tag : "?");
    body[n++] = ' ';
    put(body, n, what ? what : "?");
    if (count > kMaxValues) count = kMaxValues;
    for (size_t i = 0; i < count; ++i) {
        if (n < kMaxLine - 2) body[n++] = ' ';
        putNumber(body, n, values[i]);
    }
    body[n++] = '\r';
    body[n++] = '\n';

    Lock lock;
    const uint32_t seq = ++s.seq;
    char num[12];
    size_t k = 0;
    putNumber(num, k, seq);
    const size_t total = k + n;
    if (kRing - (s.head - s.tail) < total) {
        ++s.dropped;
        return;
    }
    for (size_t i = 0; i < k; ++i) s.ring[(s.head++) & (kRing - 1)] = num[i];
    for (size_t i = 0; i < n; ++i) s.ring[(s.head++) & (kRing - 1)] = body[i];
}

void pin(uint8_t id, bool level) {
    if (s.port.pin) s.port.pin(id, level);
}

void poll() {
    if (s.out == nullptr) return;
    // A synchronous UART takes every write at once; a DMA one takes the
    // next only after the last has gone. Either way stop when refused.
    for (int round = 0; round < 8; ++round) {
        if (s.txLen == 0) {
            Lock lock;
            char* b = s.tx[s.txNext];
            while (s.txLen < kTxBuf && s.tail != s.head) {
                b[s.txLen++] = s.ring[(s.tail++) & (kRing - 1)];
            }
        }
        if (s.txLen == 0) return;
        if (!s.out->write((const uint8_t*)s.tx[s.txNext], s.txLen)) return;   // still sending: next time
        s.sent += (uint32_t)s.txLen;
        s.txLen = 0;
        s.txNext ^= 1;
    }
}

Stats stats() {
    Lock lock;
    Stats r;
    r.lines = s.seq;
    r.dropped = s.dropped;
    r.bytesSent = s.sent;
    return r;
}

} // namespace dbg
