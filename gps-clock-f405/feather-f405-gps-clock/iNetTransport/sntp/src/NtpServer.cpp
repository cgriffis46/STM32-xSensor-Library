#include "NtpServer.h"
#include <cstring>

namespace {

uint32_t be32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | p[3];
}
void put32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 24); p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);  p[3] = static_cast<uint8_t>(v);
}
void put64(uint8_t* p, uint64_t v) {
    put32(p, static_cast<uint32_t>(v >> 32));
    put32(p + 4, static_cast<uint32_t>(v));
}

constexpr uint8_t kModeClient = 3;
constexpr uint8_t kModeServer = 4;
constexpr uint8_t kLeapNone = 0;
constexpr uint8_t kLeapAlarm = 3;     // not synchronised
constexpr uint8_t kStratumUnsync = 16;
constexpr int     kReadTries = 4;     // of the reference, against a writer

} // namespace

NtpServer::NtpServer(Clock& clock, int8_t precisionLog2, uint32_t driftPpm)
    : clock_(clock), precision_(precisionLog2), driftPpm_(driftPpm) {}

uint32_t NtpServer::refIdAscii(const char* s) {
    uint8_t b[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4 && s != nullptr && s[i] != 0; ++i) b[i] = static_cast<uint8_t>(s[i]);
    return be32(b);
}

uint32_t NtpServer::refIdIp(const IpAddress& ip) {
    return be32(ip.b);
}

uint32_t NtpServer::shortFormat(uint64_t us) {
    // seconds x 65536, from microseconds; at most 0xFFFF.FFFF
    const uint64_t v = (us * 65536u + 500000u) / 1000000u;
    return v > 0xFFFFFFFFu ? 0xFFFFFFFFu : static_cast<uint32_t>(v);
}

void NtpServer::setReference(uint8_t stratum, uint32_t refId, uint64_t refTime, uint32_t rootDelayUs,
                             uint32_t rootDispersionUs) {
    if (stratum < 1) stratum = 1;
    if (stratum > 15) stratum = 15;
    seq_.fetch_add(1, std::memory_order_acq_rel);     // odd: a write in progress
    std::atomic_thread_fence(std::memory_order_release);
    stratum_.store(stratum, std::memory_order_relaxed);
    refId_.store(refId, std::memory_order_relaxed);
    refSec_.store(static_cast<uint32_t>(refTime >> 32), std::memory_order_relaxed);
    refFrac_.store(static_cast<uint32_t>(refTime), std::memory_order_relaxed);
    rootDelayUs_.store(rootDelayUs, std::memory_order_relaxed);
    rootDispUs_.store(rootDispersionUs, std::memory_order_relaxed);
    synced_.store(1, std::memory_order_relaxed);
    seq_.fetch_add(1, std::memory_order_release);     // even again
}

void NtpServer::setUnsynchronized() {
    seq_.fetch_add(1, std::memory_order_acq_rel);
    std::atomic_thread_fence(std::memory_order_release);
    synced_.store(0, std::memory_order_relaxed);
    stratum_.store(kStratumUnsync, std::memory_order_relaxed);
    seq_.fetch_add(1, std::memory_order_release);
}

size_t NtpServer::respond(const IpAddress& from, uint16_t fromPort, uint8_t* buf, size_t len, size_t cap,
                          uint32_t nowMs) {
    (void)from; (void)fromPort; (void)nowMs;
    requests_.fetch_add(1, std::memory_order_relaxed);

    // The receive timestamp first, before anything else takes time.
    uint64_t rx = 0;
    const bool clockOk = clock_.ntpNow(&rx);

    const uint8_t version = len != 0 ? static_cast<uint8_t>((buf[0] >> 3) & 7) : 0;
    const uint8_t mode = len != 0 ? static_cast<uint8_t>(buf[0] & 7) : 0;
    if (!clockOk || buf == nullptr || len < kPacket || cap < kPacket || mode != kModeClient || version < 1 ||
        version > 4) {
        ignored_.fetch_add(1, std::memory_order_relaxed);
        return 0;
    }

    // The reference, consistent with itself.
    bool synced = false;
    uint32_t stratum = kStratumUnsync, refId = 0, refSec = 0, refFrac = 0, delayUs = 0, dispUs = 0;
    bool got = false;
    for (int i = 0; i < kReadTries && !got; ++i) {
        const uint32_t s1 = seq_.load(std::memory_order_acquire);
        if (s1 & 1u) continue;
        synced = synced_.load(std::memory_order_relaxed) != 0;
        stratum = stratum_.load(std::memory_order_relaxed);
        refId = refId_.load(std::memory_order_relaxed);
        refSec = refSec_.load(std::memory_order_relaxed);
        refFrac = refFrac_.load(std::memory_order_relaxed);
        delayUs = rootDelayUs_.load(std::memory_order_relaxed);
        dispUs = rootDispUs_.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        got = seq_.load(std::memory_order_relaxed) == s1;
    }
    if (!got) {
        ignored_.fetch_add(1, std::memory_order_relaxed);
        return 0;
    }

    // The client's transmit timestamp comes back as our originate.
    uint8_t origin[8];
    std::memcpy(origin, buf + 40, 8);
    const uint8_t poll = buf[2];

    std::memset(buf, 0, kPacket);
    if (synced) {
        // The dispersion grows with the time since the clock was set:
        // driftPpm us for every second (RFC 5905's PHI).
        const uint64_t ref = (static_cast<uint64_t>(refSec) << 32) | refFrac;
        const uint64_t age = rx - ref;                     // NTP units; wraps right across 2036
        const uint64_t ageSec = (age >> 32) < 0x80000000u ? (age >> 32) : 0;   // a clock set "later": no age
        const uint64_t disp = dispUs + ageSec * driftPpm_;
        buf[0] = static_cast<uint8_t>((kLeapNone << 6) | (version << 3) | kModeServer);
        buf[1] = static_cast<uint8_t>(stratum);
        put32(buf + 4, shortFormat(delayUs));
        put32(buf + 8, shortFormat(disp));
        put32(buf + 12, refId);
        put64(buf + 16, ref);
    } else {
        buf[0] = static_cast<uint8_t>((kLeapAlarm << 6) | (version << 3) | kModeServer);
        buf[1] = kStratumUnsync;
        put32(buf + 12, refIdAscii("INIT"));
    }
    buf[2] = poll;
    buf[3] = static_cast<uint8_t>(precision_);
    std::memcpy(buf + 24, origin, 8);
    put64(buf + 32, rx);

    // The transmit timestamp last.
    uint64_t tx = rx;
    if (!clock_.ntpNow(&tx)) tx = rx;
    put64(buf + 40, tx);

    replies_.fetch_add(1, std::memory_order_relaxed);
    return kPacket;
}
