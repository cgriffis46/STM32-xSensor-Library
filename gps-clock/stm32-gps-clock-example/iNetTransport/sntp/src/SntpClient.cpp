#include "SntpClient.h"
#include <cstring>

namespace {

// Waits after each try: three tries, then the request fails (8 s).
constexpr uint32_t kRetryMs[] = {2000, 2000, 4000};
constexpr uint8_t  kTries = sizeof kRetryMs / sizeof kRetryMs[0];

constexpr uint32_t kNtpToUnix = 2208988800u;   // 1900-01-01 to 1970-01-01, s

uint32_t be32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | p[3];
}
void put32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 24); p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);  p[3] = static_cast<uint8_t>(v);
}
uint32_t mix(uint32_t x) {   // a bit mixer, so consecutive tries differ everywhere
    x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16;
    return x;
}

} // namespace

uint64_t SntpClient::ntpToUnixMs(uint32_t sec, uint32_t frac) {
    // Era 0 (1900-2036) has the top bit set from 1968 on; anything
    // with it clear is taken as era 1, after February 2036.
    const uint64_t s = (sec & 0x80000000u) ? static_cast<uint64_t>(sec) - kNtpToUnix
                                           : static_cast<uint64_t>(sec) + 0x100000000ull - kNtpToUnix;
    return s * 1000u + ((static_cast<uint64_t>(frac) * 1000u + 0x80000000u) >> 32);   // rounded to the nearest ms
}

bool SntpClient::begin(const IpAddress& server, uint32_t nonce, uint32_t nowMs) {
    busy_ = false;
    kod_ = false;
    if (server.isZero()) return false;
    server_ = server;
    nonceHi_ = mix(nonce);
    nonceLo_ = mix(nonce ^ 0x9E3779B9u);
    tries_ = 0;
    busy_ = true;
    txPending_ = true;
    nextTx_ = nowMs;
    txDelay_ = 0;
    return true;
}

SntpClient::Action SntpClient::poll(uint32_t nowMs, uint8_t* buf, size_t cap) {
    Action a;
    if (!busy_ || !txPending_ || (nowMs - nextTx_) < txDelay_) return a;
    if (tries_ >= kTries) {
        busy_ = false;
        a.event = Event::Failed;
        return a;
    }
    if (buf == nullptr || cap < kPacket) return a;
    if (tries_ > 0) {   // a fresh nonce per try: an answer to an earlier one doesn't count
        nonceHi_ = mix(nonceHi_ + tries_);
        nonceLo_ = mix(nonceLo_ ^ nonceHi_);
    }
    std::memset(buf, 0, kPacket);
    buf[0] = 0x23;      // LI 0, version 4, mode 3 (client)
    put32(buf + 40, nonceHi_);   // transmit timestamp: the server echoes it back
    put32(buf + 44, nonceLo_);
    a.len = kPacket;
    a.dst = server_;
    sentAt_ = nowMs;
    nextTx_ = nowMs;
    txDelay_ = kRetryMs[tries_++];
    return a;
}

uint32_t SntpClient::nextWakeMs(uint32_t nowMs) const {
    if (!busy_ || !txPending_) return 0xFFFFFFFFu;
    const uint32_t gone = nowMs - nextTx_;
    return gone >= txDelay_ ? 0 : txDelay_ - gone;
}

SntpClient::Action SntpClient::receive(const uint8_t* p, size_t len, uint32_t nowMs) {
    Action a;
    if (!busy_ || p == nullptr || len < kPacket) return a;
    const uint8_t li = p[0] >> 6, vn = (p[0] >> 3) & 7, mode = p[0] & 7;
    if (mode != 4 || vn < 3 || vn > 4) return a;              // a server's reply
    if (be32(p + 24) != nonceHi_ || be32(p + 28) != nonceLo_) return a;   // to this request

    stratum_ = p[1];
    if (stratum_ == 0) {   // kiss-o'-death: the server says stop asking
        kod_ = true;
        busy_ = false;
        a.event = Event::Failed;
        return a;
    }
    const uint32_t rxSec = be32(p + 32), rxFrac = be32(p + 36);
    const uint32_t txSec = be32(p + 40), txFrac = be32(p + 44);
    if (li == 3 || stratum_ > 15 || (txSec == 0 && txFrac == 0)) {   // server not synchronised
        busy_ = false;
        a.event = Event::Failed;
        return a;
    }

    // Round trip less the time the server held it, then the server's
    // transmit time plus half the trip back.
    const uint64_t t2 = ntpToUnixMs(rxSec, rxFrac), t3 = ntpToUnixMs(txSec, txFrac);
    const uint32_t held = t3 > t2 ? static_cast<uint32_t>(t3 - t2) : 0;
    const uint32_t trip = nowMs - sentAt_;
    rttMs_ = trip > held ? trip - held : 0;
    unixMs_ = t3 + rttMs_ / 2;
    atMs_ = nowMs;
    busy_ = false;
    a.event = Event::Synced;
    return a;
}
