#include "DhcpClient.h"
#include <cstring>

namespace {

constexpr uint32_t kInfinite      = 0xFFFFFFFFu;
constexpr uint32_t kMaxLeaseSec   = 24u * 24u * 3600u; // 24 days: deadlines stay inside a 32-bit ms counter
constexpr uint32_t kFirstRetryMs  = 2000;
constexpr uint32_t kMaxRetryMs    = 32000;
constexpr uint8_t  kRequestTries  = 4;               // REQUESTs for one offer before starting over
constexpr uint32_t kMinRenewMs    = 4000;
constexpr uint32_t kMaxRenewMs    = 60000;

constexpr uint8_t kMagic[4] = {99, 130, 83, 99};

enum Opt : uint8_t {
    OptPad = 0, OptSubnet = 1, OptRouter = 3, OptDns = 6, OptRequestedIp = 50, OptLease = 51,
    OptNtp = 42, OptMsgType = 53, OptServerId = 54, OptParams = 55, OptMaxSize = 57, OptT1 = 58, OptT2 = 59,
    OptClientId = 61, OptEnd = 255
};

uint32_t be32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | p[3];
}
void put32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 24); p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);  p[3] = static_cast<uint8_t>(v);
}
IpAddress ipAt(const uint8_t* p) { return IpAddress(p[0], p[1], p[2], p[3]); }
uint32_t remaining(uint32_t now, uint32_t t0, uint32_t ms) {
    const uint32_t gone = now - t0;
    return gone >= ms ? 0 : ms - gone;
}
uint32_t clamp(uint32_t v, uint32_t lo, uint32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

// What one reply carried.
struct Reply {
    uint8_t   type = 0;
    IpAddress yiaddr, subnet, router, dns, ntp, serverId;
    bool      hasSubnet = false, hasServerId = false;
    uint32_t  leaseSec = 0, t1Sec = 0, t2Sec = 0;
    bool      hasLease = false, hasT1 = false, hasT2 = false;
};

// False for anything that isn't a well-formed reply.
bool parse(const uint8_t* p, size_t len, Reply& r) {
    if (len < 240 || std::memcmp(p + 236, kMagic, 4) != 0) return false;
    r.yiaddr = ipAt(p + 16);
    size_t i = 240;
    while (i < len) {
        const uint8_t code = p[i];
        if (code == OptPad) { ++i; continue; }
        if (code == OptEnd) break;
        if (i + 1 >= len) return false;
        const uint8_t n = p[i + 1];
        const uint8_t* v = p + i + 2;
        if (i + 2 + n > len) return false;
        switch (code) {
        case OptMsgType:  if (n >= 1) r.type = v[0]; break;
        case OptSubnet:   if (n >= 4) { r.subnet = ipAt(v); r.hasSubnet = true; } break;
        case OptRouter:   if (n >= 4) r.router = ipAt(v); break;   // the first of the list
        case OptDns:      if (n >= 4) r.dns = ipAt(v); break;
        case OptNtp:      if (n >= 4) r.ntp = ipAt(v); break;
        case OptServerId: if (n >= 4) { r.serverId = ipAt(v); r.hasServerId = true; } break;
        case OptLease:    if (n >= 4) { r.leaseSec = be32(v); r.hasLease = true; } break;
        case OptT1:       if (n >= 4) { r.t1Sec = be32(v); r.hasT1 = true; } break;
        case OptT2:       if (n >= 4) { r.t2Sec = be32(v); r.hasT2 = true; } break;
        default: break;
        }
        i += 2 + n;
    }
    return r.type != 0;
}

uint32_t secToMs(uint32_t sec) {
    if (sec > kMaxLeaseSec) sec = kMaxLeaseSec;
    return sec * 1000u;
}

} // namespace

void DhcpClient::begin(const MacAddress& mac, uint32_t seed, uint32_t nowMs, const IpAddress& previous) {
    mac_ = mac;
    seed_ = seed ^ (static_cast<uint32_t>(mac.b[2]) << 24) ^ (static_cast<uint32_t>(mac.b[3]) << 16) ^
            (static_cast<uint32_t>(mac.b[4]) << 8) ^ mac.b[5];
    requested_ = previous;
    restart(nowMs);
}

void DhcpClient::restart(uint32_t nowMs) {
    state_ = State::Selecting;
    seed_ = seed_ * 1664525u + 1013904223u; // a fresh transaction
    xid_ = seed_;
    server_ = IpAddress();
    retryMs_ = kFirstRetryMs;
    tries_ = 0;
    sendNow(nowMs);
}

void DhcpClient::linkRestored(uint32_t nowMs) {
    if (state_ == State::Stopped) return;
    if (!bound()) {
        restart(nowMs);
        return;
    }
    // Possibly a different network now. Ask any server to confirm the
    // lease; a NAK means it isn't ours here.
    state_ = State::Rebinding;
    seed_ = seed_ * 1664525u + 1013904223u;
    xid_ = seed_;
    retryMs_ = kFirstRetryMs;
    tries_ = 0;
    sendNow(nowMs);
}

void DhcpClient::scheduleRetry(uint32_t nowMs) {
    ++tries_;
    uint32_t delay = retryMs_;
    if (state_ == State::Selecting || state_ == State::Requesting) {
        retryMs_ = retryMs_ * 2 > kMaxRetryMs ? kMaxRetryMs : retryMs_ * 2;
    } else {
        // Half the time left before the next stage, within limits
        // (RFC 2131 4.4.5 says at least 60 s; a little keener here).
        const uint32_t gone = nowMs - boundAt_;
        uint32_t left = kMaxRenewMs * 2;
        if (leaseMs_ != kInfinite) {
            const uint32_t until = state_ == State::Renewing ? t2Ms_ : leaseMs_;
            left = until > gone ? until - gone : 0;
        }
        delay = clamp(left / 2, kMinRenewMs, kMaxRenewMs);
    }
    nextTx_ = nowMs;
    txDelay_ = delay;
    txPending_ = true;
}

DhcpClient::Action DhcpClient::poll(uint32_t nowMs, uint8_t* buf, size_t cap) {
    Action a;
    if (state_ == State::Stopped) return a;

    if (bound() && leaseMs_ != kInfinite) {
        const uint32_t gone = nowMs - boundAt_;
        if (gone >= leaseMs_) {
            a.event = Event::Lost;
            requested_ = lease_.ip; // ask for it back
            lease_ = NetConfig();
            restart(nowMs);
        } else if (state_ == State::Bound && gone >= t1Ms_) {
            state_ = State::Renewing;
            xid_ = seed_ = seed_ * 1664525u + 1013904223u;
            sendNow(nowMs);
        } else if (state_ == State::Renewing && gone >= t2Ms_) {
            state_ = State::Rebinding;
            sendNow(nowMs);
        }
    }

    if (!txPending_ || !after(nowMs, nextTx_, txDelay_)) return a;

    if (state_ == State::Requesting && tries_ >= kRequestTries) restart(nowMs); // the offer went stale

    a.len = build(buf, cap, state_ == State::Selecting ? Discover : Request);
    a.dst = state_ == State::Renewing ? server_ : IpAddress(255, 255, 255, 255);
    scheduleRetry(nowMs);
    if (a.len == 0) txPending_ = true; // buffer too small: try again
    return a;
}

DhcpClient::Action DhcpClient::receive(const uint8_t* p, size_t len, uint32_t nowMs) {
    Action a;
    if (state_ == State::Stopped || state_ == State::Bound || p == nullptr) return a;
    if (len < 240 || p[0] != 2 /* BOOTREPLY */ || p[1] != 1 || p[2] != 6) return a;
    if (be32(p + 4) != xid_ || std::memcmp(p + 28, mac_.b, 6) != 0) return a;
    Reply r;
    if (!parse(p, len, r)) return a;

    switch (state_) {
    case State::Selecting:
        if (r.type != Offer || r.yiaddr.isZero() || !r.hasServerId) return a;
        server_ = r.serverId;
        requested_ = r.yiaddr;
        state_ = State::Requesting;
        retryMs_ = kFirstRetryMs;
        tries_ = 0;
        sendNow(nowMs);
        return a;

    case State::Requesting:
    case State::Renewing:
    case State::Rebinding: {
        if (state_ == State::Requesting && r.hasServerId && r.serverId != server_) return a; // another server's answer
        if (r.type == Nak) {
            if (bound()) a.event = Event::Lost;
            lease_ = NetConfig();
            requested_ = IpAddress();
            restart(nowMs);
            return a;
        }
        if (r.type != Ack || r.yiaddr.isZero()) return a;

        lease_ = NetConfig();
        lease_.mac = mac_;
        lease_.dhcp = true;
        lease_.ip = r.yiaddr;
        lease_.subnet = r.hasSubnet ? r.subnet : IpAddress(255, 255, 255, 0);
        lease_.gateway = r.router;
        lease_.dns = r.dns;
        lease_.ntp = r.ntp;
        if (r.hasServerId) server_ = r.serverId;
        requested_ = r.yiaddr;

        leaseSec_ = r.hasLease ? r.leaseSec : 3600;
        boundAt_ = nowMs;
        if (leaseSec_ == kInfinite) {
            leaseMs_ = t1Ms_ = t2Ms_ = kInfinite;
        } else {
            leaseMs_ = secToMs(leaseSec_);
            t1Ms_ = r.hasT1 ? secToMs(r.t1Sec) : leaseMs_ / 2;
            t2Ms_ = r.hasT2 ? secToMs(r.t2Sec) : leaseMs_ / 8 * 7;
            if (t2Ms_ >= leaseMs_) t2Ms_ = leaseMs_ / 8 * 7;
            if (t1Ms_ >= t2Ms_) t1Ms_ = t2Ms_ / 8 * 7;
        }
        state_ = State::Bound;
        txPending_ = false;
        a.event = Event::Bound;
        return a;
    }
    default:
        return a;
    }
}

uint32_t DhcpClient::nextWakeMs(uint32_t nowMs) const {
    if (state_ == State::Stopped) return kInfinite;
    uint32_t wait = kInfinite;
    if (txPending_) wait = remaining(nowMs, nextTx_, txDelay_);
    if (bound() && leaseMs_ != kInfinite) {
        uint32_t w = remaining(nowMs, boundAt_, leaseMs_);
        if (state_ == State::Bound) w = remaining(nowMs, boundAt_, t1Ms_);
        else if (state_ == State::Renewing && remaining(nowMs, boundAt_, t2Ms_) < w) w = remaining(nowMs, boundAt_, t2Ms_);
        if (w < wait) wait = w;
    }
    return wait;
}

size_t DhcpClient::build(uint8_t* b, size_t cap, uint8_t type) const {
    if (b == nullptr || cap < kTxPacket) return 0;
    std::memset(b, 0, kTxPacket);
    b[0] = 1;                               // BOOTREQUEST
    b[1] = 1;                               // Ethernet
    b[2] = 6;
    put32(b + 4, xid_);
    // Ask for replies by broadcast while we have no address to receive
    // a unicast on. Renewing, we have one.
    if (state_ != State::Renewing) b[10] = 0x80;
    if (state_ == State::Renewing || state_ == State::Rebinding) std::memcpy(b + 12, lease_.ip.b, 4); // ciaddr
    std::memcpy(b + 28, mac_.b, 6);         // chaddr
    std::memcpy(b + 236, kMagic, 4);

    uint8_t* o = b + 240;
    *o++ = OptMsgType; *o++ = 1; *o++ = type;
    *o++ = OptClientId; *o++ = 7; *o++ = 1;
    std::memcpy(o, mac_.b, 6); o += 6;
    const bool requesting = type == Request && state_ != State::Renewing && state_ != State::Rebinding;
    if (requesting || (type == Discover && !requested_.isZero())) {
        *o++ = OptRequestedIp; *o++ = 4;
        std::memcpy(o, requested_.b, 4); o += 4;
    }
    if (requesting) {
        *o++ = OptServerId; *o++ = 4;
        std::memcpy(o, server_.b, 4); o += 4;
    }
    *o++ = OptMaxSize; *o++ = 2; *o++ = static_cast<uint8_t>(kMaxPacket >> 8); *o++ = static_cast<uint8_t>(kMaxPacket);
    *o++ = OptParams; *o++ = 7;
    *o++ = OptSubnet; *o++ = OptRouter; *o++ = OptDns; *o++ = OptNtp; *o++ = OptLease; *o++ = OptT1; *o++ = OptT2;
    *o++ = OptEnd;
    return kTxPacket;
}
