#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include "NetTypes.h"
#include "iUdpResponder.h"

// A small NTP server (RFC 5905, server mode, answering clients and SNTP
// clients alike) as pure logic: a request in, a reply out. It is an
// iUdpResponder, so a chip driver with a UDP responder runs it in its
// driver thread (W5500: w5500_param_t::responder, responderPort = 123):
//
//     struct RtcClock : NtpServer::Clock { bool ntpNow(uint64_t* t) override { ... } } clock;
//     NtpServer ntp(clock, -12);              // the clock's resolution: 2^-12 s
//     param.responder = &ntp; param.responderPort = NtpServer::kPort;
//     ...
//     ntp.setReference(1, NtpServer::refIdAscii("GPS"), whenSet, 0, 100000);   // set from GPS
//
// The time comes from Clock::ntpNow(), read twice per request: when the
// request is handed over (the receive timestamp) and just before the
// reply is returned (the transmit timestamp). The server says how good
// that time is from what setReference() was last told: stratum,
// reference ID, when the clock was last set, and the root delay and
// dispersion then. The dispersion grows by driftPpm for every second
// since, as RFC 5905 has a server do. Until setReference() (and after
// setUnsynchronized()) replies say "not synchronised" (leap indicator
// 3, stratum 16), which clients ignore.
//
// Only client-mode requests (mode 3) of NTP versions 1 to 4, 48 bytes or
// more, are answered, with a 48 byte reply of the same version. Control
// and private modes (6, 7) and everything else get nothing, so the
// server can't be used to amplify traffic. Extension fields and MACs in
// a request are ignored.
//
// setReference()/setUnsynchronized() may be called from any thread
// while respond() runs in another: the reference is kept behind a
// sequence counter, and a respond() that keeps meeting a write in
// progress sends nothing (the client tries again).
class NtpServer : public iUdpResponder {
public:
    static constexpr uint16_t kPort = 123;
    static constexpr size_t   kPacket = 48;
    static constexpr uint32_t kNtpToUnix = 2208988800u;   // 1900-01-01 to 1970-01-01, s

    // The time to serve, as an NTP timestamp: seconds since 1900 in the
    // high 32 bits (wrapping in 2036, as NTP's do), the fraction of a
    // second in units of 2^-32 in the low. false: the clock can't be
    // read just now (no reply is sent). Called from respond(), so from
    // the driver thread: short, and never blocking for long.
    class Clock {
    public:
        virtual ~Clock() = default;
        virtual bool ntpNow(uint64_t* ntpTime) = 0;
    };

    struct Stats {
        uint32_t requests;   // datagrams handed over
        uint32_t replies;
        uint32_t ignored;    // not a client request, too short, or the clock unreadable
    };

    // precisionLog2: the clock's resolution as a power of two, in
    // seconds (-12 for 1/4096 s). driftPpm: how fast the clock may wander
    // from true time once set (20 for a watch crystal).
    explicit NtpServer(Clock& clock, int8_t precisionLog2 = -10, uint32_t driftPpm = 20);

    // The clock was set (or confirmed) at refTime (an NTP timestamp, as
    // ntpNow() gives) from a source of this stratum, so the server is
    // one more: 1 for a GPS or other reference clock, the upstream
    // server's stratum + 1 for time from NTP. refId: for stratum 1 four
    // ASCII characters (refIdAscii("GPS")), otherwise the upstream
    // server's IPv4 address (refIdIp()). rootDelayUs/rootDispersionUs:
    // the round trip to, and the error of, the primary reference at
    // refTime (for GPS time from NMEA, say 0 and 100000).
    void setReference(uint8_t stratum, uint32_t refId, uint64_t refTime, uint32_t rootDelayUs,
                      uint32_t rootDispersionUs);
    // Replies say "not synchronised" from now on.
    void setUnsynchronized();
    bool synchronized() const { return synced_.load(std::memory_order_acquire) != 0; }

    // iUdpResponder
    size_t respond(const IpAddress& from, uint16_t fromPort, uint8_t* buf, size_t len, size_t cap,
                   uint32_t nowMs) override;

    Stats stats() const {
        Stats s;
        s.requests = requests_.load(std::memory_order_relaxed);
        s.replies = replies_.load(std::memory_order_relaxed);
        s.ignored = ignored_.load(std::memory_order_relaxed);
        return s;
    }

    // ---- helpers ----
    static uint32_t refIdAscii(const char* s);          // up to 4 characters, NUL padded
    static uint32_t refIdIp(const IpAddress& ip);
    // Unix seconds and a 2^-32 fraction as an NTP timestamp.
    static uint64_t fromUnix(uint32_t unixSeconds, uint32_t fraction) {
        return (static_cast<uint64_t>(unixSeconds + kNtpToUnix) << 32) | fraction;
    }
    // Microseconds as NTP's 16.16 "short format" seconds, saturating.
    static uint32_t shortFormat(uint64_t us);

private:
    Clock&  clock_;
    int8_t  precision_;
    uint32_t driftPpm_;

    // The reference, behind seq_ (odd while being written).
    std::atomic<uint32_t> seq_{0};
    std::atomic<uint32_t> synced_{0};
    std::atomic<uint32_t> stratum_{16};
    std::atomic<uint32_t> refId_{0};
    std::atomic<uint32_t> refSec_{0}, refFrac_{0};
    std::atomic<uint32_t> rootDelayUs_{0}, rootDispUs_{0};

    std::atomic<uint32_t> requests_{0}, replies_{0}, ignored_{0};
};
