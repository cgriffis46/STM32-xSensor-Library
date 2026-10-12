#pragma once
#include <cstddef>
#include <cstdint>
#include "NetTypes.h"

// An SNTP client (RFC 4330) for one time request at a time, as pure
// logic, in the same shape as DhcpClient and DnsClient:
//
//     sntp.begin(serverIp, nonce, now);
//     ...
//     SntpClient::Action a = sntp.poll(now, buf, sizeof buf);
//     if (a.len) send buf[0..a.len) to a.dst, port 123
//     on a datagram from port 123:  a = sntp.receive(data, len, now)
//     a.event == Synced: the time was unixMs() at local time atMs()
//
// The result is Unix time in ms (UTC) at a moment on the caller's own
// ms counter, corrected for half the round trip: the caller's clock
// then runs on from there. Accuracy is a few ms on a LAN.
//
// A reply only counts if it echoes the transmit timestamp of the
// request it answers (a random nonce, per RFC 4330 section 5), comes
// from a synchronised server (stratum 1..15, leap indicator not 3)
// and is a server-mode packet. A kiss-o'-death (stratum 0) fails the
// request rather than being retried, as the RFC asks. NTP's 2036
// rollover is handled.
class SntpClient {
public:
    static constexpr uint16_t kServerPort = 123;
    static constexpr size_t   kPacket = 48;

    enum class Event : uint8_t { None, Synced, Failed };

    struct Action {
        size_t    len = 0;      // bytes to send to dst:123; 0 for none
        IpAddress dst;
        Event     event = Event::None;
    };

    // Starts a request, dropping any in progress. nonce: random bits
    // (mix in a hardware ID and a timer). false for a zero server.
    bool begin(const IpAddress& server, uint32_t nonce, uint32_t nowMs);
    void cancel() { busy_ = false; }

    // Call regularly. buf: room for kPacket bytes.
    Action poll(uint32_t nowMs, uint8_t* buf, size_t cap);

    // A datagram that came from the server's port 123.
    Action receive(const uint8_t* pkt, size_t len, uint32_t nowMs);

    uint32_t nextWakeMs(uint32_t nowMs) const;

    bool     busy() const { return busy_; }
    uint64_t unixMs() const { return unixMs_; }     // after Synced: the time...
    uint32_t atMs() const { return atMs_; }         // ...at this local ms
    uint32_t rttMs() const { return rttMs_; }
    uint8_t  stratum() const { return stratum_; }
    bool     kissOfDeath() const { return kod_; }

    // NTP's (seconds since 1900, 2^-32 fraction) as Unix ms, taking
    // the 2036 rollover into account (timestamps with the top bit clear
    // are after 2036).
    static uint64_t ntpToUnixMs(uint32_t sec, uint32_t frac);

private:
    IpAddress server_;
    uint32_t  nonceHi_ = 0, nonceLo_ = 0;   // transmit timestamp sent, echoed as originate
    bool      busy_ = false;
    bool      txPending_ = false;
    uint8_t   tries_ = 0;
    uint32_t  nextTx_ = 0;
    uint32_t  txDelay_ = 0;
    uint32_t  sentAt_ = 0;
    uint64_t  unixMs_ = 0;
    uint32_t  atMs_ = 0;
    uint32_t  rttMs_ = 0;
    uint8_t   stratum_ = 0;
    bool      kod_ = false;
};
