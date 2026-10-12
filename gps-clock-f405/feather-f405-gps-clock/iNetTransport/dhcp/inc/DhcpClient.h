#pragma once
#include <cstddef>
#include <cstdint>
#include "NetTypes.h"

// A DHCP client (RFC 2131) as pure logic: it builds the packets to send
// and reads the ones received, and keeps the lease timers. It sends
// nothing itself and never blocks, so any chip driver that can move a
// UDP datagram can use it from its own state machine:
//
//     dhcp.begin(mac, seed, now);
//     ...
//     DhcpClient::Action a = dhcp.poll(now, buf, sizeof buf);
//     if (a.len)  send buf[0..a.len) from port 68 to a.dst, port 67
//     if (a.event == DhcpClient::Event::Bound) apply dhcp.lease()
//     ...
//     on a datagram to port 68:  a = dhcp.receive(data, len, now)
//
// Times are ms from a free-running counter. Leases are capped at 24
// days so every deadline fits that counter's wrap.
//
// Not done: ARP probing of the offered address (DHCPDECLINE), and
// options that overflow into the sname/file fields.
class DhcpClient {
public:
    static constexpr uint16_t kClientPort = 68;
    static constexpr uint16_t kServerPort = 67;
    static constexpr size_t   kMaxPacket  = 576;  // the most a client must accept
    static constexpr size_t   kTxPacket   = 300;  // what we send: BOOTP's minimum size

    enum class State : uint8_t {
        Stopped,
        Selecting,   // DISCOVER sent, waiting for an OFFER
        Requesting,  // REQUEST sent for an offer, waiting for an ACK
        Bound,
        Renewing,    // past T1: REQUEST unicast to the server that gave the lease
        Rebinding,   // past T2, or the link came back: REQUEST broadcast to any server
    };

    enum class Event : uint8_t {
        None,
        Bound,  // an address is now (or still) ours: apply lease()
        Lost,   // the address is gone (lease expired, or NAKed): stop using it
    };

    struct Action {
        size_t    len = 0;       // bytes to send, from port 68 to dst:67; 0 for none
        IpAddress dst;
        Event     event = Event::None;
    };

    // Starts from scratch: DISCOVER on the next poll(). seed makes the
    // transaction IDs differ between devices and boots (mix in a
    // hardware unique ID or a timer). previous: an address to ask for
    // again, e.g. the last lease (all zeros for none).
    void begin(const MacAddress& mac, uint32_t seed, uint32_t nowMs,
               const IpAddress& previous = IpAddress());
    void stop() { state_ = State::Stopped; }

    // The link came back after being down: confirm the lease is still
    // good here (REQUEST broadcast) rather than wait for T1.
    void linkRestored(uint32_t nowMs);

    // Call regularly. buf: room for kTxPacket bytes.
    Action poll(uint32_t nowMs, uint8_t* buf, size_t cap);

    // A datagram that arrived on port 68.
    Action receive(const uint8_t* pkt, size_t len, uint32_t nowMs);

    // ms until poll() next has something to do.
    uint32_t nextWakeMs(uint32_t nowMs) const;

    State state() const { return state_; }
    bool  bound() const { return state_ == State::Bound || state_ == State::Renewing || state_ == State::Rebinding; }

    // Valid while bound(): ip, subnet, gateway, dns, ntp (and the mac
    // given to begin()).
    const NetConfig& lease() const { return lease_; }
    uint32_t leaseSeconds() const { return leaseSec_; }
    const IpAddress& server() const { return server_; }

private:
    enum MsgType : uint8_t { Discover = 1, Offer = 2, Request = 3, Decline = 4, Ack = 5, Nak = 6, Release = 7 };

    size_t build(uint8_t* buf, size_t cap, uint8_t type) const;
    void   restart(uint32_t nowMs);
    void   sendNow(uint32_t nowMs) { nextTx_ = nowMs; txDelay_ = 0; txPending_ = true; }
    void   scheduleRetry(uint32_t nowMs);

    static bool after(uint32_t now, uint32_t t0, uint32_t ms) { return (now - t0) >= ms; }

    State      state_ = State::Stopped;
    MacAddress mac_;
    uint32_t   xid_ = 0;
    uint32_t   seed_ = 0;
    IpAddress  requested_;     // in REQUEST option 50 / DISCOVER hint
    IpAddress  server_;        // option 54 of the offer we took
    NetConfig  lease_;
    NetConfig  offer_;
    uint32_t   leaseSec_ = 0;

    // When the next transmission is due: txPending_ and nextTx_ (a
    // deadline as since + delay, kept as base and delay so wrap is safe).
    bool       txPending_ = false;
    uint32_t   nextTx_ = 0;      // base time
    uint32_t   txDelay_ = 0;     // ms after nextTx_
    uint32_t   retryMs_ = 0;     // current backoff
    uint8_t    tries_ = 0;

    uint32_t   boundAt_ = 0;     // ms when the current lease was granted
    uint32_t   t1Ms_ = 0, t2Ms_ = 0, leaseMs_ = 0;  // from boundAt_; leaseMs_ 0xFFFFFFFF: infinite
};
