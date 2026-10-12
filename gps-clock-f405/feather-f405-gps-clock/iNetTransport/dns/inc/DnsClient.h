#pragma once
#include <cstddef>
#include <cstdint>
#include "NetTypes.h"

// A DNS client (RFC 1035) for one A-record lookup at a time, as pure
// logic: it builds the query and reads the answer, and keeps the
// retry timer. It sends nothing itself and never blocks — the same
// shape as DhcpClient, for a chip driver's state machine to run:
//
//     dns.begin("pool.ntp.org", dnsServer, id, now);
//     ...
//     DnsClient::Action a = dns.poll(now, buf, sizeof buf);
//     if (a.len) send buf[0..a.len) to a.dst, port 53
//     on a datagram from port 53:  a = dns.receive(data, len, now)
//     a.event: Resolved (address()) or Failed (error())
//
// The answer must carry this query's ID and question, so a late reply
// to an earlier query is ignored. Names are looked up as given (no
// search domains); CNAME chains are followed by taking the first A
// record in the answer, which is how servers send them.
class DnsClient {
public:
    static constexpr uint16_t kServerPort = 53;
    static constexpr size_t   kMaxName = 253;
    static constexpr size_t   kMaxQuery = 12 + (kMaxName + 2) + 4;  // header, name, type and class

    enum class Event : uint8_t { None, Resolved, Failed };
    enum class Error : uint8_t {
        None,
        BadName,        // empty, too long, or an empty or over-long label
        NoServer,       // no DNS server to ask
        NotFound,       // the name doesn't exist, or has no IPv4 address
        ServerFailure,  // the server answered with an error
        Timeout,        // no answer after every retry
    };

    struct Action {
        size_t    len = 0;      // bytes to send to dst:53; 0 for none
        IpAddress dst;
        Event     event = Event::None;
    };

    // Starts a lookup, dropping any in progress. The name is copied.
    // false (and error() says why) if it can't start; no event follows.
    bool begin(const char* name, const IpAddress& server, uint16_t id, uint32_t nowMs);
    void cancel() { busy_ = false; }

    // Call regularly. buf: room for kMaxQuery bytes.
    Action poll(uint32_t nowMs, uint8_t* buf, size_t cap);

    // A datagram that came from the server's port 53.
    Action receive(const uint8_t* pkt, size_t len, uint32_t nowMs);

    // ms until poll() next has something to do.
    uint32_t nextWakeMs(uint32_t nowMs) const;

    bool busy() const { return busy_; }
    const IpAddress& address() const { return addr_; }
    Error error() const { return error_; }

private:
    size_t build(uint8_t* buf, size_t cap) const;
    bool   questionMatches(const uint8_t* pkt, size_t len, size_t& at) const;

    char      name_[kMaxName + 2] = {0};
    IpAddress server_;
    IpAddress addr_;
    uint16_t  id_ = 0;
    bool      busy_ = false;
    bool      txPending_ = false;
    uint8_t   tries_ = 0;
    uint32_t  nextTx_ = 0;
    uint32_t  txDelay_ = 0;
    Error     error_ = Error::None;
};
