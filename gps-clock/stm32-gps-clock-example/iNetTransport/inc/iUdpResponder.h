#pragma once
#include <cstddef>
#include <cstdint>
#include "NetTypes.h"

// A small UDP server that answers each request with at most one reply,
// in the driver thread, as the request is read off the chip: an NTP
// server, a discovery or status responder. The chip driver owns the
// socket (W5500: w5500_param_t::responder / responderPort) and sends the
// reply back to where the request came from.
//
// Answering in the driver thread, rather than handing datagrams to an
// application thread, is what an NTP server wants: the receive time is
// taken the moment the request is read, and the reply goes out within a
// few SPI transfers. The price is that respond() must be short and must
// not block: no locks that another thread may hold for long, no waits.
class iUdpResponder {
public:
    virtual ~iUdpResponder() = default;

    // A request of len bytes from `from`:`fromPort`, in buf. Write the
    // reply into buf (it may overwrite the request; at most cap bytes)
    // and return its length, or 0 to send nothing. nowMs is the driver's
    // clock when the request was read. Driver thread.
    virtual size_t respond(const IpAddress& from, uint16_t fromPort, uint8_t* buf, size_t len,
                           size_t cap, uint32_t nowMs) = 0;
};
