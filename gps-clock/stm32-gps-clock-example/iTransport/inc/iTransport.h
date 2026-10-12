#pragma once
#include <cstdint>
#include <cstddef>

// Receives bytes pushed by a concrete iTransport's own ISR (or
// equivalent), one at a time. Implementations must be safe to call
// from whatever context the concrete transport calls this from —
// interrupt context for something like Stm32HalUartTransport (e.g.
// xStreamBufferSendFromISR() (FreeRTOS), or an atomic/lock-free ring
// buffer; nothing that can block, or that takes a mutex unsafely for
// ISR context).
//
// Deliberately separate from iTransport itself: where received bytes
// actually END UP (a FreeRTOS stream buffer, a POSIX pipe, whatever)
// is entirely the sink's business, not the transport's. This is what
// lets a concrete transport (Stm32HalUartTransport) stay completely
// RTOS-agnostic — it just calls onByteReceived() and doesn't know or
// care what happens after that.
class iTransportRxSink {
public:
    virtual ~iTransportRxSink() = default;
    virtual void onByteReceived(uint8_t byte) = 0;
};

// Generic interface for a stream-oriented transport — deliberately
// separate from ISensorTransport, which models register-addressed
// access (writeReg/readRegs). A stream transport has no concept of a
// "register": bytes just flow continuously in both directions,
// framed however whatever protocol runs over them decides (BNO085's
// SHTP-over-UART adds its own control-byte/escaping/CRC framing on
// top of this — nothing in this interface knows or cares about that;
// it's a layer above this one).
//
// Genuinely protocol-agnostic, not just in name: write()/setRxSink()
// say nothing about UART specifically, and this interface is meant to
// be the common base for whatever physical medium a concrete
// transport actually uses — UART (Stm32HalUartTransport), and,
// following the same shape, CAN (iTransportCan), Ethernet
// (iTransportEthernet), RS485, ASI, or anything else that's
// fundamentally "send some bytes, receive via a sink." This used to
// be named IUartTransport/IUartRxSink — renamed once it became clear
// non-UART transports were a real, near-term goal (SafeTransport<T>
// needing to back CAN/Ethernet/RS485/ASI), not just a naming
// preference.
//
// Receiving is push-based via iTransportRxSink above, not
// request/response or queue-pull — a concrete transport calls the
// attached sink directly from its own ISR (or equivalent) whenever a
// byte arrives.
//
// The sink is attached via setRxSink(), AFTER construction, not
// passed into the constructor. This matters, not just style: a sink
// is very often owned by something that itself needs a reference to
// the ALREADY-CONSTRUCTED transport (xBNO085 takes an iTransport& and
// is itself an iTransportRxSink — there's no way to hand the
// transport a not-yet-constructed xBNO085's reference at the
// transport's own construction time). setRxSink() breaks that
// chicken-and-egg deadlock: construct the transport with no sink,
// construct whatever owns the sink with a reference to the transport,
// then have IT call transport.setRxSink(*this) from its own
// constructor body — safe, since that's a normal call on an
// already-fully-constructed object, not a virtual dispatch on the
// object still under construction.
class iTransport {
public:
    virtual ~iTransport() = default;

    // Sends len bytes, non-blocking — returns true once the transfer
    // has been *issued*, not once it's landed (same convention as
    // ISensorTransport elsewhere in this codebase). Returns false if
    // a previous send is still in flight.
    //
    // Unlike ISensorTransport's writes, the bytes are NOT necessarily
    // copied: a transport may send straight from `data` after this
    // returns (Stm32HalUartTransport does). Keep `data` valid until
    // the send has finished — a member or static buffer, not a local.
    virtual bool write(const uint8_t* data, size_t len) = 0;

    // Attaches the sink that receives each byte as it arrives. A
    // concrete transport should treat "no sink attached yet" as
    // "silently drop received bytes," not as an error — bytes can
    // arrive before whatever owns the sink has finished wiring itself
    // up. sink must outlive this transport (or at least outlive
    // however long it stays attached).
    //
    // Also makes sure reception is running, where a transport has
    // something to start. So calling it again with the same sink is
    // harmless, and is how a driver that has heard nothing for too
    // long asks for a stalled receiver to be started again.
    virtual void setRxSink(iTransportRxSink& sink) = 0;
};
