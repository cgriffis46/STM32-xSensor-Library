#pragma once
#include <cstddef>
#include <cstdint>
#include "NetTypes.h"

// The seam between a network chip's driver and the interface built
// on it (xEthernet, xWifi). Nothing above this knows which chip it
// is, or what bus the chip is on:
//
//   W5500 over SPI          -> iBlockTransport  (iNetTransport/w5500)
//   ESP-AT Wi-Fi over UART  -> iTransport        (stream)
//   ATWINC1500 over SPI     -> iBlockTransport
//   a PHY on MII/RMII       -> the MCU's own Ethernet MAC + a host
//                              TCP/IP stack (lwIP) behind this same
//                              interface. Not on the STM32L432,
//                              which has no Ethernet MAC.
//
// These are socket-offload devices: the chip (or the stack behind the
// driver) does TCP, and this interface deals in sockets and bytes.
//
// Threading: every call in iNetDevice, and every callback into
// iNetDeviceHost, happens on ONE thread — the interface's driver
// thread. A device driver needs no locks of its own; the host's
// callbacks are where data crosses to user threads.
//
// Non-blocking, like the sensor drivers: poll() advances the driver's
// state machine as far as it can without waiting, and says how long
// it can be left before it needs calling again.

enum class SocketEvent : uint8_t {
    Listening,  // listen() is in place: the socket is waiting for a peer
    Connected,  // connect() completed, or a peer connected to a listen()ing socket
    Closed,     // the connection is over (either end closed it), or close() finished
    Failed,     // connect()/listen() didn't succeed, or the connection was lost
                // through an error (timeout, the chip reset). Implies Closed.
};

enum class DeviceEvent : uint8_t {
    Ready,      // initialised and configured; sockets can be opened
    Failed,     // gave up on the chip (no answer, wrong ID, a bus error).
                // Every socket is lost. The driver starts again by itself.
    LinkUp,     // Ethernet: cable and PHY link. Wi-Fi: joined, with an address.
    LinkDown,
    JoinFailed, // Wi-Fi: join() didn't succeed
};

// Implemented by the interface; called by the device driver, on the
// driver thread — except wakeFromIsr().
class iNetDeviceHost {
public:
    virtual ~iNetDeviceHost() = default;

    // The one call allowed from interrupt context: get poll() called
    // soon. For a driver whose bytes arrive in an ISR (a UART module)
    // and that has decided they are worth looking at.
    virtual void wakeFromIsr() = 0;

    // Room for received bytes on socket s right now. The driver never
    // takes more than this off the chip, and asks again later when it
    // was 0 — the host makes sure that's soon after room appears.
    virtual size_t rxSpace(uint8_t s) = 0;

    // Bytes received on socket s. len never exceeds the last rxSpace().
    virtual void rxDeliver(uint8_t s, const uint8_t* data, size_t len) = 0;

    // Bytes waiting to be sent on socket s.
    virtual size_t txPending(uint8_t s) = 0;

    // Takes up to max of them into dst. Returns how many it took.
    virtual size_t txTake(uint8_t s, uint8_t* dst, size_t max) = 0;

    virtual void socketEvent(uint8_t s, SocketEvent ev) = 0;
    virtual void deviceEvent(DeviceEvent ev) = 0;

    // The address in use changed: a static address applied, a DHCP
    // lease obtained or renewed with a different address, or (ip all
    // zeros) the address lost. Every open socket is lost with it.
    virtual void addressChanged(const NetConfig& cfg) = 0;

    // The answer to iNetDevice::resolve(): ok and the address, or not.
    virtual void resolved(bool ok, const IpAddress& ip) = 0;

    // The answer to iNetDevice::requestTime(): ok, and the time was
    // unixMs (UTC, ms since 1970) at atMs on the ms counter poll() is
    // given — or not.
    virtual void timeReceived(bool ok, uint64_t unixMs, uint32_t atMs) = 0;
};

// Implemented by a chip driver.
class iNetDevice {
public:
    virtual ~iNetDevice() = default;

    // Who to report to. Called once, before anything else.
    virtual void attach(iNetDeviceHost& host) = 0;

    // Sockets the interface may use, 0..socketCount()-1. A driver can
    // keep some of the chip's for itself (the W5500's DHCP socket).
    virtual uint8_t socketCount() const = 0;

    // Starts (or restarts) the chip with this address. Until the first
    // call the driver does nothing. Every open socket is lost.
    virtual void configure(const NetConfig& cfg) = 0;

    // Advances the state machine. Returns ms until it next needs
    // calling — 0 for "again now" — though calling it sooner is
    // harmless. nowMs: a free-running ms counter.
    virtual uint32_t poll(uint32_t nowMs) = 0;

    // Open socket s as a TCP client. Completes with Connected or
    // Failed. Held until the interface has an address. localPort: 0 lets the driver pick. Whatever socket s was
    // doing before is dropped without an event: from here on, events
    // on s are about this connection. false: s out of range, or the
    // driver can't take requests (not configured) — no event follows.
    virtual bool connect(uint8_t s, const IpAddress& ip, uint16_t port, uint16_t localPort) = 0;

    // Open socket s listening on port: Listening once it is (or Failed),
    // then Connected when a peer connects. Same rules as connect().
    virtual bool listen(uint8_t s, uint16_t port) = 0;

    // Close socket s, gracefully where it can. Always ends in a Closed
    // event, even if s already was.
    virtual void close(uint8_t s) = 0;

    // The chip's interrupt line fired. Called on the driver thread, not
    // from the ISR (the host relays it). A driver with no interrupt
    // line leaves this empty and polls.
    virtual void interrupt() = 0;

    // Looks up name's IPv4 address with the interface's DNS server;
    // the answer comes through iNetDeviceHost::resolved(). The name is
    // copied. A new call replaces one in progress, whose answer then
    // never comes. Held until the interface has an address. false: it
    // can't be asked now (not configured, name too long), and no answer
    // follows.
    virtual bool resolve(const char* name) = 0;

    // Asks a time server — a name or "a.b.c.d" — for the time (SNTP);
    // the answer comes through iNetDeviceHost::timeReceived(). Same
    // rules as resolve().
    virtual bool requestTime(const char* server) = 0;
};

// An Ethernet controller. Adds what only a wired PHY has.
class iEthernetDevice : public iNetDevice {
public:
    // As last read from the PHY; 0 while the link is down.
    virtual uint16_t speedMbps() const = 0;
    virtual bool     fullDuplex() const = 0;
};

// A Wi-Fi module. LinkUp/LinkDown report association.
class iWifiDevice : public iNetDevice {
public:
    // Join a network. Completes with LinkUp or JoinFailed. The driver
    // copies the strings before returning.
    virtual void join(const char* ssid, const char* passphrase) = 0;

    // Leave it. Every socket is lost; ends in LinkDown.
    virtual void leave() = 0;

    // Signal strength of the current network, dBm; 0 when not joined.
    virtual int8_t rssi() const = 0;
};
