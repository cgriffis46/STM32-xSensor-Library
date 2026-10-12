#pragma once
#include <cstddef>
#include <cstdint>
#include "xNetInterface.h"

// A TCP connection on an xEthernet or xWifi, for use from any user
// thread. Every call that can wait takes a timeout in ms and puts the
// calling thread to sleep — on the socket's event group — until it has
// something to return or the timeout runs out. Nothing here polls.
//
//     xClient client(eth);
//     if (client.connect(IpAddress(192, 168, 1, 10), 80, 5000)) {
//         client.write(req, sizeof req, 1000);
//         uint8_t buf[256];
//         int32_t n = client.read(buf, sizeof buf, 2000); // sleeps until data
//         ...
//         client.stop();
//     }
//
// One thread per client: the stream buffers behind it allow one
// reader and one writer, and the driver thread is the other end of
// each. Two threads sharing one client need their own lock.
//
// A client takes one of the interface's sockets in connect()/listen()
// and holds it until stop() (or its destructor).
class xClient {
public:
    explicit xClient(xNetInterface& net) : net_(net) {}
    ~xClient() { stop(); }

    xClient(const xClient&) = delete;
    xClient& operator=(const xClient&) = delete;

    // Connects, waiting up to timeoutMs. false: no free socket, the
    // peer refused or never answered, or the timeout ran out. Calling
    // it on a client that is already connected drops that connection.
    bool connect(const IpAddress& ip, uint16_t port, uint32_t timeoutMs);

    // Starts listening on port, returning once the chip is listening
    // (not when a peer arrives — that's accept()). One connection per
    // client: to serve several at once, have several clients listen
    // on the same port.
    bool listen(uint16_t port, uint32_t timeoutMs = 1000);

    // After listen(): waits for a peer to connect.
    bool accept(uint32_t timeoutMs);

    // Up to len received bytes into buf, sleeping up to timeoutMs for
    // the first to arrive. Returns how many (> 0); 0 if the timeout ran
    // out with nothing received; -1 once the connection is closed and
    // everything it received has been read (or if never connected).
    int32_t read(uint8_t* buf, size_t len, uint32_t timeoutMs);

    // Queues len bytes to send, sleeping up to timeoutMs for room when
    // the socket's buffer is full. Returns how many were queued — less
    // than len if the timeout ran out — or -1 if not connected. The
    // bytes are copied: buf is free as soon as this returns.
    int32_t write(const uint8_t* buf, size_t len, uint32_t timeoutMs);

    // Waits until everything written has been handed to the chip.
    bool flush(uint32_t timeoutMs);

    // Received bytes ready to read now.
    size_t available() const;

    // Connected, and not yet closed by either end. Data received
    // before a close can still be read after this turns false.
    bool connected() const;

    // Closes the connection, waiting up to timeoutMs for the close to
    // finish, and gives the socket back. Unread data is discarded.
    void stop(uint32_t timeoutMs = 1000);

    // The interface socket this client holds, or -1.
    int socket() const { return s_; }

private:
    bool claim();
    bool open(const xNetInterface::Msg& m, uint32_t timeoutMs, EventBits_t success);
    void abandon();
    static TickType_t left(TickType_t start, TickType_t total);

    xNetInterface& net_;
    int s_ = -1;
};
