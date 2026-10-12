#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"
#include "event_groups.h"
#include "semphr.h"
#include "NetTypes.h"
#include "iNetDevice.h"

class xClient;

// The FreeRTOS half of a network interface, shared by xEthernet and
// xWifi: whatever the chip, and whatever bus it is on, this is how
// user threads reach it.
//
//   user threads                       driver thread (run())
//   ------------                       ---------------------
//   client.connect() ---- inbox ---->  device.connect()
//   client.write()   -> tx stream  ->  device takes bytes, sends
//   client.read()    <- rx stream  <-  device delivers bytes
//        ^ sleeps on the socket's event group until data, room,
//          connect or close — or its timeout
//
// ONE thread, the driver thread, owns the chip driver (iNetDevice) and
// is the only one that touches it. It sleeps on a FreeRTOS queue — the
// inbox — for as long as the driver's poll() says it can, and is woken
// early by anything put there: a request from a user thread, a "data
// waiting" kick from write(), or the chip's interrupt line.
//
// Each socket has:
//   - an rx and a tx stream buffer. Single writer, single reader, so no
//     lock is needed: the driver thread writes rx and reads tx, the
//     socket's owning user thread does the opposite.
//   - an event group the user thread sleeps on. The stream buffers
//     carry the bytes; the event group carries "something changed" —
//     which also covers what a stream buffer can't signal, like the
//     peer closing the connection while a reader is asleep.
//
// Setup:
//
//     xEthernet eth(chip);                   // or xWifi
//     eth.begin(netConfig);                  // before the driver thread starts
//     osThreadNew(netThread, &eth, &attrs);  // netThread calls eth.run()
//     // EXTI callback for the chip's INT pin, if wired:
//     //     eth.interruptFromIsr();
//
// Timeouts are in ms throughout.
class xNetInterface : private iNetDeviceHost {
public:
    struct Config {
        uint8_t maxSockets = 4;     // at most the device's socketCount() and kMaxSockets
        size_t  rxBufBytes = 1024;  // per socket, each direction; from the FreeRTOS heap
        size_t  txBufBytes = 1024;
        uint8_t inboxDepth = 8;
        // Time: the server asked when DHCP names none (option 42); a
        // name or "a.b.c.d", which must outlive the interface. nullptr:
        // no time unless DHCP gives a server.
        const char* ntpServer = "pool.ntp.org";
        // Re-synchronised this often once there is an address, the
        // first time at once; a failed attempt is tried again after a
        // minute. 0: only when syncTime() is called.
        uint32_t ntpIntervalMs = 3600000;
    };
    static constexpr uint8_t  kMaxSockets = 8;
    static constexpr uint32_t kForever = 0xFFFFFFFFu; // as a timeout: wait as long as it takes

    // ms to FreeRTOS ticks, kForever to portMAX_DELAY.
    static TickType_t toTicks(uint32_t ms);

    xNetInterface(const xNetInterface&) = delete;
    xNetInterface& operator=(const xNetInterface&) = delete;

    // Creates the queue, event groups and stream buffers, and asks the
    // driver thread to start the chip with this address. Call once,
    // before the driver thread runs. false: out of FreeRTOS heap.
    bool begin(const NetConfig& net);

    // Start the chip again with a new address. Every socket is lost.
    bool reconfigure(const NetConfig& net);

    // The driver thread's body. Never returns.
    void run();

    // One pass of run(): handle what is in the inbox, poll the device,
    // then sleep on the inbox for at most maxWaitMs. For a test, or to
    // run the interface from a thread that does other things too.
    void service(uint32_t maxWaitMs);

    // From the ISR (EXTI callback) for the chip's interrupt line.
    void interruptFromIsr();

    // Chip initialised. Before this, connect() waits (up to its timeout).
    bool ready() const;
    bool waitReady(uint32_t timeoutMs);

    // Ethernet: cable and PHY link. Wi-Fi: joined.
    bool linkUp() const;
    bool waitLinkUp(uint32_t timeoutMs);

    // An address is in use: the static one applied, or a DHCP lease.
    // Sockets can't connect before this.
    bool hasAddress() const;
    bool waitAddress(uint32_t timeoutMs);

    // The address in use (all zeros without one) — with DHCP, what the
    // server handed out, including gateway, DNS and NTP servers.
    NetConfig address() const;

    // Looks up host's IPv4 address with the interface's DNS server,
    // sleeping up to timeoutMs for the answer. "a.b.c.d" is answered at
    // once, without asking. false: no such name, no DNS server, no
    // answer in time. One lookup at a time: concurrent callers queue.
    bool resolve(const char* host, IpAddress& out, uint32_t timeoutMs);

    // Asks the time server now (see Config::ntpServer), sleeping up to
    // timeoutMs for the answer. Not needed with Config::ntpIntervalMs,
    // which keeps the time without being asked.
    bool syncTime(uint32_t timeoutMs);

    // The time has been received at least once.
    bool timeValid() const;

    // Unix time in ms (UTC), from the last answer plus the RTOS tick
    // since; 0 until timeValid(). Keep it re-synchronised (the default
    // ntpIntervalMs does) — the tick counter wraps after 49 days, and a
    // crystal drifts a few seconds a day.
    uint64_t unixTimeMs() const;

    uint8_t socketCount() const { return nSockets_; }

protected:
    xNetInterface(iNetDevice& dev, const Config& cfg);
    ~xNetInterface() override;

    enum class Op : uint8_t { Kick, Configure, Connect, Listen, Close, Join, Leave, Resolve, SyncTime };
    struct Msg {
        Op        op = Op::Kick;
        uint8_t   sock = 0;
        uint32_t  id = 0;       // Resolve/SyncTime: whose request
        uint16_t  port = 0;
        uint16_t  localPort = 0;
        IpAddress ip;
        NetConfig cfg;
    };

    // Puts a request in the inbox, waiting up to timeoutMs for room.
    bool post(const Msg& m, uint32_t timeoutMs);

    // Driver thread. Ops this class doesn't know (xWifi's Join/Leave).
    virtual void handleOther(const Msg& m) { (void)m; }

    // Interface-wide event bits.
    static constexpr EventBits_t kIfReady      = 1u << 0;
    static constexpr EventBits_t kIfFailed     = 1u << 1;
    static constexpr EventBits_t kIfLink       = 1u << 2;
    static constexpr EventBits_t kIfJoinFailed = 1u << 3;
    static constexpr EventBits_t kIfLinkDown   = 1u << 4; // kIfLink's opposite, to wait on
    static constexpr EventBits_t kIfAddress    = 1u << 5;
    static constexpr EventBits_t kIfResolved   = 1u << 6; // a resolve() answer is in
    static constexpr EventBits_t kIfTime       = 1u << 7; // a time answer is in
    EventGroupHandle_t events_ = nullptr;

    iNetDevice& dev_;

private:
    friend class xClient;

    // Per-socket event bits.
    static constexpr EventBits_t kEvAccepted  = 1u << 0; // driver thread has taken the connect/listen
    static constexpr EventBits_t kEvConnected = 1u << 1;
    static constexpr EventBits_t kEvClosed    = 1u << 2; // sticky until the next connect/listen
    static constexpr EventBits_t kEvFailed    = 1u << 3;
    static constexpr EventBits_t kEvRx        = 1u << 4; // bytes added to rx
    static constexpr EventBits_t kEvTx        = 1u << 5; // room made in tx
    static constexpr EventBits_t kEvListening = 1u << 6;
    static constexpr EventBits_t kEvAll       = 0x7F;

    struct Slot {
        StreamBufferHandle_t rx = nullptr;
        StreamBufferHandle_t tx = nullptr;
        EventGroupHandle_t   ev = nullptr;
        xClient*             owner = nullptr;
        std::atomic<bool>    rxStalled{false}; // driver found rx full; a reader should kick
    };

    // For xClient.
    int  claim(xClient* owner);  // a free socket, or -1
    void release(int s);
    Slot& slot(int s) { return slots_[s]; }
    void kick();                 // wake the driver thread: data to send, or room to receive

    void handle(const Msg& m);
    static uint32_t nowMs();

    // iNetDeviceHost, all on the driver thread but wakeFromIsr().
    void   wakeFromIsr() override;
    size_t rxSpace(uint8_t s) override;
    void   rxDeliver(uint8_t s, const uint8_t* data, size_t len) override;
    size_t txPending(uint8_t s) override;
    size_t txTake(uint8_t s, uint8_t* dst, size_t max) override;
    void   socketEvent(uint8_t s, SocketEvent ev) override;
    void   deviceEvent(DeviceEvent ev) override;
    void   addressChanged(const NetConfig& cfg) override;
    void   resolved(bool ok, const IpAddress& ip) override;
    void   timeReceived(bool ok, uint64_t unixMs, uint32_t atMs) override;

    void startTimeSync(uint32_t id);   // driver thread
    bool waitAnswer(EventBits_t bit, const uint32_t& answerId, uint32_t id, uint32_t timeoutMs);

    Config        cfg_;
    uint8_t       nSockets_ = 0;
    QueueHandle_t inbox_ = nullptr;
    Slot          slots_[kMaxSockets];
    NetConfig     addr_;                   // written by the driver thread, inside a critical section

    // resolve() and syncTime(): one caller at a time (requestMutex_).
    // Each request has an id; an answer counts for the caller whose id
    // the driver thread stamps on it, so an answer that comes after its
    // caller gave up can't be taken by the next one.
    SemaphoreHandle_t requestMutex_ = nullptr;
    uint32_t      requestSeq_ = 0;          // under requestMutex_
    static constexpr size_t kNameBuffer = 256;   // a DNS name (at most 253) and its NUL
    char          resolveName_[kNameBuffer] = {0};
    uint32_t      activeResolveId_ = 0;     // driver thread
    uint32_t      activeTimeId_ = 0;
    // Answers: written by the driver thread inside a critical section.
    uint32_t      resolvedId_ = 0;
    bool          resolvedOk_ = false;
    IpAddress     resolvedIp_;
    uint32_t      timeId_ = 0;
    bool          timeOk_ = false;
    bool          timeValid_ = false;
    uint64_t      syncUnixMs_ = 0;
    uint32_t      syncAtMs_ = 0;
    // Automatic re-synchronisation, driver thread only.
    bool          hasAddr_ = false;
    bool          syncInFlight_ = false;
    uint32_t      syncBase_ = 0;
    uint32_t      syncDelay_ = 0;
    char          ntpName_[16] = {0};       // DHCP's NTP server, as text

    std::atomic<bool> kickPending_{false}; // a Kick is in the inbox, or the driver is about to look anyway
    std::atomic<bool> irqPending_{false};
};
