#include "xNetInterface.h"
#include <cstring>
#include "task.h"

xNetInterface::xNetInterface(iNetDevice& dev, const Config& cfg)
    : dev_(dev), cfg_(cfg) {
    dev_.attach(*this);
}

xNetInterface::~xNetInterface() {
    for (Slot& k : slots_) {
        if (k.rx) vStreamBufferDelete(k.rx);
        if (k.tx) vStreamBufferDelete(k.tx);
        if (k.ev) vEventGroupDelete(k.ev);
    }
    if (events_) vEventGroupDelete(events_);
    if (requestMutex_) vSemaphoreDelete(requestMutex_);
    if (inbox_) vQueueDelete(inbox_);
}

TickType_t xNetInterface::toTicks(uint32_t ms) {
    return ms == kForever ? portMAX_DELAY : pdMS_TO_TICKS(ms);
}

uint32_t xNetInterface::nowMs() {
    return static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

bool xNetInterface::begin(const NetConfig& net) {
    if (inbox_ != nullptr) return reconfigure(net);

    nSockets_ = cfg_.maxSockets;
    if (nSockets_ > dev_.socketCount()) nSockets_ = dev_.socketCount();
    if (nSockets_ > kMaxSockets) nSockets_ = kMaxSockets;

    inbox_  = xQueueCreate(cfg_.inboxDepth, sizeof(Msg));
    events_ = xEventGroupCreate();
    requestMutex_ = xSemaphoreCreateMutex();
    if (inbox_ == nullptr || events_ == nullptr || requestMutex_ == nullptr) return false;
    xEventGroupSetBits(events_, kIfLinkDown);
    for (uint8_t s = 0; s < nSockets_; ++s) {
        Slot& k = slots_[s];
        k.rx = xStreamBufferCreate(cfg_.rxBufBytes, 1);
        k.tx = xStreamBufferCreate(cfg_.txBufBytes, 1);
        k.ev = xEventGroupCreate();
        if (k.rx == nullptr || k.tx == nullptr || k.ev == nullptr) return false;
        xEventGroupSetBits(k.ev, kEvClosed); // a socket nobody has opened is closed
    }
    return reconfigure(net);
}

bool xNetInterface::reconfigure(const NetConfig& net) {
    Msg m;
    m.op = Op::Configure;
    m.cfg = net;
    return post(m, 1000);
}

void xNetInterface::run() {
    configASSERT(inbox_ != nullptr); // begin() first
    for (;;) service(1000);
}

// The kick flag is cleared BEFORE the inbox is drained and the device
// polled. Data written after this point posts a fresh Kick, so the
// sleep at the end can't miss it; data written before it is seen by
// the poll.
void xNetInterface::service(uint32_t maxWaitMs) {
    kickPending_.store(false);

    Msg m;
    while (xQueueReceive(inbox_, &m, 0) == pdPASS) handle(m);
    if (irqPending_.exchange(false)) dev_.interrupt();

    // Keep the time: the first sync as soon as there is an address,
    // then every ntpIntervalMs.
    const bool autoSync = cfg_.ntpIntervalMs != 0 && hasAddr_ && !syncInFlight_;
    if (autoSync && (nowMs() - syncBase_) >= syncDelay_) startTimeSync(0);

    uint32_t wait = dev_.poll(nowMs());
    if (cfg_.ntpIntervalMs != 0 && hasAddr_ && !syncInFlight_) {
        const uint32_t gone = nowMs() - syncBase_;
        const uint32_t due = gone >= syncDelay_ ? 0 : syncDelay_ - gone;
        if (due < wait) wait = due;
    }
    if (wait > maxWaitMs) wait = maxWaitMs;
    if (wait == 0) return;

    TickType_t t = pdMS_TO_TICKS(wait);
    if (t == 0) t = 1; // a tick slower than 1 ms: still sleep, rather than spin
    if (xQueueReceive(inbox_, &m, t) == pdPASS) handle(m);
}

void xNetInterface::handle(const Msg& m) {
    switch (m.op) {
    case Op::Kick:
        break; // waking up was the point
    case Op::Configure:
        dev_.configure(m.cfg);
        break;
    case Op::Connect:
    case Op::Listen: {
        if (m.sock >= nSockets_) break;
        Slot& k = slots_[m.sock];
        // The owning user thread is waiting for kEvAccepted, not
        // touching the buffers, and nothing else uses them: safe to
        // empty. Any event the device raised about the socket's last
        // connection, before this, is wiped with them.
        xStreamBufferReset(k.rx);
        xStreamBufferReset(k.tx);
        k.rxStalled.store(false);
        xEventGroupClearBits(k.ev, kEvAll);
        const bool ok = (m.op == Op::Connect)
            ? dev_.connect(m.sock, m.ip, m.port, m.localPort)
            : dev_.listen(m.sock, m.localPort);
        if (!ok) socketEvent(m.sock, SocketEvent::Failed);
        // Last, so a refusal is already showing when the client wakes.
        xEventGroupSetBits(k.ev, kEvAccepted);
        break;
    }
    case Op::Close:
        if (m.sock < nSockets_) dev_.close(m.sock);
        break;
    case Op::Resolve:
        // resolveName_ is the caller's, who holds requestMutex_ and is
        // waiting: it can't change under us. The device copies it.
        activeResolveId_ = m.id;
        if (!dev_.resolve(resolveName_)) resolved(false, IpAddress());
        break;
    case Op::SyncTime:
        startTimeSync(m.id);
        break;
    default:
        handleOther(m);
        break;
    }
}

bool xNetInterface::post(const Msg& m, uint32_t timeoutMs) {
    if (inbox_ == nullptr) return false;
    return xQueueSend(inbox_, &m, toTicks(timeoutMs)) == pdPASS;
}

void xNetInterface::kick() {
    if (kickPending_.exchange(true)) return; // one is already on its way
    Msg m;
    // Full inbox: the driver thread has plenty to wake it already, and
    // clears kickPending_ when it does.
    (void)xQueueSend(inbox_, &m, 0);
}

void xNetInterface::interruptFromIsr() {
    if (inbox_ == nullptr) return;
    irqPending_.store(true);
    if (kickPending_.exchange(true)) return;
    Msg m;
    BaseType_t woken = pdFALSE;
    (void)xQueueSendFromISR(inbox_, &m, &woken);
    portYIELD_FROM_ISR(woken);
}

// As interruptFromIsr(), without telling the device anything: it
// asked for this itself.
void xNetInterface::wakeFromIsr() {
    if (inbox_ == nullptr || kickPending_.exchange(true)) return;
    Msg m;
    BaseType_t woken = pdFALSE;
    (void)xQueueSendFromISR(inbox_, &m, &woken);
    portYIELD_FROM_ISR(woken);
}

bool xNetInterface::ready() const {
    return events_ != nullptr && (xEventGroupGetBits(events_) & kIfReady) != 0;
}

bool xNetInterface::waitReady(uint32_t timeoutMs) {
    if (events_ == nullptr) return false;
    return (xEventGroupWaitBits(events_, kIfReady, pdFALSE, pdFALSE, toTicks(timeoutMs)) & kIfReady) != 0;
}

bool xNetInterface::linkUp() const {
    return events_ != nullptr && (xEventGroupGetBits(events_) & kIfLink) != 0;
}

bool xNetInterface::waitLinkUp(uint32_t timeoutMs) {
    if (events_ == nullptr) return false;
    return (xEventGroupWaitBits(events_, kIfLink, pdFALSE, pdFALSE, toTicks(timeoutMs)) & kIfLink) != 0;
}

bool xNetInterface::hasAddress() const {
    return events_ != nullptr && (xEventGroupGetBits(events_) & kIfAddress) != 0;
}

bool xNetInterface::waitAddress(uint32_t timeoutMs) {
    if (events_ == nullptr) return false;
    return (xEventGroupWaitBits(events_, kIfAddress, pdFALSE, pdFALSE, toTicks(timeoutMs)) & kIfAddress) != 0;
}

NetConfig xNetInterface::address() const {
    taskENTER_CRITICAL();
    const NetConfig a = addr_;
    taskEXIT_CRITICAL();
    return a;
}

// ---- DNS and time ----

// Waits for an answer stamped with this request's id. One stamped with
// another (an earlier caller's, arriving late) is passed over.
bool xNetInterface::waitAnswer(EventBits_t bit, const uint32_t& answerId, uint32_t id, uint32_t timeoutMs) {
    const TickType_t start = xTaskGetTickCount();
    const TickType_t total = toTicks(timeoutMs);
    for (;;) {
        TickType_t left = portMAX_DELAY;
        if (total != portMAX_DELAY) {
            const TickType_t gone = xTaskGetTickCount() - start;
            left = gone >= total ? 0 : total - gone;
        }
        const EventBits_t b = xEventGroupWaitBits(events_, bit, pdTRUE, pdFALSE, left);
        if (b & bit) {
            taskENTER_CRITICAL();
            const bool mine = answerId == id;
            taskEXIT_CRITICAL();
            if (mine) return true;
        }
        if (left == 0) return false;
    }
}

bool xNetInterface::resolve(const char* host, IpAddress& out, uint32_t timeoutMs) {
    if (IpAddress::parse(host, out)) return true;   // nothing to look up
    if (host == nullptr || events_ == nullptr || std::strlen(host) >= kNameBuffer) return false;
    if (xSemaphoreTake(requestMutex_, toTicks(timeoutMs)) != pdPASS) return false;

    std::strcpy(resolveName_, host);
    Msg m;
    m.op = Op::Resolve;
    m.id = ++requestSeq_;
    xEventGroupClearBits(events_, kIfResolved);
    bool ok = post(m, timeoutMs) && waitAnswer(kIfResolved, resolvedId_, m.id, timeoutMs);
    if (ok) {
        taskENTER_CRITICAL();
        ok = resolvedOk_;
        if (ok) out = resolvedIp_;
        taskEXIT_CRITICAL();
    }
    xSemaphoreGive(requestMutex_);
    return ok;
}

bool xNetInterface::syncTime(uint32_t timeoutMs) {
    if (events_ == nullptr) return false;
    if (xSemaphoreTake(requestMutex_, toTicks(timeoutMs)) != pdPASS) return false;
    Msg m;
    m.op = Op::SyncTime;
    m.id = ++requestSeq_;
    xEventGroupClearBits(events_, kIfTime);
    bool ok = post(m, timeoutMs) && waitAnswer(kIfTime, timeId_, m.id, timeoutMs);
    if (ok) {
        taskENTER_CRITICAL();
        ok = timeOk_;
        taskEXIT_CRITICAL();
    }
    xSemaphoreGive(requestMutex_);
    return ok;
}

bool xNetInterface::timeValid() const {
    taskENTER_CRITICAL();
    const bool v = timeValid_;
    taskEXIT_CRITICAL();
    return v;
}

uint64_t xNetInterface::unixTimeMs() const {
    taskENTER_CRITICAL();
    const bool valid = timeValid_;
    const uint64_t base = syncUnixMs_;
    const uint32_t at = syncAtMs_;
    taskEXIT_CRITICAL();
    return valid ? base + (nowMs() - at) : 0;
}

// Driver thread. Which server: DHCP's, else the configured one. A
// request already in progress (an automatic one, say) is replaced, and
// its answer is this one's.
void xNetInterface::startTimeSync(uint32_t id) {
    activeTimeId_ = id;
    syncInFlight_ = true;
    const char* server = cfg_.ntpServer;
    if (!addr_.ntp.isZero()) server = addr_.ntp.format(ntpName_);
    if (server == nullptr || *server == 0 || !dev_.requestTime(server)) timeReceived(false, 0, nowMs());
}

// ---- sockets, for xClient ----

int xNetInterface::claim(xClient* owner) {
    int found = -1;
    taskENTER_CRITICAL();
    for (uint8_t s = 0; s < nSockets_; ++s) {
        if (slots_[s].owner == nullptr) {
            slots_[s].owner = owner;
            found = s;
            break;
        }
    }
    taskEXIT_CRITICAL();
    return found;
}

void xNetInterface::release(int s) {
    taskENTER_CRITICAL();
    slots_[s].owner = nullptr;
    taskEXIT_CRITICAL();
}

// ---- iNetDeviceHost: the driver thread ----

// The stall flag goes up before the space is read, so a reader that
// makes room in between sees it and kicks; see xClient::read().
size_t xNetInterface::rxSpace(uint8_t s) {
    if (s >= nSockets_) return 0;
    Slot& k = slots_[s];
    k.rxStalled.store(true);
    const size_t n = xStreamBufferSpacesAvailable(k.rx);
    if (n != 0) k.rxStalled.store(false);
    return n;
}

void xNetInterface::rxDeliver(uint8_t s, const uint8_t* data, size_t len) {
    if (s >= nSockets_) return;
    Slot& k = slots_[s];
    xStreamBufferSend(k.rx, data, len, 0); // fits: len <= rxSpace()
    xEventGroupSetBits(k.ev, kEvRx);
}

size_t xNetInterface::txPending(uint8_t s) {
    if (s >= nSockets_) return 0;
    return xStreamBufferBytesAvailable(slots_[s].tx);
}

size_t xNetInterface::txTake(uint8_t s, uint8_t* dst, size_t max) {
    if (s >= nSockets_) return 0;
    Slot& k = slots_[s];
    const size_t n = xStreamBufferReceive(k.tx, dst, max, 0);
    if (n != 0) xEventGroupSetBits(k.ev, kEvTx);
    return n;
}

void xNetInterface::socketEvent(uint8_t s, SocketEvent ev) {
    if (s >= nSockets_) return;
    Slot& k = slots_[s];
    switch (ev) {
    case SocketEvent::Listening:
        xEventGroupSetBits(k.ev, kEvListening);
        break;
    case SocketEvent::Connected:
        xEventGroupSetBits(k.ev, kEvConnected);
        break;
    case SocketEvent::Closed:
        xEventGroupClearBits(k.ev, kEvConnected);
        xEventGroupSetBits(k.ev, kEvClosed);
        break;
    case SocketEvent::Failed:
        xEventGroupClearBits(k.ev, kEvConnected);
        xEventGroupSetBits(k.ev, kEvFailed | kEvClosed);
        break;
    }
}

void xNetInterface::deviceEvent(DeviceEvent ev) {
    switch (ev) {
    case DeviceEvent::Ready:
        xEventGroupClearBits(events_, kIfFailed);
        xEventGroupSetBits(events_, kIfReady);
        break;
    case DeviceEvent::Failed:
        xEventGroupClearBits(events_, kIfReady);
        xEventGroupSetBits(events_, kIfFailed);
        break;
    case DeviceEvent::LinkUp:
        xEventGroupClearBits(events_, kIfJoinFailed | kIfLinkDown);
        xEventGroupSetBits(events_, kIfLink);
        break;
    case DeviceEvent::LinkDown:
        xEventGroupClearBits(events_, kIfLink);
        xEventGroupSetBits(events_, kIfLinkDown);
        break;
    case DeviceEvent::JoinFailed:
        xEventGroupSetBits(events_, kIfJoinFailed);
        break;
    }
}

void xNetInterface::addressChanged(const NetConfig& cfg) {
    taskENTER_CRITICAL();
    addr_ = cfg;
    taskEXIT_CRITICAL();
    // A new address (perhaps a new network): set the time straight away.
    hasAddr_ = !cfg.ip.isZero();
    syncBase_ = nowMs();
    syncDelay_ = 0;
    if (cfg.ip.isZero()) {
        xEventGroupClearBits(events_, kIfAddress);
    } else {
        xEventGroupSetBits(events_, kIfAddress);
    }
}

void xNetInterface::resolved(bool ok, const IpAddress& ip) {
    taskENTER_CRITICAL();
    resolvedId_ = activeResolveId_;
    resolvedOk_ = ok;
    resolvedIp_ = ip;
    taskEXIT_CRITICAL();
    xEventGroupSetBits(events_, kIfResolved);
}

void xNetInterface::timeReceived(bool ok, uint64_t unixMs, uint32_t atMs) {
    taskENTER_CRITICAL();
    timeId_ = activeTimeId_;
    timeOk_ = ok;
    if (ok) {
        timeValid_ = true;
        syncUnixMs_ = unixMs;
        syncAtMs_ = atMs;
    }
    taskEXIT_CRITICAL();
    // The next automatic sync: a full interval after a good one, a
    // minute (or the interval, if shorter) after a failure.
    syncInFlight_ = false;
    syncBase_ = nowMs();
    syncDelay_ = ok ? cfg_.ntpIntervalMs : (cfg_.ntpIntervalMs < 60000 ? cfg_.ntpIntervalMs : 60000);
    xEventGroupSetBits(events_, kIfTime);
}
