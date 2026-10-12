#include "xClient.h"
#include "task.h"

typedef xNetInterface Net;

// What is left of `total` ticks counted from `start`.
TickType_t xClient::left(TickType_t start, TickType_t total) {
    if (total == portMAX_DELAY) return portMAX_DELAY;
    const TickType_t gone = xTaskGetTickCount() - start;
    return gone >= total ? 0 : total - gone;
}

bool xClient::claim() {
    if (s_ >= 0) return true; // reuse the socket we hold
    s_ = net_.claim(this);
    return s_ >= 0;
}

bool xClient::connect(const IpAddress& ip, uint16_t port, uint32_t timeoutMs) {
    if (!claim()) return false;
    Net::Msg m;
    m.op = Net::Op::Connect;
    m.sock = static_cast<uint8_t>(s_);
    m.ip = ip;
    m.port = port;
    return open(m, timeoutMs, Net::kEvConnected);
}

bool xClient::listen(uint16_t port, uint32_t timeoutMs) {
    if (!claim()) return false;
    Net::Msg m;
    m.op = Net::Op::Listen;
    m.sock = static_cast<uint8_t>(s_);
    m.localPort = port;
    return open(m, timeoutMs, Net::kEvListening);
}

// Two waits. First for the driver thread to take the request
// (kEvAccepted): it clears the socket's bits as it does, so an event
// left over from the socket's previous connection can't be mistaken
// for this one's. Then for the outcome: `success` (connected, or
// listening) or closed.
bool xClient::open(const Net::Msg& m, uint32_t timeoutMs, EventBits_t success) {
    Net::Slot& k = net_.slot(s_);
    const TickType_t start = xTaskGetTickCount();
    const TickType_t total = Net::toTicks(timeoutMs);

    xEventGroupClearBits(k.ev, Net::kEvAccepted);
    if (!net_.post(m, timeoutMs)) return false;
    EventBits_t b = xEventGroupWaitBits(k.ev, Net::kEvAccepted, pdTRUE, pdFALSE, left(start, total));
    if ((b & Net::kEvAccepted) == 0) {
        abandon();
        return false;
    }
    b = xEventGroupWaitBits(k.ev, success | Net::kEvClosed, pdFALSE, pdFALSE, left(start, total));
    if ((b & success) != 0 && (b & Net::kEvClosed) == 0) return true;
    if ((b & Net::kEvClosed) == 0) abandon(); // still trying: tell the driver to give up
    return false;
}

void xClient::abandon() {
    Net::Msg m;
    m.op = Net::Op::Close;
    m.sock = static_cast<uint8_t>(s_);
    net_.post(m, 100);
}

bool xClient::accept(uint32_t timeoutMs) {
    if (s_ < 0) return false;
    Net::Slot& k = net_.slot(s_);
    const EventBits_t b = xEventGroupWaitBits(k.ev, Net::kEvConnected | Net::kEvClosed, pdFALSE, pdFALSE,
                                              Net::toTicks(timeoutMs));
    return (b & Net::kEvConnected) != 0 && (b & Net::kEvClosed) == 0;
}

// kEvRx is cleared BEFORE the stream buffer is looked at: bytes that
// land after the look set it again, so the wait can't sleep through
// them.
int32_t xClient::read(uint8_t* buf, size_t len, uint32_t timeoutMs) {
    if (s_ < 0) return -1;
    if (buf == nullptr || len == 0) return 0;
    Net::Slot& k = net_.slot(s_);
    const TickType_t start = xTaskGetTickCount();
    const TickType_t total = Net::toTicks(timeoutMs);

    for (;;) {
        xEventGroupClearBits(k.ev, Net::kEvRx);
        size_t n = xStreamBufferReceive(k.rx, buf, len, 0);
        if (n == 0 && (xEventGroupGetBits(k.ev) & Net::kEvClosed) != 0) {
            // The driver delivers everything before it reports Closed,
            // but it may have done both since the look above.
            n = xStreamBufferReceive(k.rx, buf, len, 0);
            if (n == 0) return -1;
        }
        if (n != 0) {
            // The driver stopped taking bytes off the chip because this
            // buffer was full. There's room now: wake it.
            if (k.rxStalled.exchange(false)) net_.kick();
            return static_cast<int32_t>(n);
        }
        const TickType_t w = left(start, total);
        if (w == 0) return 0;
        xEventGroupWaitBits(k.ev, Net::kEvRx | Net::kEvClosed, pdFALSE, pdFALSE, w);
    }
}

int32_t xClient::write(const uint8_t* buf, size_t len, uint32_t timeoutMs) {
    if (s_ < 0) return -1;
    if (buf == nullptr || len == 0) return 0;
    Net::Slot& k = net_.slot(s_);
    const TickType_t start = xTaskGetTickCount();
    const TickType_t total = Net::toTicks(timeoutMs);
    size_t sent = 0;

    for (;;) {
        xEventGroupClearBits(k.ev, Net::kEvTx);
        const EventBits_t b = xEventGroupGetBits(k.ev);
        if ((b & Net::kEvConnected) == 0 || (b & Net::kEvClosed) != 0) {
            return sent != 0 ? static_cast<int32_t>(sent) : -1;
        }
        const size_t n = xStreamBufferSend(k.tx, buf + sent, len - sent, 0);
        if (n != 0) {
            sent += n;
            net_.kick();
        }
        if (sent == len) return static_cast<int32_t>(sent);
        const TickType_t w = left(start, total);
        if (w == 0) return static_cast<int32_t>(sent);
        xEventGroupWaitBits(k.ev, Net::kEvTx | Net::kEvClosed, pdFALSE, pdFALSE, w);
    }
}

bool xClient::flush(uint32_t timeoutMs) {
    if (s_ < 0) return false;
    Net::Slot& k = net_.slot(s_);
    const TickType_t start = xTaskGetTickCount();
    const TickType_t total = Net::toTicks(timeoutMs);
    for (;;) {
        xEventGroupClearBits(k.ev, Net::kEvTx);
        if (xStreamBufferIsEmpty(k.tx) == pdTRUE) return true;
        if ((xEventGroupGetBits(k.ev) & Net::kEvClosed) != 0) return false;
        const TickType_t w = left(start, total);
        if (w == 0) return false;
        xEventGroupWaitBits(k.ev, Net::kEvTx | Net::kEvClosed, pdFALSE, pdFALSE, w);
    }
}

size_t xClient::available() const {
    if (s_ < 0) return 0;
    return xStreamBufferBytesAvailable(net_.slot(s_).rx);
}

bool xClient::connected() const {
    if (s_ < 0) return false;
    const EventBits_t b = xEventGroupGetBits(net_.slot(s_).ev);
    return (b & Net::kEvConnected) != 0 && (b & Net::kEvClosed) == 0;
}

// The Close request is always sent, even if the socket already shows
// Closed (the peer closed it): the device then makes sure the chip's
// socket is closed too, and answers Closed again. A close that hasn't
// finished by the timeout is left to the device, which forces it, and
// a later connect() on the same socket replaces it anyway.
void xClient::stop(uint32_t timeoutMs) {
    if (s_ < 0) return;
    Net::Slot& k = net_.slot(s_);
    Net::Msg m;
    m.op = Net::Op::Close;
    m.sock = static_cast<uint8_t>(s_);
    if (net_.post(m, timeoutMs)) {
        xEventGroupWaitBits(k.ev, Net::kEvClosed, pdFALSE, pdFALSE, Net::toTicks(timeoutMs));
    }
    net_.release(s_);
    s_ = -1;
}
