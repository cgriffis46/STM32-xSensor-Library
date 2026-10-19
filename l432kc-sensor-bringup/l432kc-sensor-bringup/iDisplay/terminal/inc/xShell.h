#pragma once
#include <atomic>
#include <stddef.h>
#include <stdint.h>
#include "iTransport.h"
#include "xHmi.h"
#include "xLogLines.h"

namespace idisplay {

// A command line on a serial terminal, for maintenance: the HMI's tags
// read and its editable fields set by typing, and the log.
//
//   help                 the commands
//   tags                 every tag the HMI shows, its value, and for an
//                        editable field what it may be set to
//   get <tag>            one tag (any the source has)
//   set <tag> <value>    an editable field: a number, on/off text (or 1/0)
//                        for a state, a choice's label (or value)
//   pin <dddd>           unlocks a level, as on the panel
//   lock                 locks again
//   log [n]              the last n lines of the log (10)
//
// set keeps the panel's rules (xHmi::fieldAccepts): only fields the
// layout marks editable, within min..max in whole steps and no more
// decimals than the field shows, a level's PIN first. Three wrong PINs
// lock the shell out for 60 s; an unlock ends unlockTimeoutMs after the
// last command. The shell has its own unlock and lockout, apart from the
// panel's; the PINs are the panel's (xHmi::setPin). Every set goes to the
// panel's audit callback (xHmi::setAudit), from the shell's thread.
//
//   static xShell shell(shellUart, hmi, &logLines, "plant");
//   shell.begin();                                  // attaches to the UART
//   for (;;) { shell.poll(osKernelGetTickCount()); osDelay(10); }    // its own thread
//
// Threads: bytes come in from the UART's interrupt into a 64 byte ring;
// everything else happens in poll(). The shell reads and writes tags from
// its thread while the GUI task reads them from its own, so the tag
// source must allow that (PLCTransport's PlcTagSource does: the registry
// has a lock; give xTagTable setLock()). The HMI's layout and PINs are
// only read (set the PINs before starting either).
//
// Output goes through a 512 byte ring and two 128 byte buffers in turn;
// long listings are written a line at a time as room frees. What doesn't
// fit (a line typed while a listing is going out) is dropped and counted.
// RAM: about 1 KB.
class xShell : public iTransportRxSink {
public:
    static constexpr size_t kLine = 64;
    static constexpr size_t kRx = 64;
    static constexpr size_t kOut = 512;
    static constexpr size_t kTx = 128;
    static constexpr uint8_t kLogDefault = 10;

    xShell(iTransport& io, xHmi& hmi, const xLogLinesBase* log = nullptr, const char* name = nullptr)
        : io_(io), hmi_(hmi), log_(log), name_(name) {}

    // Attaches to the transport and prints the greeting and a prompt.
    void begin();

    // From the transport (often its interrupt).
    void onByteReceived(uint8_t byte) override;

    // Handles what came in and sends what is waiting. Call it every few
    // ms from the shell's thread.
    void poll(uint32_t nowMs);

    xHmiLevel unlocked() const { return unlocked_; }
    bool lockedOut() const { return lockout_; }
    uint32_t dropped() const { return dropped_; }
    uint32_t rxDropped() const { return rxDropped_.load(std::memory_order_relaxed); }

    // Runs one line as if typed (no echo). For tests.
    void execute(char* line, uint32_t nowMs);

private:
    enum class Listing : uint8_t { None, Help, Tags, Log };

    void input(uint8_t b, uint32_t nowMs);
    void prompt();
    void emit(const char* s);
    void emitLine(const char* a, const char* b = "", const char* c = "", const char* d = "");
    size_t room() const { return kOut - outCount_; }
    void send();
    void listMore();
    bool tagLine(uint16_t tag, char* out, size_t cap);
    int32_t tagIndex(const char* name) const;
    const xHmiItem* shownBy(uint16_t tag) const;
    const xHmiItem* field(uint16_t tag) const;
    void get(const char* name);
    void set(const char* name, const char* value, uint32_t nowMs);
    void pin(const char* digits, uint32_t nowMs);
    void log(const char* n);

    iTransport& io_;
    xHmi& hmi_;
    const xLogLinesBase* log_;
    const char* name_;

    uint8_t rx_[kRx];
    std::atomic<uint16_t> rxHead_{0}, rxTail_{0};
    std::atomic<uint32_t> rxDropped_{0};

    char line_[kLine + 1];
    uint8_t lineLen_ = 0;
    bool lastCr_ = false;
    uint8_t esc_ = 0;               // 1 after ESC, 2 in a CSI sequence

    char out_[kOut];
    size_t outHead_ = 0, outCount_ = 0;
    uint8_t tx_[2][kTx];
    uint8_t txNext_ = 0;
    size_t txLen_ = 0;
    uint32_t dropped_ = 0;

    Listing listing_ = Listing::None;
    uint16_t listAt_ = 0, listEnd_ = 0;

    xHmiLevel unlocked_ = xHmiLevel::None;
    uint32_t lastCommand_ = 0;
    uint8_t wrong_ = 0;
    bool lockout_ = false;
    uint32_t lockoutAt_ = 0;
};

} // namespace idisplay
