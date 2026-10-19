#pragma once
#include <stddef.h>
#include <stdint.h>

namespace idisplay {

// The last few lines of a text log, kept for a display to show: the
// debug log (itransport's DebugLog.h) through its tap,
//
//   static xLogLines<8, 48> logLines;
//   dbg::setTap(xLogLinesBase::tap, &logLines);   // dbg::poll() feeds it
//
// or anything else that writes text. Lines end at '\n'; '\r' is dropped;
// a piece of text may end in the middle of a line (it is kept until the
// rest comes). A line longer than the width is cut. The oldest line goes
// when a new one comes.
//
// The writer (dbg::poll()'s thread) and the readers (the GUI task, the
// shell) are usually different threads: give setLock() a mutex for that,
// or a reader may copy a line while it is being replaced. Each lock is
// held for one line's copy.
class xLogLinesBase {
public:
    // Adds text. Not from an interrupt when there is a lock.
    void append(const char* text, size_t len);

    // dbg::Tap's signature: ctx is the xLogLinesBase.
    static void tap(const char* text, size_t len, void* ctx) {
        static_cast<xLogLinesBase*>(ctx)->append(text, len);
    }

    // Lines held (complete ones), at most lines().
    uint16_t count() const;
    uint16_t lines() const { return lines_; }
    uint8_t width() const { return width_; }
    // Every line ever completed, so a reader can tell new ones came.
    uint32_t total() const;

    // Copies a line, ending in 0: back 0 is the newest. Returns its
    // length; 0 (and "") when there is no such line.
    size_t copy(uint16_t back, char* out, size_t cap) const;

    void setLock(void (*lock)(void* ctx), void (*unlock)(void* ctx), void* ctx) {
        lock_ = lock; unlock_ = unlock; lockCtx_ = ctx;
    }

protected:
    xLogLinesBase(uint16_t lines, uint8_t width, char* text, uint8_t* lengths)
        : lines_(lines), width_(width), text_(text), len_(lengths) {}

private:
    void commit();

    uint16_t lines_;
    uint8_t width_;
    char* text_;            // lines_ + 1 rows of width_: the last is the one being built
    uint8_t* len_;          // lines_ + 1
    uint16_t head_ = 0;     // where the next complete line goes
    uint32_t total_ = 0;
    void (*lock_)(void*) = nullptr;
    void (*unlock_)(void*) = nullptr;
    void* lockCtx_ = nullptr;
};

template <uint16_t LINES, uint8_t WIDTH>
class xLogLines : public xLogLinesBase {
    static_assert(LINES > 0 && WIDTH > 0, "xLogLines: size");
public:
    xLogLines() : xLogLinesBase(LINES, WIDTH, text_, len_) {}
private:
    char text_[(LINES + 1u) * WIDTH];
    uint8_t len_[LINES + 1u] = {};
};

} // namespace idisplay
