#include "xTerminal.h"
#include <string.h>

namespace idisplay {

xTerminalBase::xTerminalBase(iTransport& out, const xTerminalConfig& config, uint8_t cols, uint8_t rows,
                             uint8_t logRows, uint8_t logCols, uint8_t* want, uint8_t* sent)
    : iTextSurface(cols, rows), out_(out), config_(config), logRows_(config.log ? logRows : 0),
      want_(want), sent_(sent) {
    const uint8_t f = config_.frame ? 1 : 0;
    x0_ = f;
    y0_ = f;
    w_ = (uint8_t)(cols + 2 * f);
    h_ = (uint8_t)(rows + 2 * f);
    logTop_ = h_;
    if (logRows_) {
        if (!f) ++logTop_;                  // a row of '-' between them
        if (logCols > w_) w_ = logCols;
        h_ = (uint8_t)(logTop_ + logRows_);
    }
}

// The buffers are the derived class's members, so they are first
// touched here, not in the constructor.
void xTerminalBase::ensureInit() {
    if (init_) return;
    init_ = true;
    memset(want_, ' ', (size_t)w_ * h_);
    memset(sent_, kUnknown, (size_t)w_ * h_);
    drawFrame();
}

void xTerminalBase::drawFrame() {
    const uint8_t right = (uint8_t)(cols() + 1);
    const uint8_t bottom = (uint8_t)(rows() + 1);
    if (config_.frame) {
        for (uint8_t x = 0; x <= right; ++x) {
            const char c = x == 0 || x == right ? '+' : '-';
            want_[x] = (uint8_t)c;
            want_[(size_t)bottom * w_ + x] = (uint8_t)c;
        }
        for (uint8_t y = 1; y < bottom; ++y) {
            want_[(size_t)y * w_] = '|';
            want_[(size_t)y * w_ + right] = '|';
        }
    } else if (logRows_) {
        memset(want_ + (size_t)rows() * w_, '-', w_);
    }
}

void xTerminalBase::clear() {
    ensureInit();
    for (uint8_t y = 0; y < rows(); ++y) memset(want_ + (size_t)(y0_ + y) * w_ + x0_, ' ', cols());
    homeCursor();
}

void xTerminalBase::drawCell(uint8_t col, uint8_t row, char c, bool inverse) {
    ensureInit();
    uint8_t b = (uint8_t)c;
    if (c != kDegreeChar && (b < 0x20 || b > 0x7E)) b = '?';
    want_[(size_t)(y0_ + row) * w_ + x0_ + col] = (uint8_t)(b | (inverse ? kInverse : 0));
}

void xTerminalBase::drawLog() {
    if (!logRows_ || config_.log == nullptr) return;
    char line[256];
    for (uint8_t r = 0; r < logRows_; ++r) {
        const size_t n = config_.log->copy((uint16_t)(logRows_ - 1 - r), line, sizeof line);
        uint8_t* row = want_ + (size_t)(logTop_ + r) * w_;
        for (uint8_t x = 0; x < w_; ++x) {
            const uint8_t b = x < n ? (uint8_t)line[x] : ' ';
            row[x] = b < 0x20 || b > 0x7E ? '?' : b;
        }
    }
}

void xTerminalBase::put(const char* s) {
    while (*s && txLen_ < kTxBuffer) tx_[txNext_][txLen_++] = (uint8_t)*s++;
}

void xTerminalBase::putNumber(uint8_t v) {
    char d[4];
    uint8_t n = 0;
    do { d[n++] = (char)('0' + v % 10); v = (uint8_t)(v / 10); } while (v);
    while (n && txLen_ < kTxBuffer) tx_[txNext_][txLen_++] = (uint8_t)d[--n];
}

void xTerminalBase::startFlush() {
    if (!started_ || redrawWanted_) {
        redrawWanted_ = false;
        started_ = true;
        memset(sent_, kUnknown, (size_t)w_ * h_);
        put("\x1b[0m\x1b[2J\x1b[?25l");
        at_ = -1;
        attrInverse_ = false;
    }
    drawLog();
    pos_ = 0;
    scanning_ = true;
    flushWanted_ = false;
}

// Fills the buffer with the changed cells from pos_ on, until it is full
// or every cell has been looked at.
void xTerminalBase::scan() {
    const uint16_t cells = (uint16_t)(w_ * h_);
    // The most one cell can take: ESC[250;250H, ESC[27m, two bytes.
    static const size_t kWorst = 10 + 5 + 2;
    while (pos_ < cells) {
        const uint8_t cell = want_[pos_];
        if (cell == sent_[pos_]) { ++pos_; continue; }
        if (txLen_ + kWorst > kTxBuffer) return;
        if (at_ != (int16_t)pos_) {
            put("\x1b[");
            putNumber((uint8_t)(pos_ / w_ + 1));
            put(";");
            putNumber((uint8_t)(pos_ % w_ + 1));
            put("H");
        }
        const bool inv = (cell & kInverse) != 0;
        if (inv != attrInverse_) {
            put(inv ? "\x1b[7m" : "\x1b[27m");
            attrInverse_ = inv;
        }
        const uint8_t c = (uint8_t)(cell & ~kInverse);
        if (c == (uint8_t)kDegreeChar) put("\xC2\xB0");
        else tx_[txNext_][txLen_++] = c;
        sent_[pos_] = cell;
        ++pos_;
        // After the last column a terminal's cursor waits to wrap; where it
        // is then differs between terminals, so it is moved explicitly.
        at_ = pos_ % w_ == 0 ? -1 : (int16_t)pos_;
    }
    scanning_ = false;
}

void xTerminalBase::main(uint32_t nowMs) {
    ensureInit();
    if (failed_) {
        if (nowMs - failedAt_ < config_.backoffMs) return;
        failed_ = false;
        started_ = false;
        flushWanted_ = true;
    }
    if (!scanning_ && txLen_ == 0 && flushWanted_) startFlush();
    for (;;) {
        if (scanning_) scan();
        if (txLen_ == 0) return;
        if (!out_.write(tx_[txNext_], txLen_)) break;
        bytesSent_ += (uint32_t)txLen_;
        txNext_ ^= 1;
        txLen_ = 0;
        refusing_ = false;
        if (!scanning_) return;
    }
    // Refused: the last buffer is still going out (or the UART is stuck).
    if (!refusing_) {
        refusing_ = true;
        refusedSince_ = nowMs;
    } else if (nowMs - refusedSince_ >= config_.stuckMs) {
        refusing_ = false;
        failed_ = true;
        failedAt_ = nowMs;
        ++failures_;
        scanning_ = false;
        txLen_ = 0;
        flushWanted_ = false;
        return;
    }
    if (config_.sleep) config_.sleep(1);
}

} // namespace idisplay
