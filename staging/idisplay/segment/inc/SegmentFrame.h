#pragma once
#include <stdint.h>
#include <string.h>

namespace idisplay {

// One segment of a segment LCD: the common line it is driven by and the
// segment line. For a multiplexed glass (4 COM: the STM32L1's LCD, the
// HT1621, the PCF8576) every icon, digit stroke, point and bar is one of these.
struct SegRef {
    uint8_t com;
    uint8_t seg;
};

// The whole glass as bits, up to 8 COM x 64 SEG: what a segment LCD's
// display RAM holds. Fields render into one; the driver copies it out.
class SegmentFrame {
public:
    static constexpr uint8_t kMaxCom = 8;
    static constexpr uint8_t kMaxSeg = 64;

    void clear() { memset(bits_, 0, sizeof bits_); }
    void set(SegRef s, bool on) {
        if (s.com >= kMaxCom || s.seg >= kMaxSeg) return;
        const uint64_t m = (uint64_t)1 << s.seg;
        if (on) bits_[s.com] |= m; else bits_[s.com] &= ~m;
    }
    bool get(SegRef s) const {
        return s.com < kMaxCom && s.seg < kMaxSeg && ((bits_[s.com] >> s.seg) & 1u);
    }
    uint64_t row(uint8_t com) const { return com < kMaxCom ? bits_[com] : 0; }
    bool operator==(const SegmentFrame& o) const { return memcmp(bits_, o.bits_, sizeof bits_) == 0; }
    bool operator!=(const SegmentFrame& o) const { return !(*this == o); }

private:
    uint64_t bits_[kMaxCom] = {};
};

// A segment LCD driver. Non-blocking: write() hands a frame over and
// returns false, doing nothing, while the previous one is still being
// taken up (the STM32L1's update request pending); try again later.
class iSegmentLcd {
public:
    virtual ~iSegmentLcd() = default;
    virtual bool write(const SegmentFrame& f) = 0;
    virtual bool busy() const = 0;
};

} // namespace idisplay
