#pragma once
#include <stdint.h>
#include "iTagSource.h"
#include "SegmentFrame.h"
#include "SegmentFields.h"

namespace idisplay {

// A segment LCD as an HMI: a layout of fields, each bound to tags, drawn
// from an iTagSource (a PLC tag database, the meter's own tags, a chain
// of both: xTagChain). refresh() renders every field into a frame and
// hands it to the LCD only if it changed. Non-blocking: a busy LCD keeps
// the frame for the next refresh().
//
//   static iSegmentField* const fields[] = { &mainValue, &voltsIcon, &autoIcon, ... };
//   xSegmentHmi hmi(lcd, tags, fields, sizeof fields / sizeof fields[0]);
//   every 100 ms, or when a tag changes:  hmi.refresh();
//
// Fields are drawn in order onto a cleared frame, so a later field wins
// a segment two fields share.
class xSegmentHmi {
public:
    xSegmentHmi(iSegmentLcd& lcd, iTagSource& src, iSegmentField* const* fields, uint16_t count)
        : lcd_(lcd), src_(src), fields_(fields), count_(count) {}

    // True if a new frame went to the LCD.
    bool refresh() {
        SegmentFrame f;
        if (!test_) for (uint16_t i = 0; i < count_; i++) fields_[i]->render(src_, f);
        else for (uint8_t c = 0; c < SegmentFrame::kMaxCom; c++)
            for (uint8_t s = 0; s < SegmentFrame::kMaxSeg; s++) f.set(SegRef{c, s}, true);
        if (!pending_ && haveShown_ && f == shown_) return false;
        if (!lcd_.write(f)) { pending_ = true; return false; }
        shown_ = f;
        haveShown_ = true;
        pending_ = false;
        return true;
    }

    // Every segment on, for checking the glass (the stock 121GW does it at power on).
    void setTestPattern(bool on) { test_ = on; }
    // Something else drew on the LCD: the next refresh() writes even if
    // the frame is the one this HMI showed last.
    void invalidate() { haveShown_ = false; }
    const SegmentFrame& shown() const { return shown_; }

private:
    iSegmentLcd& lcd_;
    iTagSource& src_;
    iSegmentField* const* fields_;
    uint16_t count_;
    SegmentFrame shown_;
    bool haveShown_ = false;
    bool pending_ = false;
    bool test_ = false;
};

// Several tag sources as one: find() asks each in turn, the first that
// knows the name answers. Up to 8 sources, 2^24 handles each.
class xTagChain : public iTagSource {
public:
    xTagChain(iTagSource* const* sources, uint8_t count) : src_(sources), n_(count > 8 ? 8 : count) {}

    int32_t find(const char* name) override {
        for (uint8_t i = 0; i < n_; i++) {
            const int32_t h = src_[i]->find(name);
            if (h >= 0 && h < (1 << 24)) return (int32_t)((uint32_t)i << 24 | (uint32_t)h);
        }
        return -1;
    }
    bool read(int32_t h, xTagValue& out) override {
        if (h < 0) return false;
        const uint8_t i = (uint8_t)(h >> 24);
        return i < n_ && src_[i]->read(h & 0xFFFFFF, out);
    }
    bool write(int32_t h, const xTagValue& v) override {
        if (h < 0) return false;
        const uint8_t i = (uint8_t)(h >> 24);
        return i < n_ && src_[i]->write(h & 0xFFFFFF, v);
    }

private:
    iTagSource* const* src_;
    uint8_t n_;
};

} // namespace idisplay
