#pragma once
#include <stddef.h>
#include <stdint.h>
#include "xScreen.h"
#include "iTagSource.h"
#include "xFormat.h"

namespace idisplay {

// One reading on one row: a caption on the left, the value and its unit
// on the right,
//
//   Temp        21.5\x7FC
//
// with the value from a variable, a function or a tag:
//
//   static float testTemp;                                   // a sensor test value
//   static xLabel labels[] = {
//       xLabel::of("Temp", testTemp, 1, "\x7F" "C"),
//       xLabel::fn("Press", [](const void* s) { return (double)static_cast<const Barometer*>(s)->getPressure() / 100; },
//                  &bmp, 1, "hPa"),                         // isensor's interfaces
//       xLabel::tag("Level", tags, "tank.level", 0, "%"),   // an iTagSource (PlcTagSource, xTagTable)
//       xLabel::of("Pump", pumpOn, "run", "stop"),
//   };
//
// Numbers are shown as the HMI shows them (half away from zero, no -0;
// NaN, a missing tag or a failed read: "--"). A value too long for the
// row is '#' in every cell it would take; a caption too long is cut.
// What a label points at must outlive it. A label reads its source when
// drawn, in the GUI task (see xTagTable for variables other threads
// change).
class xLabel {
public:
    typedef double (*Getter)(const void* ctx);

    static xLabel text(const char* caption) { return xLabel(caption); }
    static xLabel of(const char* caption, const float& v, uint8_t decimals, const char* unit = nullptr);
    static xLabel of(const char* caption, const int32_t& v, const char* unit = nullptr);
    static xLabel of(const char* caption, const bool& v, const char* on, const char* off);
    static xLabel fn(const char* caption, Getter get, const void* ctx, uint8_t decimals, const char* unit = nullptr);
    static xLabel tag(const char* caption, iTagSource& source, const char* name, uint8_t decimals,
                      const char* unit = nullptr);
    // A tag that is true or false, shown as on or off text.
    static xLabel tagState(const char* caption, iTagSource& source, const char* name, const char* on, const char* off);

    // The value now: None when there is none (a caption only, a missing
    // tag, a failed read).
    xTagValue value();
    // The value as shown, without the unit, into out (ending in 0).
    // Returns its length; kFormatTooLong (xFormat.h) if it doesn't fit.
    size_t valueText(char* out, size_t cap);

    // Draws it at col,row in width cells.
    void draw(iTextSurface& s, uint8_t col, uint8_t row, uint8_t width);

    const char* caption() const { return caption_; }
    const char* unit() const { return unit_; }

private:
    enum class Source : uint8_t { None, Float, Int, Bool, Getter, Tag };
    explicit xLabel(const char* caption) : caption_(caption) {}

    const char* caption_;
    const char* unit_ = nullptr;
    const char* on_ = nullptr;
    const char* off_ = nullptr;
    Source source_ = Source::None;
    uint8_t decimals_ = 0;
    const void* ptr_ = nullptr;     // the variable, or the getter's context
    Getter get_ = nullptr;
    iTagSource* tags_ = nullptr;
    const char* name_ = nullptr;
    int32_t handle_ = -1;           // found again while -1
};

// A block of the screen holding labels, one a row, with an optional
// title row (shown inverse) at the top:
//
//   static xCell outside(0, 0, 20, 4, labels, 4, "Outside");
//
// More labels than rows: scroll() moves through them (xCellScreen does
// it with Up and Down). The labels are the application's (an array that
// outlives the cell).
class xCell {
public:
    xCell(uint8_t col, uint8_t row, uint8_t width, uint8_t height, xLabel* labels, uint8_t count,
          const char* title = nullptr)
        : col_(col), row_(row), width_(width), height_(height), labels_(labels), count_(count), title_(title) {}

    void render(iTextSurface& s);

    // Rows for labels (the height less the title's).
    uint8_t labelRows() const { return (uint8_t)(height_ > (title_ ? 1 : 0) ? height_ - (title_ ? 1 : 0) : 0); }
    bool scrolls() const { return count_ > labelRows(); }
    // The first label shown. scroll() stays within the labels.
    uint8_t first() const { return first_; }
    void scroll(int by);

    uint8_t count() const { return count_; }
    xLabel& label(uint8_t i) { return labels_[i]; }

private:
    uint8_t col_, row_, width_, height_;
    xLabel* labels_;
    uint8_t count_;
    const char* title_;
    uint8_t first_ = 0;
};

// A screen of cells: a maintenance page of sensor readings, say. Up and
// Down scroll every cell that has more labels than rows; Back goes back
// (pop). Open it from a menu (push).
class xCellScreen : public xScreen {
public:
    xCellScreen(xCell* const* cells, uint8_t count) : cells_(cells), count_(count) {}
    void onKey(xKey key, xKeyAction action, xNavigator& nav) override;
    void render(iTextSurface& s) override;
private:
    xCell* const* cells_;
    uint8_t count_;
};

} // namespace idisplay
