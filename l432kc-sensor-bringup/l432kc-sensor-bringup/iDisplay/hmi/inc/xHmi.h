#pragma once
#include <stddef.h>
#include <stdint.h>
#include "xScreen.h"
#include "xHmiLayout.h"
#include "iTagSource.h"

namespace idisplay {

// A whole HMI as one xScreen: draws the current screen of a layout
// (xHmiLayout, made by hmi_gen.py) with values read from an iTagSource,
// moves between the layout's screens on the keys it gives them, and
// edits the fields the layout marks editable. It is the GUI's home
// screen:
//
//   static PlcTagSource tags(registry);           // PLCTransport
//   static xHmi hmi(plant_hmi, tags);             // plant_hmi from plant.json
//   hmi.setPin(xHmiLevel::Operator, "2580");      // if fields have levels
//   static xGui gui(lcd, hmi);
//   static xGuiTicker ticker(gui, plant_hmi.refreshMs);
//   gui.start(); ticker.start();
//
// Like every screen, it only draws in render(), in the GUI task; the
// ticker's refreshes (or a data thread's) are what make it redraw. The
// HD44780 and SSD1306 drivers send only what changed, so redrawing
// everything each time is cheap on the bus.
//
// How a value is shown (the browser preview does the same):
//   Value  the number rounded to its decimals (half away from zero), in
//          width cells, right-aligned unless asked otherwise. Too long
//          for its width: '#' in every cell. Unknown tag, failed read or
//          not a number (NaN, infinite): "--". A bool is 1 or 0.
//   State  on or off text, left-aligned in its width; unknown: "--".
//   Choice the label of the choice whose value the tag holds; another
//          value or unknown: "--".
//   Time   seconds modulo a day as HH:MM or HH:MM:SS; unknown or
//          negative: dashes (xTimeHHMM).
//
// Editing, on a screen that has fields (the generator keeps Enter free
// there, and fields off the last row, which is the status line):
//   Enter         picks the first field: inverse (OLED) and the blinking
//                 cursor (LCD) on it, its label on the last row
//   Up / Down     the previous / next field; Back: done
//   Enter         edits it, starting from the tag's value (a value that
//                 can't be read: "No value"). A field with a level asks
//                 for a PIN first, unless that level (or a higher one) is
//                 unlocked; 4 digits, Up/Down change one, Enter takes it,
//                 Back goes back a digit. Three wrong: "Locked" for 60 s.
//   Up / Down     change it: a number by one step (and on auto-repeat;
//                 ten steps at a time after ten repeats), within min..max;
//                 a state flips; a choice moves to the previous / next.
//                 Back drops the edit.
//   Enter         unchanged: back to picking. Changed: "Write <value>?"
//                 with No picked; Up/Down choose, Enter answers, Back
//                 returns to the edit. Yes writes (rounded to the shown
//                 decimals) and says "Written" or "Refused".
// An edit (or a pick) left alone for editTimeoutMs is dropped; an unlock
// ends unlockTimeoutMs after the last key. Both are counted in refreshes,
// so they need the ticker. Every write goes to the audit callback.
// Messages show on the last row until a key or about 2 s.
//
// RAM: a handle per tag (tagCount, at most kMaxTags), the PINs and the
// editing state: 332 bytes on a Cortex-M4.
class xHmi : public xScreen {
public:
    static constexpr uint16_t kMaxTags = 64;
    static constexpr uint8_t kPinDigits = 4;
    static constexpr uint8_t kPinTries = 3;
    static constexpr uint32_t kLockoutMs = 60000;
    static constexpr uint32_t kMessageMs = 2000;

    enum class Mode : uint8_t { View, Pick, Edit, Confirm, Pin };

    // Who wrote what: tag, its value before (kind None if it couldn't be
    // read) and the value sent, whether the source took it, and the level
    // unlocked at the time.
    typedef void (*AuditFn)(const char* tag, const xTagValue& before, const xTagValue& after,
                            bool written, xHmiLevel unlocked, void* ctx);

    xHmi(const xHmiLayout& layout, iTagSource& source);

    // The PIN for a level: exactly kPinDigits digits. False (and none
    // set) otherwise. A level without a PIN can't be unlocked, so its
    // fields can't be edited (unless a higher level's PIN is set).
    bool setPin(xHmiLevel level, const char* digits);
    void setAudit(AuditFn fn, void* ctx) { audit_ = fn; auditCtx_ = ctx; }
    AuditFn audit() const { return audit_; }
    void* auditContext() const { return auditCtx_; }

    // The screen shown, by index or id. False: no such screen. Drops any
    // edit.
    uint8_t current() const { return screen_; }
    bool showScreen(uint8_t index);
    bool showScreen(const char* id);

    Mode mode() const { return mode_; }
    xHmiLevel unlocked() const { return unlocked_; }
    const char* message() const { return message_; }

    // Tags the source doesn't have (yet): how many, and their names.
    uint16_t missingCount() const;
    const char* missing(uint16_t i) const;

    // Finds the tags again (after the source changed, say).
    void rebind();

    void onEnter(xNavigator& nav) override;
    void onKey(xKey key, xKeyAction action, xNavigator& nav) override;
    void onRefresh(uint32_t arg) override;
    void render(iTextSurface& s) override;

    // The number as a Value item shows it, into out, ending in 0, before
    // it is aligned in its width. Returns its length; 0 (and "--") when v
    // is NaN or infinite; kTooLong when it doesn't fit in cap - 1
    // characters or is 1e15 or more once scaled by its decimals. Public
    // for tests.
    static constexpr size_t kTooLong = (size_t)-1;
    static size_t formatNumber(double v, uint8_t decimals, char* out, size_t cap);

    // For other ways in to the same HMI (xShell, a serial shell), so they
    // keep its rules:
    const xHmiLayout& layout() const { return layout_; }
    iTagSource& source() { return source_; }
    // The highest level whose PIN is digits; None if none matches.
    xHmiLevel checkPin(const char* digits) const;
    bool pinSet(xHmiLevel level) const;
    // An item's value as the panel shows it, before alignment (Value,
    // State, Choice): "--" unknown, '#' in every cell when too long.
    static size_t itemText(const xHmiItem& it, bool known, const xTagValue& v, char* out, size_t cap);
    // Whether a field may be set to x, as the panel could set it: a Value
    // within min..max, a whole number of steps from min and no more
    // decimals than it shows; a State 0 or 1 (written as a Bool); a
    // Choice one of its values. out is what to write.
    static bool fieldAccepts(const xHmiItem& it, double x, xTagValue& out);

private:
    bool bound(uint16_t tag);
    bool readTag(uint16_t tag, xTagValue& v);
    uint16_t fieldCount() const;
    const xHmiItem* field(uint16_t i) const;
    uint32_t ticks(uint32_t ms) const;
    void say(const char* text);
    void dropEdit();
    void startEdit();
    void step(int32_t by);
    void finishEdit();
    void enterDigit();
    xTagValue pendingValue() const;
    void drawItem(iTextSurface& s, const xHmiItem& it, bool picked);
    void drawStatus(iTextSurface& s, const char* a, const char* b, bool inverse);
    void drawConfirm(iTextSurface& s);
    void drawPin(iTextSurface& s);

    const xHmiLayout& layout_;
    iTagSource& source_;
    int32_t handles_[kMaxTags];
    uint8_t screen_ = 0;

    Mode mode_ = Mode::View;
    uint16_t pick_ = 0;             // the field picked, among the screen's fields
    int32_t pending_ = 0;           // Value: steps from min; State: 0/1; Choice: index
    int32_t original_ = 0;          // the same, as the edit started
    bool yes_ = false;              // Confirm: Yes picked
    uint8_t repeats_ = 0;           // auto-repeats in a row
    uint32_t idle_ = 0;             // refreshes since the last key

    char pins_[3][kPinDigits];      // by xHmiLevel; [0] unused
    bool pinSet_[3] = {false, false, false};
    xHmiLevel unlocked_ = xHmiLevel::None;
    xHmiLevel need_ = xHmiLevel::None;
    uint8_t digits_[kPinDigits];
    uint8_t digit_ = 0;
    uint8_t wrong_ = 0;
    uint32_t lockout_ = 0;          // refreshes left

    const char* message_ = nullptr;
    uint32_t messageLeft_ = 0;

    AuditFn audit_ = nullptr;
    void* auditCtx_ = nullptr;
};

} // namespace idisplay
