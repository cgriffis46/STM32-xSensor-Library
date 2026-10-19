#pragma once
#include <stdint.h>

namespace idisplay {

// An HMI described as constant tables, made from a JSON file by
// hmi/tools/hmi_gen.py (see iDisplay/README.md). All of it is const, so
// it stays in flash; the only RAM an HMI needs is xHmi's own.
//
// Positions and widths are in character cells. The generator has
// already checked that every item fits on the display and that no two
// overlap, so nothing here checks again beyond clipping at the edge.

enum class xHmiKind : uint8_t {
    Text,   // fixed text
    Value,  // a tag as a number, in width cells, then unit
    State,  // a tag as on or off text (non-zero or true is on)
    Choice, // a tag as the label of the choice with its value
    Time    // a tag holding seconds (since midnight, or Unix time) as HH:MM[:SS]
};

enum class xHmiAlign : uint8_t { Right, Left };

// Who may change a field. A field with a level is edited only after
// that level's PIN (or a higher one) is entered on the panel.
enum class xHmiLevel : uint8_t { None, Operator, Engineer };

// One entry of a choice: what it shows and the tag value it stands for.
struct xHmiChoice {
    const char* label;
    double value;
};

// What makes an item an editable field (a Value, State or Choice item
// with an "edit" in the JSON). Values outside min..max are never
// written; a number moves in whole steps from min.
struct xHmiEdit {
    const char* label;          // shown while it is picked or edited, and when confirming
    double min, max, step;      // Value only
    xHmiLevel level;
};

static constexpr uint16_t kHmiNoTag = 0xFFFF;
static constexpr uint8_t kHmiNoScreen = 0xFF;
static constexpr uint8_t kHmiKeys = 6;          // xKey: Up, Down, Enter, Back, Left, Right

struct xHmiItem {
    xHmiKind kind;
    uint8_t col, row;
    uint8_t width;          // Value: the number's cells (unit after); State, Choice: the longest text's
    uint8_t decimals;       // Value: places after the point
    xHmiAlign align;        // Value: in its width
    bool seconds;           // Time: HH:MM:SS rather than HH:MM
    uint16_t tag;           // index into xHmiLayout::tags, or kHmiNoTag (Text)
    const char* text;       // Text
    const char* unit;       // Value: printed after the number, or nullptr
    const char* on;         // State
    const char* off;        // State
    const xHmiChoice* choices;  // Choice
    uint8_t choiceCount;
    const xHmiEdit* edit;   // nullptr: shown only
};

struct xHmiScreen {
    const char* id;
    const xHmiItem* items;
    uint16_t itemCount;
    uint8_t keys[kHmiKeys]; // the screen a key's press opens, by xKey; kHmiNoScreen: none
};

struct xHmiLayout {
    uint8_t cols, rows;
    uint16_t refreshMs;     // how often the application should post a refresh (xGuiTicker)
    const char* const* tags;
    uint16_t tagCount;
    const xHmiScreen* screens;  // the first is the home screen
    uint8_t screenCount;
    uint32_t editTimeoutMs;     // an edit left alone this long is dropped
    uint32_t unlockTimeoutMs;   // a PIN's unlock lasts this long after the last key
};

} // namespace idisplay
