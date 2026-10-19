#pragma once
#include <stdint.h>
#include "xGuiEvent.h"
#include "iTextSurface.h"

namespace idisplay {

// What a field made of a key. In every field Up and Down step on Pressed
// and on Repeat (an auto-repeat button held down), Enter on Pressed is
// done (in xTextField: takes the character), and holding Enter (Held)
// or Back is cancel.
enum class xFieldResult : uint8_t {
    None,       // not a key it uses
    Changed,    // the value being edited changed
    Done,       // Enter: the value is final; save it
    Cancelled   // Back: leave without saving
};

// The fields an edit screen is made of. A field holds the value being
// edited, turns keys into changes, and draws itself at the cursor. It
// does not save anything or change screens: the screen it is on does
// that when the result is Done or Cancelled.
//
//   void onKey(xKey k, xKeyAction a, xNavigator& nav) override {
//       switch (active.onKey(k, a)) {
//       case xFieldResult::Done:      station.active = active.value(); save(); nav.pop(); break;
//       case xFieldResult::Cancelled: nav.pop(); break;
//       default: break;
//       }
//   }

// Yes or no. Up or Down flips it.
class xYesNoField {
public:
    explicit xYesNoField(const char* yes = "Yes", const char* no = "No") : yes_(yes), no_(no) {}
    bool value() const { return value_; }
    void set(bool v) { value_ = v; }
    xFieldResult onKey(xKey key, xKeyAction action);
    void render(iTextSurface& s) const;
private:
    const char* yes_;
    const char* no_;
    bool value_ = false;
};

// One of a list of texts. Up goes to the previous one, Down the next;
// neither wraps. The texts are not copied.
class xChoiceField {
public:
    static constexpr uint8_t kMaxChoices = 16;
    void clear() { count_ = 0; sel_ = 0; }
    bool add(const char* text);
    uint8_t count() const { return count_; }
    uint8_t selected() const { return sel_; }
    void select(uint8_t i) { if (i < count_) sel_ = i; }
    xFieldResult onKey(xKey key, xKeyAction action);
    void render(iTextSurface& s) const;
private:
    const char* texts_[kMaxChoices];
    uint8_t count_ = 0;
    uint8_t sel_ = 0;
};

// A string entered a character at a time, for a station name or a
// password with three buttons.
//
// The character under the cursor is drawn inverse. Up and Down step it
// through the character set, wrapping. After the set come two
// commands: delete (drawn '<' by default) removes the character before
// the cursor, and end (drawn '>') finishes, keeping what is before the
// cursor. Enter takes the character and moves on; on delete or end it
// does that command. Back, or holding Enter, cancels.
//
// Moving onto a character already in the string shows that character,
// so an existing value is kept by pressing Enter along it; at the end
// of the string the cursor shows end, so the last Enter finishes.
// Characters not in the set are kept as they are unless stepped away
// from.
// Once the buffer is full, Up and Down offer only the two commands.
//
// The string lives in the caller's buffer, NUL terminated; capacity
// counts the NUL.
class xTextField {
public:
    static const char kCharsetUpper[];          // A-Z 0-9 space
    static const char kCharsetAlnum[];          // A-Z a-z 0-9 space - _ .
    static const char kCharsetPrintable[];      // every printable ASCII character: change the command glyphs

    xTextField(char* buf, uint8_t capacity, const char* charset = kCharsetAlnum);

    // Start editing what is in the buffer now, from its first character.
    void begin();
    const char* value() const { return buf_; }
    uint8_t length() const { return len_; }
    uint8_t cursor() const { return pos_; }

    // How the two commands are drawn. Pick glyphs that are not in the
    // character set, or the screen cannot tell them apart.
    void setCommandGlyphs(char del, char end) { delGlyph_ = del; endGlyph_ = end; }

    xFieldResult onKey(xKey key, xKeyAction action);

    // From the cursor to the end of its row, scrolled sideways so the
    // character being edited is always in view.
    void render(iTextSurface& s) const;

private:
    // What is under the cursor: an index into the character set, or one
    // of these.
    static constexpr uint8_t kKeep = 0xFF;   // the string's own character, not in the set

    uint8_t delIndex() const { return setLen_; }
    uint8_t endIndex() const { return (uint8_t)(setLen_ + 1); }
    void loadCandidate();
    char glyphFor(uint8_t candidate) const;

    char* buf_;
    uint8_t cap_;
    const char* charset_;
    uint8_t setLen_;
    uint8_t len_ = 0;
    uint8_t pos_ = 0;
    uint8_t cand_ = 0;
    char delGlyph_ = '<';
    char endGlyph_ = '>';
};

// An xTextField with its own buffer of N bytes (N - 1 characters).
template <uint8_t N>
class xTextFieldN : public xTextField {
public:
    explicit xTextFieldN(const char* charset = kCharsetAlnum) : xTextField(storage_, N, charset) {}
    char* buffer() { return storage_; }
private:
    char storage_[N] = {0};
};

} // namespace idisplay
