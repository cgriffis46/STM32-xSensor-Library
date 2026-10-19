#pragma once
#include <stdint.h>
#include "xScreen.h"
#include "iTextSurface.h"

namespace idisplay {

// What choosing a menu item does when it is not just opening a screen.
// Runs in the GUI task; nav opens the next screen, if any. ctx is
// whatever was given to add() (the station number, say).
typedef void (*xMenuAction)(xNavigator& nav, void* ctx);

// A list of items, one per row, that scrolls to keep the selection in
// view. Up and Down move the selection (on Pressed and Repeat), Enter
// chooses it, holding Enter (Held) or Back goes back. Each item opens a
// screen (pushed, so Back returns here), runs an action, or is a Back
// item.
//
// The texts are not copied: give it string literals or strings that
// outlive the menu. No heap.
class xMenu {
public:
    static constexpr uint8_t kMaxItems = 16;

    void clear();
    void setTitle(const char* title) { title_ = title; }   // shown inverse on the first row; nullptr for none

    bool add(const char* text, xScreen& target);
    bool add(const char* text, xMenuAction action, void* ctx = nullptr);
    bool addBack(const char* text = "Back");

    uint8_t count() const { return count_; }
    uint8_t selected() const { return sel_; }
    void select(uint8_t i) { if (i < count_) sel_ = i; }

    void up() { if (sel_ > 0) --sel_; }
    void down() { if (sel_ + 1 < count_) ++sel_; }
    void choose(xNavigator& nav);

    // Up and Down on Pressed or Repeat, Enter on Pressed or Held, Back
    // on Pressed. True if the key was used.
    bool onKey(xKey key, xKeyAction action, xNavigator& nav);

    // From the cursor row down to the bottom of the surface. The
    // selected row is marked with '>' and drawn inverse.
    void render(iTextSurface& s);

private:
    enum class Kind : uint8_t { Screen, Action, Back };
    struct Item {
        const char* text;
        Kind kind;
        xScreen* screen;
        xMenuAction action;
        void* ctx;
    };
    bool addItem(const char* text, Kind kind, xScreen* screen, xMenuAction action, void* ctx);

    Item items_[kMaxItems];
    uint8_t count_ = 0;
    uint8_t sel_ = 0;
    uint8_t first_ = 0;     // first item in view
    const char* title_ = nullptr;
};

// A screen that is a menu and nothing else. Fill menu() once, in the
// application's setup; the selection is kept when a screen it opened
// is popped, and reset when it is opened afresh.
class xMenuScreen : public xScreen {
public:
    explicit xMenuScreen(const char* title = nullptr) { menu_.setTitle(title); }
    xMenu& menu() { return menu_; }

    void onEnter(xNavigator& nav) override { (void)nav; menu_.select(0); }
    void onKey(xKey key, xKeyAction action, xNavigator& nav) override { menu_.onKey(key, action, nav); }
    void render(iTextSurface& s) override { menu_.render(s); }

private:
    xMenu menu_;
};

} // namespace idisplay
