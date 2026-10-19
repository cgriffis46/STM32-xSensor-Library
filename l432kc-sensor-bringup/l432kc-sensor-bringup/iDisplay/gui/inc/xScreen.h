#pragma once
#include <stdint.h>
#include "xGuiEvent.h"
#include "iTextSurface.h"

namespace idisplay {

class xScreen;

// How a screen changes the screen shown. Passed to a screen's handlers,
// which run in the GUI task, so the change happens at once; from any
// other thread, post an xGuiEvent (show, push, pop, home) instead.
class xNavigator {
public:
    virtual void show(xScreen& s) = 0;  // replace the top screen
    virtual void push(xScreen& s) = 0;  // open s over the current one
    virtual void pop() = 0;             // back to the one underneath
    virtual void home() = 0;            // back to the home screen
protected:
    ~xNavigator() {}
};

// One screen of the GUI: the default readings screen, a menu, a page
// that edits one setting. The application's screens derive from this
// (the ready-made menu screen is xMenuScreen).
//
// Every handler runs in the GUI task, one event at a time, and the
// screen is redrawn after each. A screen does not draw anywhere but in
// render(), and never touches a button or a bus.
class xScreen {
public:
    virtual ~xScreen() {}

    // It has just been opened (show, push, or home). Load the values
    // to edit here.
    virtual void onEnter(xNavigator& nav) { (void)nav; }

    // It is on top again, the screen over it having been popped. A menu
    // keeps its selection; an edit screen might reload its value.
    virtual void onResume(xNavigator& nav) { (void)nav; }

    virtual void onKey(xKey key, xKeyAction action, xNavigator& nav) {
        (void)key; (void)action; (void)nav;
    }

    // A Refresh event: new data is available. arg is whatever the
    // poster put in it (which reading changed, say).
    virtual void onRefresh(uint32_t arg) { (void)arg; }

    // Draws the whole screen. The surface has been cleared, the cursor
    // is at 0,0 and inverse is off.
    virtual void render(iTextSurface& s) = 0;
};

} // namespace idisplay
