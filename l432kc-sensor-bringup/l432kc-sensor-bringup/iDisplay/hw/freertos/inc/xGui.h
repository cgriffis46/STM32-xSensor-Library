#pragma once
#include <stdint.h>
#include "cmsis_os2.h"
#include "xGuiCore.h"
#include "xButton.h"
#include "iDisplayDevice.h"

namespace idisplay {

// The GUI task (CMSIS-RTOS2). It owns the display: nothing else draws
// on it or flushes it, and the only way in is its event queue.
//
//   - Buttons (xGuiButton) only post key events to the queue, from
//     their pin interrupts.
//   - Other threads post Refresh when there is new data to show, or a
//     screen change (show, push, pop, home), with post().
//   - The task waits on the queue and does nothing else: no timer and
//     no polling. Each event goes to the top screen or the screen stack
//     (xGuiCore), then the screen is drawn into the display's buffer
//     and flushed. Only the pages that changed go out on the bus.
//
// The queue holds one event by default. That is the flow control:
// while an event is waiting, or the task is still drawing the last
// one, the next button press finds the queue full and is dropped, so
// a bouncing or held button cannot pile up presses behind a slow bus.
// The cost is that a Refresh posted with no timeout is dropped too
// while a key is waiting; it is not missed, as the key's redraw shows
// the new data anyway.
//
//   static xssd1306<Stm32HalI2CTransport, 64> oled(param, &hi2c1, SSD1306_I2C_ADDR, i2c1Mutex);
//   static HomeScreen homeScreen;
//   static xGui gui(oled, homeScreen);
//   static xGuiButton upButton(gui, xKey::Up), downButton(gui, xKey::Down), enterButton(gui, xKey::Enter);
//
//   gui.start();                                   // before osKernelStart()
//
// (For long presses and auto-repeat, give the buttons an xButtonConfig
// and put them in an xGuiButtonGroup; see below.)
//
//   void HAL_GPIO_EXTI_Callback(uint16_t pin) {    // EXTI on both edges, pull-up, button to ground
//       if (pin == UP_Pin) upButton.onEdge(HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == GPIO_PIN_RESET);
//       ...
//   }
//
//   gui.post(xGuiEvent::refresh());                // from the sensor thread, when a reading lands
//
// Use the x* variant of the display driver (xssd1306): its sleep()
// gives up the CPU while a transfer is in flight. With the bare one the
// task spins until the flush is done.
//
// Ticks are taken to be milliseconds, the usual 1 kHz tick.
class xGui {
public:
    xGui(iDisplayDevice& display, xScreen& home, uint32_t queueDepth = 1);

    // Creates the queue and the task. False if either could not be
    // created. Call once, before or after osKernelStart().
    bool start(const char* name = "gui", uint32_t stackBytes = 1024,
               osPriority_t priority = osPriorityNormal);

    // Posts ev to the GUI. From an interrupt, timeoutTicks must be 0.
    // False if the queue stayed full (or start() has not run).
    bool post(const xGuiEvent& ev, uint32_t timeoutTicks = 0);

    osMessageQueueId_t queue() const { return queue_; }
    xGuiCore& core() { return core_; }

    // The task body: opens the home screen, draws it, then handles
    // events for ever. start() runs it in its own task.
    void run();

    // One pass of run() after the first draw: waits up to timeoutTicks
    // for an event and handles it. False on timeout. For tests, and for
    // running the GUI from a task of your own.
    bool runOnce(uint32_t timeoutTicks);

    // Opens the home screen and draws it: the start of run().
    void begin();

private:
    static void threadEntry(void* arg);
    void draw();

    iDisplayDevice& display_;
    xGuiCore core_;
    uint32_t queueDepth_;
    osMessageQueueId_t queue_ = nullptr;
    osThreadId_t thread_ = nullptr;
};

// A button wired to an interrupt on both edges. Its only output is a
// key event posted to the GUI's queue, with no wait: if the queue is
// full the event is dropped.
//
// What it sends is set by its xButtonConfig: plain (Pressed as it goes
// down), long press (short press: Pressed on release; long: Held), or
// auto-repeat (Pressed, then Repeat while held). Held and Repeat come
// from xGuiButtonGroup's timer, so a long press or auto-repeat button
// must be added to a started group.
class xGuiButton {
public:
    xGuiButton(xGui& gui, xKey key, uint16_t debounceMs = 50, bool reportReleases = false)
        : gui_(gui), button_(key, debounceMs, reportReleases) {}
    xGuiButton(xGui& gui, xKey key, const xButtonConfig& config)
        : gui_(gui), button_(key, config) {}

    // From the pin interrupt (or a poll), with the button's state now:
    // true when it is pressed. True if an event was posted.
    bool onEdge(bool down);

    // From xGuiButtonGroup's timer. True if an event was posted.
    bool onTick();

    xButton& button() { return button_; }

private:
    xGui& gui_;
    xButton button_;
};

// The clock for long presses and auto-repeat: a periodic CMSIS-RTOS2
// timer that asks each of its buttons whether it has been held long
// enough, and lets the button post Held or Repeat to the GUI's queue.
// It never touches a screen or the display. A pin interrupt cannot
// start a timer (osTimerStart is not allowed in one), so this one runs
// all the time; each tick is a few comparisons per button.
//
//   static xGuiButton upButton(gui, xKey::Up, xButtonConfig::autoRepeat());
//   static xGuiButton downButton(gui, xKey::Down, xButtonConfig::autoRepeat());
//   static xGuiButton enterButton(gui, xKey::Enter, xButtonConfig::longPress());
//   static xGuiButtonGroup buttons;
//
//   buttons.add(upButton); buttons.add(downButton); buttons.add(enterButton);
//   buttons.start();                               // after gui.start()
//
// The callback runs in the RTOS timer task (configUSE_TIMERS on
// FreeRTOS). Held and Repeat are as late as one period.
class xGuiButtonGroup {
public:
    static constexpr uint8_t kMaxButtons = 8;

    explicit xGuiButtonGroup(uint32_t periodMs = 20) : periodMs_(periodMs ? periodMs : 1) {}

    // Before start(). False when the group is full.
    bool add(xGuiButton& b);

    // Creates and starts the timer. False if it could not be.
    bool start();

    // The timer's body: one look at every button.
    void tick();

    osTimerId_t timer() const { return timer_; }

private:
    static void timerEntry(void* arg);

    xGuiButton* buttons_[kMaxButtons];
    uint8_t count_ = 0;
    uint32_t periodMs_;
    osTimerId_t timer_ = nullptr;
};

// A periodic CMSIS-RTOS2 timer that posts Refresh to the GUI, for
// screens that show values nobody announces: xHmi reading a tag
// database, a clock. Like a button it only posts to the queue; a
// refresh that finds the queue full (a key waiting) is dropped, and the
// next tick brings another.
//
//   static xGuiTicker ticker(gui, plant_hmi.refreshMs);
//   ticker.start();                                // after gui.start()
//
// The callback runs in the RTOS timer task.
class xGuiTicker {
public:
    xGuiTicker(xGui& gui, uint32_t periodMs) : gui_(gui), periodMs_(periodMs ? periodMs : 1) {}

    // Creates and starts the timer. False if it could not be.
    bool start();

    // The timer's body. True if the refresh was posted.
    bool tick() { return gui_.post(xGuiEvent::refresh(), 0); }

    osTimerId_t timer() const { return timer_; }

private:
    static void timerEntry(void* arg);

    xGui& gui_;
    uint32_t periodMs_;
    osTimerId_t timer_ = nullptr;
};

} // namespace idisplay
