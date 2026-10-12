#pragma once
#include <stdint.h>
#include "iTextSurface.h"

namespace idisplay {

// A display chip and the buffer drawn for it. The GUI draws into
// text() (and text().graphics() on a graphic display), then asks for
// the buffer to be sent with requestFlush() and runs main() until
// idle() or failed().
//
// Like the sensor drivers, a device never waits on the bus: main()
// starts a transfer or checks on one and returns. Don't draw while a
// flush is under way (idle() false); the GUI task only draws between
// flushes, so this holds as long as one thread owns the display.
class iDisplayDevice {
public:
    virtual ~iDisplayDevice() {}

    virtual iTextSurface& text() = 0;

    // Send what has been drawn since the last flush. The first flush
    // also powers the display up and configures it.
    virtual void requestFlush() = 0;

    // Run the state machine one step.
    virtual void main(uint32_t nowMs) = 0;

    // Nothing waiting to be sent and nothing in flight.
    virtual bool idle() const = 0;

    // The last transfer failed and the device is backing off before it
    // starts again from power-up. main() keeps it going: it retries by
    // itself and repaints everything once it is back.
    virtual bool failed() const = 0;
};

} // namespace idisplay
