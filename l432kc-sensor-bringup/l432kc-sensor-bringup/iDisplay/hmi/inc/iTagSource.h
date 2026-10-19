#pragma once
#include <stdint.h>

namespace idisplay {

// One tag's value as the HMI shows it: a number (every integer and real
// type, as a double), true/false, or nothing (no such tag, a type the
// HMI can't show, or the read failed).
struct xTagValue {
    enum class Kind : uint8_t { None, Bool, Number };
    Kind kind = Kind::None;
    bool b = false;
    double number = 0;

    static xTagValue none() { return xTagValue(); }
    static xTagValue boolean(bool v) { xTagValue t; t.kind = Kind::Bool; t.b = v; return t; }
    static xTagValue num(double v) { xTagValue t; t.kind = Kind::Number; t.number = v; return t; }
};

// Where an HMI reads its tags: a PLC tag database (PLCTransport's
// PlcTagSource), tags brought in over CAN or Ethernet, or a table of the
// application's own variables. Called from the GUI task only.
//
// find() is called with each name the layout uses, once it is known
// (and again on each redraw while it isn't, so tags registered after
// the GUI starts are picked up). read() is called on every redraw: it
// must not wait on anything slow, a bus or a socket. A source that gets
// its values from far away keeps the latest copy and reads that.
class iTagSource {
public:
    virtual ~iTagSource() {}
    // A handle for read(), or -1 if there is no such tag.
    virtual int32_t find(const char* name) = 0;
    // False if the value can't be had now; out is then ignored.
    virtual bool read(int32_t handle, xTagValue& out) = 0;
    // Writes an editable field's new value (a Number or a Bool), already
    // inside the field's limits. False if the source refused it: a
    // read-only tag, a value its type can't hold (2.5 into an integer),
    // or a source that only reads (the default). A source that sends
    // writes far away (CAN, Ethernet) returns true once the write is
    // queued; it must not wait for the answer here.
    virtual bool write(int32_t handle, const xTagValue& value) { (void)handle; (void)value; return false; }
};

} // namespace idisplay
