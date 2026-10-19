#pragma once
#include <stddef.h>
#include <stdint.h>
#include "iTagSource.h"

namespace idisplay {

// The application's own variables as tags, for an HMI, xLabels or the
// serial shell without a PLC tag database: sensor test values, settings,
// counters. A table of entries, each a name and where its value is:
//
//   static float setpoint = 75;
//   static const xTagTable::Entry kTags[] = {
//       xTagTable::var("tank.setpoint", setpoint, true),           // writable
//       xTagTable::var("pump.count", pumpCount),
//       xTagTable::get("sht.temp", [](const void* s) { return (double)static_cast<const Thermometer*>(s)->getTemperature(); }, &sht31),
//   };
//   static xTagTable tags(kTags, 3);
//
// A getter is read-only: any function of a context (a sensor through
// isensor's Thermometer/Barometer/Hygrometer/Altimeter interfaces). The
// table and what it points at must outlive it.
//
// Threads: reads come from the GUI task (and the shell's), while other
// threads change the variables. A 32-bit or smaller variable is read
// whole on a Cortex-M; a double or a getter that reads several values is
// not, so give setLock() functions (a mutex, or masking interrupts) if
// that matters. Writes are refused unless the value fits the variable's
// type exactly (2.5 into an integer, 300 into a uint8_t, NaN, 2 into a
// bool), as PLCTransport's PlcTagSource does.
class xTagTable : public iTagSource {
public:
    enum class Type : uint8_t { Bool, I8, U8, I16, U16, I32, U32, F32, F64, Getter };
    typedef double (*Getter)(const void* ctx);

    struct Entry {
        const char* name;
        Type type;
        void* ptr;          // the variable (not for Getter)
        bool writable;
        Getter getter;      // Getter only
        const void* ctx;    // Getter only
    };

    static Entry var(const char* n, bool& v, bool w = false)     { return Entry{n, Type::Bool, &v, w, nullptr, nullptr}; }
    static Entry var(const char* n, int8_t& v, bool w = false)   { return Entry{n, Type::I8, &v, w, nullptr, nullptr}; }
    static Entry var(const char* n, uint8_t& v, bool w = false)  { return Entry{n, Type::U8, &v, w, nullptr, nullptr}; }
    static Entry var(const char* n, int16_t& v, bool w = false)  { return Entry{n, Type::I16, &v, w, nullptr, nullptr}; }
    static Entry var(const char* n, uint16_t& v, bool w = false) { return Entry{n, Type::U16, &v, w, nullptr, nullptr}; }
    static Entry var(const char* n, int32_t& v, bool w = false)  { return Entry{n, Type::I32, &v, w, nullptr, nullptr}; }
    static Entry var(const char* n, uint32_t& v, bool w = false) { return Entry{n, Type::U32, &v, w, nullptr, nullptr}; }
    static Entry var(const char* n, float& v, bool w = false)    { return Entry{n, Type::F32, &v, w, nullptr, nullptr}; }
    static Entry var(const char* n, double& v, bool w = false)   { return Entry{n, Type::F64, &v, w, nullptr, nullptr}; }
    static Entry get(const char* n, Getter g, const void* ctx)   { return Entry{n, Type::Getter, nullptr, false, g, ctx}; }

    xTagTable(const Entry* entries, uint16_t count) : entries_(entries), count_(count) {}

    // Called around every read and write of a variable, if given.
    void setLock(void (*lock)(void* ctx), void (*unlock)(void* ctx), void* ctx) {
        lock_ = lock; unlock_ = unlock; lockCtx_ = ctx;
    }

    uint16_t count() const { return count_; }
    const Entry& entry(uint16_t i) const { return entries_[i]; }

    int32_t find(const char* name) override;
    bool read(int32_t handle, xTagValue& out) override;
    bool write(int32_t handle, const xTagValue& value) override;

private:
    const Entry* entries_;
    uint16_t count_;
    void (*lock_)(void*) = nullptr;
    void (*unlock_)(void*) = nullptr;
    void* lockCtx_ = nullptr;
};

} // namespace idisplay
