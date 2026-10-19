#include "xTagTable.h"
#include <float.h>
#include <math.h>
#include <string.h>

namespace idisplay {

int32_t xTagTable::find(const char* name) {
    if (name == nullptr) return -1;
    for (uint16_t i = 0; i < count_; ++i)
        if (entries_[i].name && strcmp(entries_[i].name, name) == 0) return i;
    return -1;
}

template <typename T>
static double load(const void* p) { T v; memcpy(&v, p, sizeof v); return (double)v; }

bool xTagTable::read(int32_t h, xTagValue& out) {
    if (h < 0 || h >= count_) return false;
    const Entry& e = entries_[h];
    if (e.type == Type::Getter) {
        if (e.getter == nullptr) return false;
        out = xTagValue::num(e.getter(e.ctx));
        return true;
    }
    if (e.ptr == nullptr) return false;
    if (lock_) lock_(lockCtx_);
    double v = 0;
    switch (e.type) {
    case Type::Bool: v = *static_cast<const bool*>(e.ptr) ? 1 : 0; break;
    case Type::I8:   v = load<int8_t>(e.ptr); break;
    case Type::U8:   v = load<uint8_t>(e.ptr); break;
    case Type::I16:  v = load<int16_t>(e.ptr); break;
    case Type::U16:  v = load<uint16_t>(e.ptr); break;
    case Type::I32:  v = load<int32_t>(e.ptr); break;
    case Type::U32:  v = load<uint32_t>(e.ptr); break;
    case Type::F32:  v = load<float>(e.ptr); break;
    case Type::F64:  v = load<double>(e.ptr); break;
    case Type::Getter: break;
    }
    if (unlock_) unlock_(lockCtx_);
    out = e.type == Type::Bool ? xTagValue::boolean(v != 0) : xTagValue::num(v);
    return true;
}

// A whole number from lo to hi, stored as T.
template <typename T>
static bool storeInt(void* p, double x, double lo, double hi) {
    if (x != floor(x) || x < lo || x > hi) return false;
    const T v = (T)x;
    memcpy(p, &v, sizeof v);
    return true;
}

bool xTagTable::write(int32_t h, const xTagValue& value) {
    if (h < 0 || h >= count_ || value.kind == xTagValue::Kind::None) return false;
    const Entry& e = entries_[h];
    if (!e.writable || e.ptr == nullptr || e.type == Type::Getter) return false;
    const double x = value.kind == xTagValue::Kind::Bool ? (value.b ? 1.0 : 0.0) : value.number;
    if (!isfinite(x)) return false;
    // Checked before taking the lock: only the store is under it.
    switch (e.type) {
    case Type::Bool: if (x != 0 && x != 1) return false; break;
    case Type::F32:  if (fabs(x) > (double)FLT_MAX) return false; break;
    default: break;
    }
    if (lock_) lock_(lockCtx_);
    bool ok = true;
    switch (e.type) {
    case Type::Bool: *static_cast<bool*>(e.ptr) = x != 0; break;
    case Type::I8:   ok = storeInt<int8_t>(e.ptr, x, -128.0, 127.0); break;
    case Type::U8:   ok = storeInt<uint8_t>(e.ptr, x, 0, 255.0); break;
    case Type::I16:  ok = storeInt<int16_t>(e.ptr, x, -32768.0, 32767.0); break;
    case Type::U16:  ok = storeInt<uint16_t>(e.ptr, x, 0, 65535.0); break;
    case Type::I32:  ok = storeInt<int32_t>(e.ptr, x, -2147483648.0, 2147483647.0); break;
    case Type::U32:  ok = storeInt<uint32_t>(e.ptr, x, 0, 4294967295.0); break;
    case Type::F32:  { const float f = (float)x; memcpy(e.ptr, &f, sizeof f); break; }
    case Type::F64:  memcpy(e.ptr, &x, sizeof x); break;
    case Type::Getter: ok = false; break;
    }
    if (unlock_) unlock_(lockCtx_);
    return ok;
}

} // namespace idisplay
