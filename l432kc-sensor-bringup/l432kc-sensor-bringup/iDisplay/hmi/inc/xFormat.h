#pragma once
#include <stddef.h>
#include <stdint.h>

namespace idisplay {

// A number as the HMI and the labels show it, without printf (newlib-nano
// leaves out %f): rounded to decimals (0-6) half away from zero, never
// "-0", into out, ending in 0. Returns its length; 0 (and "--") when v is
// NaN or infinite; kFormatTooLong when it doesn't fit in cap - 1
// characters or is 1e15 or more once scaled by its decimals. Defined in
// xHmi.cpp (idisplay_hmi).
static constexpr size_t kFormatTooLong = (size_t)-1;
size_t formatNumber(double v, uint8_t decimals, char* out, size_t cap);

} // namespace idisplay
