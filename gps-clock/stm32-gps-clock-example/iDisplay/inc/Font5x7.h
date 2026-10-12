#pragma once
#include <stdint.h>
#include "iTextSurface.h"	// kDegreeChar

namespace idisplay {

// The classic 5x7 font, ASCII 0x20 to 0x7E, plus a degree sign at 0x7F
// (kDegreeChar).
// Each glyph is 5 columns, left to right; bit 0 is the top pixel.
// Drawn in a 6x8 cell (one blank column, one blank row), which gives
// 21 x 8 characters on a 128x64 display and 21 x 4 on a 128x32.
static constexpr uint8_t kFontWidth = 5;
static constexpr uint8_t kFontHeight = 7;

// The glyph for c. Characters outside the font draw as '?'.
const uint8_t* fontGlyph(char c);

} // namespace idisplay
