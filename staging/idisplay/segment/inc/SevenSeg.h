#pragma once
#include <stdint.h>
#include "SegmentFrame.h"

namespace idisplay {

// A 7-segment digit on a glass: where its A..G strokes are.
//
//      A
//    F   B
//      G
//    E   C
//      D
struct SevenSegDigit {
    SegRef a, b, c, d, e, f, g;
};

// Strokes as bits: bit 0 A .. bit 6 G.
namespace seven {
static constexpr uint8_t A = 1, B = 2, C = 4, D = 8, E = 16, F = 32, G = 64;

// What a character looks like. Digits, the letters a 7-segment digit can
// show (upper or lower case asked for, the shape that reads best), and
// a few signs. Anything else is blank.
inline uint8_t glyph(char ch) {
    switch (ch) {
    case '0': return A | B | C | D | E | F;
    case '1': return B | C;
    case '2': return A | B | D | E | G;
    case '3': return A | B | C | D | G;
    case '4': return B | C | F | G;
    case '5': return A | C | D | F | G;
    case '6': return A | C | D | E | F | G;
    case '7': return A | B | C;
    case '8': return A | B | C | D | E | F | G;
    case '9': return A | B | C | D | F | G;
    case 'A': case 'a': return A | B | C | E | F | G;
    case 'b': case 'B': return C | D | E | F | G;
    case 'C': return A | D | E | F;
    case 'c': return D | E | G;
    case 'd': case 'D': return B | C | D | E | G;
    case 'E': case 'e': return A | D | E | F | G;
    case 'F': case 'f': return A | E | F | G;
    case 'G': case 'g': return A | C | D | E | F;
    case 'H': return B | C | E | F | G;
    case 'h': return C | E | F | G;
    case 'I': return E | F;
    case 'i': return E;
    case 'J': case 'j': return B | C | D | E;
    case 'L': case 'l': return D | E | F;
    case 'n': case 'N': return C | E | G;
    case 'O': return A | B | C | D | E | F;
    case 'o': return C | D | E | G;
    case 'P': case 'p': return A | B | E | F | G;
    case 'q': case 'Q': return A | B | C | F | G;
    case 'r': case 'R': return E | G;
    case 'S': case 's': return A | C | D | F | G;
    case 't': case 'T': return D | E | F | G;
    case 'U': return B | C | D | E | F;
    case 'u': case 'v': case 'V': return C | D | E;
    case 'y': case 'Y': return B | C | D | F | G;
    case '-': return G;
    case '_': return D;
    case '=': return D | G;
    case '\'': return F;
    case '"': return B | F;
    case '[': return A | D | E | F;
    case ']': return A | B | C | D;
    case '?': return A | B | E | G;
    default:  return 0;
    }
}

inline void draw(SegmentFrame& fr, const SevenSegDigit& d, uint8_t strokes) {
    fr.set(d.a, strokes & A); fr.set(d.b, strokes & B); fr.set(d.c, strokes & C);
    fr.set(d.d, strokes & D); fr.set(d.e, strokes & E); fr.set(d.f, strokes & F);
    fr.set(d.g, strokes & G);
}

// The strokes lit for a digit on a frame, back as bits (for tests and a
// text dump of the glass).
inline uint8_t read(const SegmentFrame& fr, const SevenSegDigit& d) {
    return (uint8_t)((fr.get(d.a) ? A : 0) | (fr.get(d.b) ? B : 0) | (fr.get(d.c) ? C : 0) |
                     (fr.get(d.d) ? D : 0) | (fr.get(d.e) ? E : 0) | (fr.get(d.f) ? F : 0) |
                     (fr.get(d.g) ? G : 0));
}

// The character a stroke pattern shows best, for a text dump: digits
// first, then letters, '?' if none.
inline char toChar(uint8_t strokes) {
    if (strokes == 0) return ' ';
    static const char order[] = "0123456789-AbCcdEFHhLnoPrStUuyJ_=";
    for (const char* p = order; *p; ++p) if (glyph(*p) == strokes) return *p;
    return '?';
}
} // namespace seven

} // namespace idisplay
