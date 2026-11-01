// Host test of Stm32L1Lcd against fake LCD registers.
#include <cstdio>
#include "Stm32L1Lcd.h"

LCD_TypeDef g_fakeLcd;
using namespace idisplay;

static int g_failures = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_failures++; } } while (0)

int main() {
	printf("Stm32L1Lcd: set-up as the 121GW's, frame to RAM, busy while UDR\n");
	Stm32L1Lcd lcd;
	CHECK(!lcd.start(Stm32L1Lcd::Config{}));        // nothing answers: false, not stuck
	g_fakeLcd.SR = LCD_SR_ENS | LCD_SR_RDY | LCD_SR_FCRSR;
	CHECK(lcd.start(Stm32L1Lcd::Config{}));
	CHECK(g_fakeLcd.CR == (LCD_CR_LCDEN | (3u << 2) | (2u << 5)));   // 1/4 duty, 1/3 bias, internal
	CHECK(g_fakeLcd.FCR == ((3u << 22) | (4u << 10) | (4u << 4)));   // PS 3, DIV 0, CC 4, PON 4
	SegmentFrame f;
	f.set({0, 0}, true); f.set({0, 31}, true); f.set({1, 32}, true); f.set({3, 40}, true); f.set({2, 26}, true);
	CHECK(lcd.write(f));
	CHECK(g_fakeLcd.RAM[0] == 0x80000001u && g_fakeLcd.RAM[1] == 0);
	CHECK(g_fakeLcd.RAM[3] == 1u);                   // COM1 SEG32
	CHECK(g_fakeLcd.RAM[4] == (1u << 26));
	CHECK(g_fakeLcd.RAM[7] == (1u << 8));            // COM3 SEG40
	CHECK(lcd.busy());                               // UDR was written; the "hardware" hasn't cleared it
	SegmentFrame g;
	CHECK(!lcd.write(g) && g_fakeLcd.RAM[0] == 0x80000001u);   // locked: untouched
	g_fakeLcd.SR &= ~LCD_SR_UDR;
	CHECK(lcd.write(g) && g_fakeLcd.RAM[0] == 0);
	lcd.setContrast(7);
	CHECK(((g_fakeLcd.FCR >> 10) & 7u) == 7u && ((g_fakeLcd.FCR >> 22) & 0xFu) == 3u);
	if (g_failures) { printf("%d FAILED\n", g_failures); return 1; }
	printf("all passed\n");
	return 0;
}
