// Host test of the LCD bring-up app: the screens, keys and contrast, on a
// fake LCD and buzzer.
#include <cstdio>
#include <string>
#include "LcdBringup.h"
#include "Gw121Simulator.h"

using namespace gw121;
using idisplay::KeyEvent;

static int g_failures = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_failures++; } } while (0)

struct FakeLcd : idisplay::iSegmentLcd {
	idisplay::SegmentFrame f;
	int writes = 0;
	bool isBusy = false;
	bool write(const idisplay::SegmentFrame& x) override { if (isBusy) return false; f = x; writes++; return true; }
	bool busy() const override { return isBusy; }
	unsigned lit() const { unsigned n = 0; for (uint8_t c = 0; c < 4; c++) for (uint8_t s = 0; s < 64; s++) n += f.get({c, s}); return n; }
};
struct FakeBeeper : idisplay::iBeeper {
	int oks = 0, nos = 0, beeps = 0;
	void beep(uint16_t onMs, uint8_t count, uint16_t) override { beeps++; if (count == 2) nos++; else if (onMs == 100) oks++; }
	void stop() override {}
	bool beeping() const override { return false; }
};
struct FakeDiag : iFrontEndDiag {
	FrontEndStatus st{};
	void status(FrontEndStatus* out) override { *out = st; }
};
struct FakeContrast : iLcdContrast { int cc = -1; void setContrast(uint8_t c) override { cc = c; } };

static KeyEvent key(uint16_t k, KeyEvent::Action a = KeyEvent::Action::Pressed) { KeyEvent e; e.keys = k; e.action = a; return e; }

// The main display as text, from the frame (for the HMI and the walk's index).
static std::string text(const idisplay::SegmentFrame& f, const idisplay::SevenSegDigit* d) {
	std::string s;
	for (int i = 0; i < 5; i++) { const uint8_t st = idisplay::seven::read(f, d[i]); s += st ? idisplay::seven::toChar(st) : ' '; }
	return s;
}

int main() {
	printf("LCD bring-up: screens, walk, contrast, HMI\n");
	FakeLcd lcd; FakeBeeper beep; FakeContrast con; FakeDiag diag;
	SimFrontEnd fe;
	LcdBringup app(lcd, beep, fe, nullptr, &con, &diag);
	uint32_t t = 0;
	auto run = [&](uint32_t ms) { for (uint32_t end = t + ms; t < end; t += 10) { fe.tick(t); app.main(t); } };

	app.start();
	CHECK(con.cc == 4 && beep.beeps == 1);
	run(50);
	CHECK(app.screen() == LcdBringup::Screen::All);
	CHECK(lcd.lit() == glass::kAllCount);                  // every glass segment, and nothing else

	// A busy LCD: the frame waits, then goes.
	app.onKey(key(kKeySetup));
	lcd.isBusy = true; run(30); const int w = lcd.writes;
	lcd.isBusy = false; run(30);
	CHECK(lcd.writes > w);
	CHECK(app.screen() == LcdBringup::Screen::Digits);
	CHECK(text(lcd.f, lcd::kMain) == "00000" && text(lcd.f, lcd::kSub) == "00000");
	run(LcdBringup::kDigitStepMs);
	CHECK(text(lcd.f, lcd::kMain) == "11111" && lcd.f.get(glass::MS_NEGATIVE));
	run(LcdBringup::kDigitStepMs * 9);
	CHECK(text(lcd.f, lcd::kMain) == "88888" && lcd.f.get(glass::MS_POINT_d0) && lcd.f.get(glass::SS_POINT_d0000));

	// Walk: one segment plus its index.
	app.onKey(key(kKeySetup)); run(20);
	CHECK(app.screen() == LcdBringup::Screen::Walk && app.walkIndex() == 0);
	CHECK(lcd.f.get(glass::kAll[0].seg));
	app.onKey(key(kKeyRange)); run(20);
	CHECK(app.walkIndex() == 1 && lcd.f.get(glass::kAll[1].seg) && !lcd.f.get(glass::kAll[0].seg));
	app.onKey(key(kKeyRange, KeyEvent::Action::Held)); run(20);
	CHECK(app.walkIndex() == 11);
	app.onKey(key(kKeyRel, KeyEvent::Action::Held)); app.onKey(key(kKeyRel, KeyEvent::Action::Held)); run(20);
	CHECK(app.walkIndex() == glass::kAllCount - 9);        // wraps
	// Every segment: it is lit, and the index reads back from the other display.
	bool allOk = true;
	for (uint16_t i = 0; i < glass::kAllCount; i++) {
		while (app.walkIndex() != i) app.onKey(key(kKeyRange));
		run(10);
		const bool sub = LcdBringup::onSubDisplay(i);
		std::string s = text(lcd.f, sub ? lcd::kMain : lcd::kSub);
		const std::string want = std::to_string(i);
		const std::string shown = s.substr(s.find_first_not_of(' '));
		// The lit segment may be a stroke of the display the index is on only
		// if that display isn't the sub one: a main-display stroke adds to the
		// sub's index never, and a sub stroke lands on the main display never.
		if (!lcd.f.get(glass::kAll[i].seg) || shown != want) { allOk = false; printf("  segment %u (%s): shown '%s'\n", i, glass::kAll[i].name, s.c_str()); break; }
	}
	CHECK(allOk);

	// Contrast, on any screen.
	const int nos = beep.nos;
	for (int i = 0; i < 5; i++) app.onKey(key(kKeyPeak));
	CHECK(con.cc == 7 && beep.nos == nos + 2);             // 5, 6, 7, then no, no
	for (int i = 0; i < 8; i++) app.onKey(key(kKeyMem));
	CHECK(con.cc == 0 && app.contrast() == 0);

	// HMI: the meter's reading on the glass, the rotary switch on the sub display.
	app.onKey(key(kKeySetup)); run(2000);
	CHECK(app.screen() == LcdBringup::Screen::Hmi);
	CHECK(lcd.f.get(glass::MS_VOLTS) && lcd.f.get(glass::MS_DC));
	CHECK(text(lcd.f, lcd::kMain).find('1') != std::string::npos);   // 1.2xxx V
	app.onPosition(Position::Ohms); run(2000);
	CHECK(lcd.f.get(glass::MS_OHMS) && !lcd.f.get(glass::MS_VOLTS));
	CHECK(text(lcd.f, lcd::kSub) == "    4");
	app.onKey(key(kKeyHold)); run(300);
	CHECK(lcd.f.get(glass::ICON_HOLD));
	app.onKey(key(kKeyMode)); run(2000);
	CHECK(lcd.f.get(glass::ICON_CONTINUITY));

	// HY ("5t" is how "St" reads back): the driver's state and faults until it runs, then AD1 / 256 and the sample count.
	app.onKey(key(kKeySetup)); diag.st = FrontEndStatus{ false, 4, 2, 0, 0 }; run(200);
	CHECK(app.screen() == LcdBringup::Screen::Hy);
	CHECK(text(lcd.f, lcd::kMain) == "5t  4" && text(lcd.f, lcd::kSub) == "    2");
	diag.st = FrontEndStatus{ true, 7, 2, 123456, -256 * 1234 }; run(200);
	CHECK(text(lcd.f, lcd::kMain) == " 1234" && lcd.f.get(glass::MS_NEGATIVE));
	CHECK(text(lcd.f, lcd::kSub) == "23456" && !lcd.f.get(glass::SS_NEGATIVE));

	// Round again: SETUP from HY goes back to ALL, and draws it.
	app.onKey(key(kKeySetup)); run(20);
	CHECK(app.screen() == LcdBringup::Screen::All && lcd.lit() == glass::kAllCount);
	// And the HMI draws again on coming back, though its own frame is unchanged.
	for (int i = 0; i < 3; i++) app.onKey(key(kKeySetup));
	run(200);
	CHECK(lcd.f.get(glass::MS_OHMS) && lcd.lit() < glass::kAllCount);

	// Without a diag, SETUP goes from HMI straight to ALL.
	{
		FakeLcd l2; FakeBeeper b2; SimFrontEnd f2;
		LcdBringup a2(l2, b2, f2);
		a2.start();
		for (int i = 0; i < 3; i++) a2.onKey(key(kKeySetup));
		CHECK(a2.screen() == LcdBringup::Screen::Hmi);
		a2.onKey(key(kKeySetup));
		CHECK(a2.screen() == LcdBringup::Screen::All);
	}

	if (g_failures) { printf("%d FAILED\n", g_failures); return 1; }
	printf("all passed\n");
	return 0;
}
