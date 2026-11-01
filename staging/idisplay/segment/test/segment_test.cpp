// Host tests of the segment LCD HMI: fields, xSegmentHmi, xTagChain.

#include <cstdio>
#include <string>
#include "SegmentFields.h"
#include "xSegmentHmi.h"
#include "FakeTags.h"

using namespace idisplay;

static int g_failures = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_failures++; } } while (0)

// A made-up glass: 5 digits on COM 0..3, points, a minus, 10 bars, 3 icons.
static SevenSegDigit D(uint8_t i) {
	const uint8_t s = (uint8_t)(i * 2);
	return SevenSegDigit{ {0, s}, {1, s}, {2, s}, {3, s}, {0, (uint8_t)(s + 1)}, {1, (uint8_t)(s + 1)}, {2, (uint8_t)(s + 1)} };
}
static const SevenSegDigit kDigits[5] = { D(0), D(1), D(2), D(3), D(4) };
static const SegRef kPoints[4] = { {3, 1}, {3, 3}, {3, 5}, {3, 7} };
static const SegRef kMinus = {3, 9};
static SegRef bar(uint8_t i) { return SegRef{ (uint8_t)(i % 4), (uint8_t)(20 + i / 4) }; }
static const SegRef kBars[10] = { bar(0), bar(1), bar(2), bar(3), bar(4), bar(5), bar(6), bar(7), bar(8), bar(9) };

// The digits as text, '.' where a point is lit, '-' first if the minus is.
static std::string shown(const SegmentFrame& f) {
	std::string s = f.get(kMinus) ? "-" : "";
	for (int i = 0; i < 5; i++) {
		s += seven::toChar(seven::read(f, kDigits[i]));
		if (i < 4 && f.get(kPoints[i])) s += '.';
	}
	return s;
}

static std::string render(NumberField& nf, FakeTags& t) {
	SegmentFrame f;
	nf.render(t, f);
	return shown(f);
}

static void testNumber() {
	printf("NumberField\n");
	FakeTags t;
	NumberField nf(kDigits, 5, kPoints, kMinus, "v", "dec", 0, "ol");
	CHECK(render(nf, t) == "-----");                       // no tag yet
	t.num("v", 1.2345); t.num("dec", 4);
	CHECK(render(nf, t) == "1.2345");
	t.num("v", 0.0005);
	CHECK(render(nf, t) == "0.0005");
	t.num("v", 3.2); t.num("dec", 1);
	CHECK(render(nf, t) == "   3.2");
	t.num("v", -12.34);
	CHECK(render(nf, t) == "-  12.3");
	t.num("v", -0.04);                                      // rounds to 0.0: no minus
	CHECK(render(nf, t) == "   0.0");
	t.num("v", 1.99996); t.num("dec", 4);
	CHECK(render(nf, t) == "2.0000");
	t.num("v", 123456); t.num("dec", 0);
	std::string ol = render(nf, t);
	CHECK(ol == "  0L ");                                   // "  OL ": O shows as 0
	t.num("v", 7); t.boolean("ol", true);
	CHECK(render(nf, t) == "  0L ");
	t.boolean("ol", false);
	t.num("v", NAN);
	CHECK(render(nf, t) == "-----");
	t.num("v", 42); t.unreadable("v");
	CHECK(render(nf, t) == "-----");
	NumberField fixed(kDigits, 5, kPoints, kMinus, "w", nullptr, 2);
	t.num("w", 9.5);
	CHECK(render(fixed, t) == "  9.50");
}

static void testIconsBarsChoice() {
	printf("IconField, BarField, ChoiceField\n");
	FakeTags t;
	SegmentFrame f;
	IconField on{ {0, 30}, "flag" }, eq{ {1, 30}, "unit", 2 }, rng{ {2, 30}, "r", 1, 2 };
	IconField always = IconField::always({3, 30});
	auto draw = [&]() { f.clear(); on.render(t, f); eq.render(t, f); rng.render(t, f); always.render(t, f); };
	draw();
	CHECK(!f.get({0, 30}) && !f.get({1, 30}) && !f.get({2, 30}) && f.get({3, 30}));
	t.boolean("flag", true); t.num("unit", 2); t.num("r", 1);
	draw();
	CHECK(f.get({0, 30}) && f.get({1, 30}) && f.get({2, 30}));
	t.num("unit", 3); t.num("r", 3); t.num("flag", 0);
	draw();
	CHECK(!f.get({0, 30}) && !f.get({1, 30}) && !f.get({2, 30}));

	BarField b(kBars, 10, "bar", 1.0, {0, 40}, {1, 40}, "en");
	auto lit = [&]() { int n = 0; for (auto s : kBars) n += f.get(s); return n; };
	t.num("bar", 0.55); t.boolean("en", true);
	f.clear(); b.render(t, f);
	CHECK(lit() == 5 && f.get({0, 40}) && !f.get({1, 40}));
	t.num("bar", -1.5);
	f.clear(); b.render(t, f);
	CHECK(lit() == 10 && !f.get({0, 40}) && f.get({1, 40}));
	t.boolean("en", false);
	f.clear(); b.render(t, f);
	CHECK(lit() == 0 && !f.get({1, 40}));

	static const ChoiceField::Choice kC[] = { {0, "OFF"}, {1, "Auto"} };
	ChoiceField c(kDigits, 5, "mode", kC, 2, "----");
	f.clear(); c.render(t, f);
	CHECK(shown(f) == "---- ");
	t.num("mode", 1);
	f.clear(); c.render(t, f);
	CHECK(shown(f) == "AUt0 " || shown(f) == "Auto ");   // 'u', 'o' read back by shape
}

struct FakeLcd : iSegmentLcd {
	int writes = 0;
	bool isBusy = false;
	SegmentFrame last;
	bool write(const SegmentFrame& f) override { if (isBusy) return false; last = f; writes++; return true; }
	bool busy() const override { return isBusy; }
};

static void testHmiAndChain() {
	printf("xSegmentHmi: writes on change only, waits out a busy LCD; xTagChain\n");
	FakeTags meterTags, boardTags;
	meterTags.num("meter.value", 1.5); meterTags.num("meter.decimals", 1);
	boardTags.boolean("bt.on", true);
	iTagSource* srcs[] = { &meterTags, &boardTags };
	xTagChain chain(srcs, 2);
	NumberField nf(kDigits, 5, kPoints, kMinus, "meter.value", "meter.decimals");
	IconField bt{ {0, 31}, "bt.on" };
	iSegmentField* const fields[] = { &nf, &bt };
	FakeLcd lcd;
	xSegmentHmi hmi(lcd, chain, fields, 2);
	CHECK(hmi.refresh() && lcd.writes == 1);
	CHECK(shown(lcd.last) == "   1.5" && lcd.last.get({0, 31}));
	CHECK(!hmi.refresh() && lcd.writes == 1);              // nothing changed
	meterTags.num("meter.value", 2.5);
	lcd.isBusy = true;
	CHECK(!hmi.refresh() && lcd.writes == 0 + 1);
	lcd.isBusy = false;
	CHECK(hmi.refresh() && lcd.writes == 2 && shown(lcd.last) == "   2.5");
	hmi.setTestPattern(true);
	CHECK(hmi.refresh() && lcd.last.get({7, 63}));
	// chain: the second source answers, writes go back to it
	const int32_t h = chain.find("bt.on");
	CHECK(h >= 0 && chain.write(h, xTagValue::boolean(false)));
	CHECK(boardTags.writes.size() == 1 && boardTags.writes[0].first == "bt.on");
	CHECK(chain.find("nope") < 0);
}

int main() {
	testNumber();
	testIconsBarsChoice();
	testHmiAndChain();
	if (g_failures) { printf("%d FAILED\n", g_failures); return 1; }
	printf("all passed\n");
	return 0;
}
