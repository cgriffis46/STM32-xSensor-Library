// Host tests of the 121GW's LCD as an HMI: the glass map, the layout's
// fields, and meter readings through meter::MeterTags onto the glass.

#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <vector>
#include "Gw121Lcd.h"
#include "Gw121Routes.h"
#include "MeterTags.h"
#include "FakeTags.h"
#include "Gw121Keys.h"

using namespace gw121;
using namespace idisplay;
using meter::Function;

static int g_failures = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_failures++; } } while (0)

static std::string digits(const SegmentFrame& f, const SevenSegDigit* d, const SegRef* points, SegRef minus) {
	std::string s = f.get(minus) ? "-" : "";
	for (int i = 0; i < 5; i++) {
		s += seven::toChar(seven::read(f, d[i]));
		if (i < 4 && f.get(points[i])) s += '.';
	}
	return s;
}
static std::string mainText(const SegmentFrame& f) { return digits(f, lcd::kMain, lcd::kMainPoints, glass::MS_NEGATIVE); }
static std::string subText(const SegmentFrame& f) { return digits(f, lcd::kSub, lcd::kSubPoints, glass::SS_NEGATIVE); }

static void testGlass() {
	printf("glass map\n");
	std::set<std::pair<int, int>> seen;
	for (unsigned i = 0; i < glass::kAllCount; i++) {
		const SegRef s = glass::kAll[i].seg;
		CHECK(s.com < 4 && s.seg <= 40 && s.seg != 26);   // SEG26 is the SD card's
		CHECK(seen.insert({s.com, s.seg}).second);
	}
	CHECK(glass::kAllCount == 160);                       // 4 x (SEG0..40 but SEG26)
}

// Which segments a field writes: drawn on a frame of all off and one of
// all on, the segments it writes come out the same in both.
static std::set<std::pair<int, int>> written(iSegmentField& fld, iTagSource& src) {
	SegmentFrame off, on;
	for (int c = 0; c < 4; c++) for (int s = 0; s <= 40; s++) on.set({(uint8_t)c, (uint8_t)s}, true);
	fld.render(src, off);
	fld.render(src, on);
	std::set<std::pair<int, int>> w;
	for (int c = 0; c < 4; c++) for (int s = 0; s <= 40; s++)
		if (off.get({(uint8_t)c, (uint8_t)s}) == on.get({(uint8_t)c, (uint8_t)s})) w.insert({c, s});
	return w;
}

static void testLayoutCoverage() {
	printf("layout: each segment one field's; which are left\n");
	LcdLayout layout;
	FakeTags t;
	std::set<std::pair<int, int>> all;
	for (uint16_t i = 0; i < layout.count(); i++) {
		CHECK(layout.fields()[i] != nullptr);
		for (auto s : written(*layout.fields()[i], t)) CHECK(all.insert(s).second);
	}
	std::vector<std::string> left;
	for (unsigned i = 0; i < glass::kAllCount; i++)
		if (!all.count({glass::kAll[i].seg.com, glass::kAll[i].seg.seg})) left.push_back(glass::kAll[i].name);
	printf("  %zu of %u segments on fields; not yet:", all.size(), glass::kAllCount);
	for (auto& n : left) printf(" %s", n.c_str());
	printf("\n");
	CHECK(left.size() <= 4);
}

// A meter that just holds a Reading, and records what it is asked.
struct FakeMeter : meter::iMeter {
	meter::Reading r{};
	bool have = false;
	std::vector<std::string> asked;
	const meter::RangeSpec* specs = nullptr;
	bool setFunction(Function f) override { asked.push_back("function " + std::to_string((int)f)); return true; }
	bool setRange(uint8_t x) override { asked.push_back("range " + std::to_string(x)); return true; }
	void setAutoRange(bool on) override { asked.push_back(std::string("auto ") + (on ? "on" : "off")); }
	void hold(bool on) override { asked.push_back(std::string("hold ") + (on ? "on" : "off")); }
	void relative(bool on) override { asked.push_back(std::string("rel ") + (on ? "on" : "off")); }
	void resetMinMax() override {}
	bool latest(meter::Reading* out) const override { if (!have) return false; *out = r; return true; }
	bool supports(Function) const override { return true; }
	uint8_t rangeCount(Function f) const override {
		uint8_t n = 0;
		for (uint8_t i = 0; i < kRouteCount; i++) if (kRoutes[i].function == f) n++;
		return n;
	}
	const meter::RangeSpec* rangeSpec(Function f, uint8_t x) const override {
		for (uint8_t i = 0; i < kRouteCount; i++) if (kRoutes[i].function == f && kRoutes[i].range == x) return &kRoutes[i].spec;
		return nullptr;
	}
	void setListener(meter::iMeterListener*) override {}
	void show(Function f, uint8_t range, float v, uint16_t status) {
		r = meter::Reading{}; r.function = f; r.range = range; r.value = v; r.status = status;
		r.unit = rangeSpec(f, range)->unit; have = true;
	}
};

struct FakeLcd : iSegmentLcd {
	SegmentFrame last; int writes = 0;
	bool write(const SegmentFrame& f) override { last = f; writes++; return true; }
	bool busy() const override { return false; }
};

static void testMeterOnGlass() {
	printf("meter readings on the glass, through MeterTags\n");
	FakeMeter m;
	meter::MeterTags mt(m);
	FakeTags board;
	board.boolean("bt.on", true);
	iTagSource* srcs[] = { &mt, &board };
	xTagChain tags(srcs, 2);
	LcdLayout layout;
	FakeLcd lcd;
	xSegmentHmi hmi(lcd, tags, layout.fields(), layout.count());

	hmi.refresh();
	CHECK(mainText(lcd.last) == "-----");                // no reading yet

	m.show(Function::DcVolts, 0, 1.2345f, meter::kValid | meter::kAutoRange);
	hmi.refresh();
	const SegmentFrame& f = lcd.last;
	CHECK(mainText(f) == "1.2345");
	CHECK(f.get(glass::MS_VOLTS) && f.get(glass::MS_DC) && !f.get(glass::MS_AC) && !f.get(glass::MS_MILLI));
	CHECK(f.get(glass::ICON_AUTO_RANGE) && f.get(glass::ICON_BT) && !f.get(glass::ICON_HOLD));
	int bars = 0; for (auto s : lcd::kBars) bars += f.get(s);
	CHECK(bars == 6 && f.get(glass::BG_POSITIVE));      // 1.2345 / 5 of 25
	CHECK(f.get(glass::BG_RANGE_5) && !f.get(glass::BG_RANGE_0_FOR_50) && !f.get(glass::BG_RANGE_1000));
	CHECK(subText(f) == "-----");                        // nobody publishes sub.value

	m.show(Function::DcVolts, 3, -742.3f, meter::kValid | meter::kHold);
	hmi.refresh();
	CHECK(mainText(lcd.last) == "- 742.3");               // 1000 V range: 1000.0, no prefix
	CHECK(lcd.last.get(glass::ICON_HOLD) && lcd.last.get(glass::BG_RANGE_1000) && lcd.last.get(glass::BG_NEGATIVE));

	m.show(Function::Ohms, 2, 4700.0f, meter::kValid);
	hmi.refresh();
	CHECK(mainText(lcd.last) == "4.7000");                // 5 k range
	CHECK(lcd.last.get(glass::MS_KILO) && lcd.last.get(glass::MS_OHMS) && !lcd.last.get(glass::MS_VOLTS));
	CHECK(!lcd.last.get(glass::MS_DC));

	m.show(Function::DcMilliVolts, 0, 0.0123f, meter::kValid);
	hmi.refresh();
	CHECK(mainText(lcd.last) == "12.300" && lcd.last.get(glass::MS_MILLI));
	CHECK(lcd.last.get(glass::BG_RANGE_5) && lcd.last.get(glass::BG_RANGE_0_FOR_50));

	m.show(Function::DcMicroAmps, 1, 0.000123f, meter::kValid);
	hmi.refresh();
	CHECK(mainText(lcd.last) == "123.00" && lcd.last.get(glass::MS_MICRO) && lcd.last.get(glass::MS_AMPS));

	m.show(Function::AcVolts, 0, 3.0f, meter::kValid);
	hmi.refresh();
	CHECK(lcd.last.get(glass::MS_AC) && !lcd.last.get(glass::MS_DC));

	m.show(Function::DcVolts, 0, 0.0f, meter::kOverload);
	hmi.refresh();
	CHECK(mainText(lcd.last) == "  0L ");
	int b2 = 0; for (auto s : lcd::kBars) b2 += lcd.last.get(s);
	CHECK(b2 == 0);

	m.show(Function::Continuity, 0, 12.3f, meter::kValid);
	hmi.refresh();
	CHECK(lcd.last.get(glass::ICON_CONTINUITY) && !lcd.last.get(glass::ICON_DIODE));

	// The sub display from a board tag.
	board.num("sub.value", 23.4); board.num("sub.decimals", 1); board.num("sub.unit", (double)(uint8_t)meter::Unit::Celsius);
	hmi.refresh();
	CHECK(subText(lcd.last) == "  23.4");

	// Writes to the control tags drive the meter.
	const int32_t hf = tags.find("meter.function"), hh = tags.find("meter.hold"), hr = tags.find("meter.range");
	CHECK(tags.write(hf, xTagValue::num((double)(uint8_t)Function::Ohms)));
	CHECK(tags.write(hh, xTagValue::boolean(true)));
	CHECK(tags.write(hr, xTagValue::num(2)));
	CHECK(!tags.write(hr, xTagValue::num(2.5)));          // not a whole number
	CHECK(!tags.write(tags.find("meter.value"), xTagValue::num(1)));   // read-only
	CHECK(m.asked.size() == 3 && m.asked[0] == "function 6" && m.asked[1] == "hold on" && m.asked[2] == "range 2");
}

static void testFormats() {
	printf("display formats of every 121GW range\n");
	for (uint8_t i = 0; i < kRouteCount; i++) {
		const meter::DisplayFormat d = meter::displayFormat(kRoutes[i].spec, 5);
		const double mant = kRoutes[i].spec.fullScale * d.scale;
		if (!(mant >= 1.0 - 1e-6 && mant <= 1000.0 * (1 + 1e-6))) printf("    %s %s: %.9g\n", meter::functionName(kRoutes[i].function), kRoutes[i].spec.label, mant);
		CHECK(mant >= 1.0 - 1e-6 && mant <= 1000.0 * (1 + 1e-6));
		double room = mant; int intDigits = 1; while (room >= 10.0 - 1e-9) { room /= 10; intDigits++; }
		CHECK(intDigits + d.decimals == 5);
	}
	const meter::RangeSpec v1000{ 1000.0f, 1.0f, 0, meter::Unit::Volt, 1, "1000V" };
	CHECK(meter::displayFormat(v1000).prefix == 0 && meter::displayFormat(v1000).decimals == 1);
}

struct FakeBeeper : iBeeper {
	int oks = 0, nos = 0, last = 0;
	void beep(uint16_t onMs, uint8_t count, uint16_t) override { if (onMs == 50 && count == 2) { nos++; last = -1; } else { oks++; last = 1; } }
	void stop() override {}
	bool beeping() const override { return false; }
};

static void testKeys() {
	printf("keys: pins, and what they do through the meter's tags\n");
	CHECK(keysFromIdr(0xFFFF) == 0);
	CHECK(keysFromIdr((uint16_t)~(1u << 12)) == kKeyRange);   // PG12
	CHECK(keysFromIdr((uint16_t)~(1u << 8)) == kKeyMode);     // PG8
	CHECK(keysFromIdr((uint16_t)~((1u << 13) | (1u << 11))) == (kKeyHold | kKeyMem));

	FakeMeter m;
	meter::MeterTags tags(m);
	FakeBeeper beep;
	Controller c(tags, &beep);
	auto press = [&](uint16_t k, bool longPress = false) {
		KeyEvent e; e.keys = k; e.action = longPress ? KeyEvent::Action::Held : KeyEvent::Action::Pressed;
		m.asked.clear();
		return c.onKey(e);
	};
	m.show(Function::DcVolts, 1, 12.0f, meter::kValid | meter::kAutoRange);
	CHECK(press(kKeyRange) && m.asked.size() == 1 && m.asked[0] == "auto off" && beep.last == 1);
	m.show(Function::DcVolts, 1, 12.0f, meter::kValid);
	CHECK(press(kKeyRange) && m.asked[0] == "range 2");
	m.show(Function::DcVolts, 3, 12.0f, meter::kValid);
	CHECK(press(kKeyRange) && m.asked[0] == "range 0");             // wraps
	CHECK(press(kKeyRange, true) && m.asked[0] == "auto on");
	CHECK(press(kKeyHold) && m.asked[0] == "hold on");
	m.show(Function::DcVolts, 3, 12.0f, meter::kValid | meter::kHold);
	CHECK(press(kKeyHold) && m.asked[0] == "hold off");
	CHECK(press(kKeyRel) && m.asked[0] == "rel on");
	c.setPosition(Position::Volts);
	CHECK(press(kKeyMode) && m.asked[0] == "function 2");           // DC V -> AC V
	c.setPosition(Position::Ohms);
	m.show(Function::Ohms, 2, 100.0f, meter::kValid);
	CHECK(press(kKeyMode) && m.asked[0] == "function 8");           // ohms -> continuity
	const int nos = beep.nos;
	CHECK(!press(kKeyPeak) && m.asked.empty() && beep.nos == nos + 1);   // not yet: "no"
	CHECK(!press(kKeyHold, true));
	// The phone's keys take the same path.
	bt::Command cmd; cmd.kind = bt::Command::Kind::Key; cmd.key = bt::Key::Rel; cmd.longPress = false;
	m.asked.clear();
	CHECK(c.onBluetooth(cmd) && m.asked[0] == "rel on");
}

int main() {
	testGlass();
	testLayoutCoverage();
	testMeterOnGlass();
	testFormats();
	testKeys();
	if (g_failures) { printf("%d FAILED\n", g_failures); return 1; }
	printf("all passed\n");
	return 0;
}
