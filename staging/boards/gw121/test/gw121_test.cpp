// Host tests of the 121GW board package: the route table, the stock
// calibration in EEPROM, the analog switch rules, the rotary switch, and
// the lot end to end (Meter, Hy3131FrontEnd, hy3131_dmm, the simulated
// HY3131) with the meter's own register images.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include "Meter.h"
#include "Gw121Routes.h"
#include "Gw121Calibration.h"
#include "Gw121Paths.h"
#include "Gw121RangeSwitch.h"
#include "SimHy3131.h"

using namespace meter;
using namespace gw121;

static int g_failures = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_failures++; } } while (0)
static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

static void loadRoute(const Hy3131Route& rt, hy3131::Config& c) { c = hy3131::defaultConfig(); rt.build(c); }

static void testRoutes() {
	printf("route table\n");
	for (uint8_t i = 0; i < kRouteCount; i++) {
		const Hy3131Route& rt = kRoutes[i];
		hy3131::Config c;
		loadRoute(rt, c);
		CHECK(c.r(hy3131::kR33) & hy3131::kR33Enosc);    // oscillator on
		CHECK(c.r(hy3131::kR31) & hy3131::kR31Enbias);   // bias on
		CHECK(c.r(hy3131::kR34) == 0 && c.r(hy3131::kR36) == 0);
		CHECK(rt.spec.unitsPerRaw > 0.0f && rt.spec.fullScale > 0.0f);
		if (rt.source == Hy3131Source::Ad1) {
			CHECK(c.ad1On());
			CHECK(rt.spec.fullScale / rt.spec.unitsPerRaw < 8388607.0f);   // fits AD1
		}
		// ranges 0..n-1, each once
		uint8_t n = 0;
		for (uint8_t j = 0; j < kRouteCount; j++) if (kRoutes[j].function == rt.function) n++;
		CHECK(rt.range < n);
		for (uint8_t j = 0; j < kRouteCount; j++) if (j != i) CHECK(!(kRoutes[j].function == rt.function && kRoutes[j].range == rt.range));
		CHECK(StockCalibration::find(rt.function, rt.range) != nullptr);
	}
	// spot checks against the stock tables
	hy3131::Config c;
	loadRoute(kRoutes[0], c);
	CHECK(kRoutes[0].function == Function::DcVolts && c.r(0x22) == 0x13 && c.r(0x23) == 0x8A && c.r(0x30) == 0x80);
	CHECK(c.gateCycles == 0x200000u);
}

struct VecEeprom : iByteReader {
	std::vector<uint8_t> m = std::vector<uint8_t>(12 * 1024, 0);
	bool read(uint32_t off, uint8_t* d, size_t n) override {
		if (off + n > m.size()) return false;
		memcpy(d, &m[off], n);
		return true;
	}
	void put32(uint32_t off, uint32_t v) { m[off] = v >> 24; m[off + 1] = v >> 16; m[off + 2] = v >> 8; m[off + 3] = (uint8_t)v; }
	void putCal(Function f, uint8_t r, int32_t offset, double g) {
		const RangeCal* rc = StockCalibration::find(f, r);
		const uint32_t a = kCalBase[rc->stockMode] + 16u * rc->stockRange;
		put32(a, (uint32_t)offset);
		put32(a + 4, (uint32_t)std::lround(g * 1e6));
	}
};

static void testCalibration() {
	printf("stock calibration from the data EEPROM\n");
	VecEeprom ee;
	StockCalibration cal(ee);
	Calibration c;
	CHECK(!cal.load(Function::DcVolts, 0, &c));                 // erased: 0s
	ee.putCal(Function::DcVolts, 0, 86, 60.774640);            // the values at 0x70 in a real 121GW's dump
	CHECK(cal.load(Function::DcVolts, 0, &c) && c.valid);
	// The stock firmware: digits = (AD1 - 86) / 60.77464. 50,000 digits = 5.0000 V.
	const double raw = 86 + 50000 * 60.774640;
	const double v = (raw - c.offset) * c.gain * kRoutes[0].spec.unitsPerRaw;
	CHECK(near(v, 5.0, 1e-4));
	CHECK(kCalBase[kMM_DCV] == 0x70 && kCalBase[kMM_OHMS] + 16 * 6 == 0x170 && kCalBase[kMM_DCuA] == 0x210);
	ee.putCal(Function::DcVolts, 1, 0, 30.0);                  // half the nominal: garbage
	CHECK(!cal.load(Function::DcVolts, 1, &c));
	ee.put32(kCalBase[kMM_HZ] + 4, 49152500u);                  // SYSCLK 4,915,250 Hz
	CHECK(cal.load(Function::Frequency, 0, &c) && near(c.gain, 4915250.0 / 4915200.0, 1e-7));
	CHECK(!cal.store(Function::DcVolts, 0, c));                 // never written
	// Continuity shares the 500 ohm range's factors, as in the stock firmware.
	CHECK(kCalBase[kMM_CONT] == kCalBase[kMM_OHMS] + 16);
}

static void testPaths() {
	printf("analog switch rules\n");
	Levels l = pathLevels(Function::DcVolts);
	CHECK(l.written(CTL_A) && l.isHigh(CTL_A) && l.isHigh(CTL_E) && !l.written(CTL_D));
	CHECK(l.isHigh(CTL_B) && l.isHigh(FRE_CTL) && l.isHigh(LED_CK) && !l.isHigh(ENB) && !l.isHigh(TEMP_CTL));
	l = pathLevels(Function::AcVolts);
	CHECK(!l.isHigh(CTL_A) && !l.isHigh(CTL_E) && l.isHigh(CTL_D) && !l.isHigh(DCmV_CTL));
	l = pathLevels(Function::AcVolts, true);
	CHECK(!l.isHigh(CTL_A) && l.isHigh(CTL_E) && !l.isHigh(CTL_D));
	l = pathLevels(Function::AcMilliAmps);
	CHECK(!l.isHigh(CTL_A) && l.isHigh(DCmV_CTL));
	CHECK(!pathLevels(Function::Ohms).isHigh(CTL_B));
	CHECK(pathLevels(Function::Continuity).isHigh(CTL_B));
	CHECK(!pathLevels(Function::Frequency).isHigh(FRE_CTL));
	CHECK(pathLevels(Function::Temperature).isHigh(TEMP_CTL));
	l = pathLevels(Function::Diode, false, true);
	CHECK(l.isHigh(ENB) && !l.isHigh(LED_CK));

	struct FakeGpio { uint16_t high = 0, touched = 0; void set(Signal s, bool h) { touched |= 1u << s; if (h) high |= 1u << s; else high &= ~(1u << s); } };
	Paths<FakeGpio> p;
	p.select(Function::Ohms, 2);
	CHECK(!(p.gpio().touched & (1u << CTL_D)));
	CHECK(!(p.gpio().high & (1u << CTL_B)) && (p.gpio().high & (1u << CTL_A)));
}

static void testRangeSwitch() {
	printf("rotary switch\n");
	RangeSwitch sw;
	Position p;
	auto idr = [](int pin) { return (uint16_t)(0xFFFF & ~(1u << pin)); };   // one contact closed
	for (int i = 0; i < 4; i++) sw.sample(idr(6));
	CHECK(!sw.poll(&p));                                      // not stable long enough
	sw.sample(idr(6)); sw.sample(idr(6));
	CHECK(sw.poll(&p) && p == Position::Volts);              // PG6: V, in the stock firmware's order
	CHECK(!sw.poll(&p));                                      // once
	for (int i = 0; i < 20; i++) sw.sample((uint16_t)(idr(6) & idr(5)));   // between two positions
	CHECK(!sw.poll(&p));
	for (int i = 0; i < 20; i++) sw.sample(idr(3));
	CHECK(sw.poll(&p) && p == Position::Ohms);
	CHECK(firstFunction(Position::Ohms) == Function::Ohms);
	CHECK(nextFunction(Position::Ohms, Function::Diode) == Function::Capacitance);
	RangeSwitch schematic(kSchematicPinOf);
	for (int i = 0; i < 10; i++) schematic.sample(idr(6));
	CHECK(schematic.poll(&p) && p == Position::MicroAmps);
}

// ---- end to end ----

using namespace hy3131sim;
using Dmm = hy3131_dmm<SimTransport>;

static void testEndToEnd() {
	printf("end to end: DC volts with the 121GW's images and a factory calibration\n");
	SimChip chip;
	hy3131_param_t prm = hy3131_default_param();
	prm.doLevel = doLevel;
	prm.doCtx = &chip;
	Dmm dmm(prm, &chip);
	struct FakeGpio { uint16_t high = 0; void set(Signal s, bool h) { if (h) high |= 1u << s; else high &= ~(1u << s); } };
	Paths<FakeGpio> paths;
	Hy3131FrontEnd<Dmm> fe(dmm, kRoutes, kRouteCount, hy3131::kXtalHz, &paths);
	VecEeprom ee;
	// This meter's own factors: 2 % off the nominal, offset 100 counts.
	for (uint8_t r = 0; r < 4; r++) {
		const RangeCal* rc = StockCalibration::find(Function::DcVolts, r);
		ee.putCal(Function::DcVolts, r, 100, rc->nominalCountsPerDigit * 1.02);
	}
	StockCalibration cal(ee);
	Meter m(fe, &cal);

	double vin = 1.234;
	// The chip: which DC volts range is loaded, and AD1 as the meter's would read.
	chip.ad1Model = [&vin](const uint8_t* reg) -> int32_t {
		for (uint8_t i = 0; i < kRouteCount; i++) {
			if (kRoutes[i].function != Function::DcVolts) continue;
			hy3131::Config c; c = hy3131::defaultConfig(); kRoutes[i].build(c);
			if (memcmp(c.reg, &reg[0x20], 20) != 0) continue;
			const RangeCal* rc = StockCalibration::find(Function::DcVolts, kRoutes[i].range);
			const double digit = (double)kRoutes[i].spec.unitsPerRaw * rc->nominalCountsPerDigit;
			return (int32_t)std::lround(100 + vin / digit * rc->nominalCountsPerDigit * 1.02);
		}
		return 0;
	};
	CHECK(m.setFunction(Function::DcVolts));
	uint32_t now = 0;
	auto run = [&](uint32_t ms) {
		for (uint32_t i = 0; i < ms; i++, now++) {
			chip.tick(now);
			for (int k = 0; k < 8; k++) dmm.main(now);
			m.main(now);
		}
	};
	run(4000);
	Reading r;
	CHECK(m.latest(&r));
	CHECK(r.range == 0);                                     // 1000 V -> 500 -> 50 -> 5 V
	CHECK((r.status & kValid) && !(r.status & kUncal));
	CHECK(near(r.value, 1.234, 2e-4));
	CHECK(memcmp(&chip.reg[0x20], kDcv0, 20) == 0);
	CHECK(paths.gpio().high & (1u << CTL_A));                // DC paths
	CHECK(chip.reg[hy3131::kInte] == hy3131::kIntAd1);

	vin = 42.0;                                              // up to 50 V
	run(3000);
	m.latest(&r);
	CHECK(r.range == 1 && near(r.value, 42.0, 2e-3));
	CHECK(dmm.faults() == 0);
	CHECK(chip.lostFlags == 0);
}

int main() {
	testRoutes();
	testCalibration();
	testPaths();
	testRangeSwitch();
	testEndToEnd();
	if (g_failures) { printf("%d FAILED\n", g_failures); return 1; }
	printf("all passed\n");
	return 0;
}
