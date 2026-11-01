// Host tests of meter::Meter: against a scripted front end, and end to end
// through Hy3131FrontEnd, hy3131_dmm and the simulated HY3131.

#include <cmath>
#include <cstdio>
#include "Meter.h"
#include "Hy3131FrontEnd.h"
#include "SimHy3131.h"

using namespace meter;

static int g_failures = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_failures++; } } while (0)
static bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// ---- a scripted front end: DC volts in three decade ranges, ohms in one ----

static const RangeSpec kDcv[3] = {
	{ 0.5f,  1e-5f, 0, Unit::Volt, 5, "500mV" },
	{ 5.0f,  1e-4f, 0, Unit::Volt, 4, "5V" },
	{ 50.0f, 1e-3f, 0, Unit::Volt, 3, "50V" },
};
static const RangeSpec kOhm = { 5000.0f, 0.1f, 0, Unit::Ohm, 1, "5k" };

struct FakeFrontEnd : iMeterFrontEnd {
	float    vin = 0.0f;           // the input, SI
	Function fn = Function::None;
	uint8_t  range = 0, tag = 0;
	bool     selected = false;
	uint32_t seq = 0;
	int      selects = 0;
	int      refuse = 0;           // refuse this many select()s
	bool     leakOld = false;      // after a select, first send one result tagged with the old tag
	uint8_t  settle = 0;
	bool     fault = false;
	RawResult pending{};
	bool     havePending = false;

	bool supports(Function f) const override { return f == Function::DcVolts || f == Function::Ohms; }
	uint8_t rangeCount(Function f) const override { return f == Function::DcVolts ? 3 : f == Function::Ohms ? 1 : 0; }
	const RangeSpec* rangeSpec(Function f, uint8_t r) const override {
		if (f == Function::DcVolts && r < 3) return &kDcv[r];
		if (f == Function::Ohms && r == 0) return &kOhm;
		return nullptr;
	}
	uint8_t settleResults(Function, uint8_t) const override { return settle; }
	uint8_t average = 1;
	uint8_t averageResults(Function, uint8_t) const override { return average; }
	bool select(Function f, uint8_t r, uint8_t t) override {
		if (refuse > 0) { refuse--; return false; }
		if (leakOld && selected) { produce(); }   // a result of the old setting, still in the pipe
		fn = f; range = r; tag = t; selected = true; selects++;
		return true;
	}
	void produce() {
		const RangeSpec* s = rangeSpec(fn, range);
		pending.seq = ++seq;
		pending.raw = vin / s->unitsPerRaw;
		pending.overload = false;
		pending.rangeTag = tag;
		havePending = true;
	}
	bool result(uint32_t lastSeq, RawResult* out) override {
		if (!havePending || pending.seq == lastSeq) return false;
		*out = pending;
		return true;
	}
	bool faulted() const override { return fault; }
};

struct Cal : iCalibrationStore {
	bool load(Function f, uint8_t r, Calibration* out) override {
		if (f == Function::DcVolts && r == 1) { *out = Calibration{1.01f, 10.0f, true}; return true; }
		return false;
	}
	bool store(Function, uint8_t, const Calibration&) override { return false; }
};

struct L : iMeterListener {
	int readings = 0, changes = 0, faults = 0;
	void onReading(const Reading&) override { readings++; }
	void onFunctionChanged(Function, uint8_t, bool) override { changes++; }
	void onFault() override { faults++; }
};

// One step: the meter looks, then the front end makes a result.
static void step(Meter& m, FakeFrontEnd& fe, int n = 1) {
	for (int i = 0; i < n; i++) {
		m.main(0);
		if (fe.selected) fe.produce();
		m.main(0);
	}
}

static void testAutoRange() {
	printf("auto range: down from the top, up on overload, OL at the top\n");
	FakeFrontEnd fe;
	Meter m(fe);
	L l; m.setListener(&l);
	CHECK(!m.setFunction(Function::Capacitance));
	fe.vin = 1.234f;
	CHECK(m.setFunction(Function::DcVolts));
	step(m, fe, 10);
	Reading r;
	CHECK(m.latest(&r));
	CHECK(r.function == Function::DcVolts);
	CHECK(r.range == 1);
	CHECK((r.status & kValid) && (r.status & kAutoRange) && (r.status & kUncal));
	CHECK(near(r.value, 1.234f, 1e-4f));
	CHECK(r.unit == Unit::Volt);

	fe.vin = 0.2f;           // down to 500 mV
	step(m, fe, 6);
	m.latest(&r);
	CHECK(r.range == 0);
	CHECK(near(r.value, 0.2f, 1e-5f));

	fe.vin = 30.0f;          // up twice
	step(m, fe, 10);
	m.latest(&r);
	CHECK(r.range == 2);
	CHECK(near(r.value, 30.0f, 1e-2f));

	fe.vin = 70.0f;          // OL on the top range
	step(m, fe, 4);
	m.latest(&r);
	CHECK(r.range == 2);
	CHECK(r.status & kOverload);
	CHECK(!(r.status & kValid));
	CHECK(l.changes >= 4);
}

static void testManualAndMailbox() {
	printf("manual range; the newest request wins\n");
	FakeFrontEnd fe;
	Meter m(fe);
	fe.vin = 1.234f;
	m.setFunction(Function::Ohms);
	m.setFunction(Function::DcVolts);
	m.setRange(0);                       // with the function still waiting: 500 mV of DC volts
	step(m, fe, 5);
	CHECK(fe.selects == 1);
	Reading r;
	m.latest(&r);
	CHECK(r.function == Function::DcVolts && r.range == 0);
	CHECK(r.status & kOverload);         // 1.234 V on 500 mV, and no auto range to save it
	CHECK(!(r.status & kAutoRange));
	m.setRange(9);                       // no such range: ignored
	step(m, fe, 2);
	CHECK(fe.range == 0);
	m.setAutoRange(true);
	step(m, fe, 6);
	m.latest(&r);
	CHECK(r.range == 1 && (r.status & kValid));
}

static void testSettleStaleRefuse() {
	printf("settle results, results of the old selection, a busy front end\n");
	FakeFrontEnd fe;
	Meter m(fe);
	fe.settle = 2;
	fe.leakOld = true;
	fe.refuse = 3;
	fe.vin = 3.0f;
	m.setFunction(Function::DcVolts);
	m.setRange(1);
	for (int i = 0; i < 3; i++) m.main(0);   // refused three times
	CHECK(fe.selects == 0);
	m.main(0);
	CHECK(fe.selects == 1);
	Reading r;
	m.latest(&r);
	CHECK(r.status & kSettling);
	step(m, fe, 1);                           // settle 1
	m.latest(&r);
	CHECK(!(r.status & kValid));
	step(m, fe, 1);                           // settle 2
	step(m, fe, 1);
	m.latest(&r);
	CHECK((r.status & kValid) && near(r.value, 3.0f, 1e-3f));
	// A result tagged with the old selection, after a change, is ignored.
	// With no settling it would otherwise be taken at once: 3,000 counts of
	// the 5 V range read as 30 mV on the 500 mV one.
	fe.settle = 0;
	fe.vin = 0.3f;
	m.setRange(0);
	m.main(0);                                // select: leaks a 5 V-range result with the old tag
	m.main(0);
	m.latest(&r);
	CHECK(r.range == 0 && (r.status & kSettling) && !(r.status & kValid));
	step(m, fe, 1);
	m.latest(&r);
	CHECK((r.status & kValid) && near(r.value, 0.3f, 1e-4f));
}

static void testCalHoldRelMinMax() {
	printf("calibration, hold, relative, min/max\n");
	FakeFrontEnd fe;
	Cal cal;
	Meter m(fe, &cal);
	fe.vin = 2.0f;
	m.setFunction(Function::DcVolts);
	m.setRange(1);
	step(m, fe, 4);
	Reading r;
	m.latest(&r);
	CHECK(!(r.status & kUncal));
	CHECK(near(r.value, (20000.0f - 10.0f) * 1.01f * 1e-4f, 1e-4f));

	m.hold(true);
	step(m, fe, 2);
	m.latest(&r);
	const float held = r.value;
	CHECK(r.status & kHold);
	fe.vin = 3.0f;
	step(m, fe, 3);
	m.latest(&r);
	CHECK(r.value == held && (r.status & kHold));
	m.hold(false);
	step(m, fe, 2);
	m.latest(&r);
	CHECK(!(r.status & kHold) && r.value > held);

	m.relative(true);
	step(m, fe, 2);                           // 3.0 V becomes the zero
	fe.vin = 3.5f;
	step(m, fe, 2);
	m.latest(&r);
	CHECK(r.status & kRelative);
	CHECK(near(r.value, 0.5f * 1.01f, 1e-3f));
	fe.vin = 2.5f;
	step(m, fe, 2);
	m.latest(&r);
	CHECK(near(r.min, -0.5f * 1.01f, 1e-3f));
	CHECK(near(r.max, 0.5f * 1.01f, 1e-3f));
	m.resetMinMax();
	step(m, fe, 1);
	m.latest(&r);
	CHECK(r.count == 1);
}

static void testFault() {
	printf("a front end fault shows, once\n");
	FakeFrontEnd fe;
	Meter m(fe);
	L l; m.setListener(&l);
	fe.vin = 1.0f;
	m.setFunction(Function::DcVolts);
	step(m, fe, 6);
	fe.fault = true;
	fe.selected = false;                      // no results while it recovers
	m.main(0); m.main(0); m.main(0);
	Reading r;
	m.latest(&r);
	CHECK(r.status & kFault);
	CHECK(l.faults == 1);
	fe.fault = false;
	fe.selected = true;
	step(m, fe, 2);
	m.latest(&r);
	CHECK((r.status & kValid) && !(r.status & kFault));
}

static void testAveraging() {
	printf("averaging: four results to a reading; overload at once\n");
	FakeFrontEnd fe;
	Meter m(fe);
	L l; m.setListener(&l);
	fe.average = 4;
	fe.vin = 1.0f;
	m.setFunction(Function::DcVolts);
	m.setRange(1);
	m.main(0);                                   // select
	const int before = l.readings;
	const float v[4] = { 1.0f, 1.2f, 1.0f, 1.2f };
	for (int i = 0; i < 4; i++) { fe.vin = v[i]; step(m, fe, 1); }
	CHECK(l.readings == before + 1);
	Reading r;
	m.latest(&r);
	CHECK((r.status & kValid) && near(r.value, 1.1f, 1e-4f));
	fe.vin = 1.0f; step(m, fe, 1);                // one into the next average
	fe.vin = 9.0f; step(m, fe, 1);                // overload on 5 V: shown now
	m.latest(&r);
	CHECK(r.status & kOverload);
	CHECK(l.readings == before + 2);
}

// ---- end to end: Meter, Hy3131FrontEnd, hy3131_dmm, the simulated chip ----

using namespace hy3131sim;
using Dmm = hy3131_dmm<SimTransport>;

// Board stand-in: DC volts straight into AD1, two ranges by AD1's input gain.
static void dcvHigh(hy3131::Config& c) { c.ad1Input(1); c.field(hy3131::kR25, 3, hy3131::kR25Ad1igShift, 0); }   // x0.9
static void dcvLow(hy3131::Config& c)  { c.ad1Input(1); c.field(hy3131::kR25, 3, hy3131::kR25Ad1igShift, 3); }   // x3.6
static const Hy3131Route kRoutes[] = {
	{ Function::DcVolts, 0, { 1.0f, 1.0f / 40000.0f, 0, Unit::Volt, 5, "1V" }, dcvLow,  Hy3131Source::Ad1, 1, 1 },
	{ Function::DcVolts, 1, { 4.0f, 1.0f / 10000.0f, 0, Unit::Volt, 4, "4V" }, dcvHigh, Hy3131Source::Ad1, 1, 1 },
};

static void testEndToEnd() {
	printf("end to end through the HY3131 front end\n");
	SimChip chip;
	hy3131_param_t p = hy3131_default_param();
	p.doLevel = doLevel;
	p.doCtx = &chip;
	Dmm dmm(p, &chip);
	Hy3131FrontEnd<Dmm> fe(dmm, kRoutes, 2);
	Meter m(fe);
	float vin = 0.5f;
	// AD1 = vin x 10,000 counts per volt x the input gain step (0.9, 1.8, 2.7, 3.6 -> 1..4)
	chip.ad1Model = [&vin](const uint8_t* reg) {
		const int ig = (reg[hy3131::kR25] >> hy3131::kR25Ad1igShift) & 3;
		return (int32_t)std::lround(vin * 10000.0f * (float)(ig + 1));
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
	run(3000);
	Reading r;
	CHECK(m.latest(&r));
	CHECK(r.range == 0);                    // started on 4 V, ranged down
	CHECK(r.status & kValid);
	CHECK(near(r.value, 0.5f, 1e-4f));
	CHECK((chip.reg[hy3131::kR25] >> hy3131::kR25Ad1igShift & 3) == 3);
	CHECK(chip.reg[hy3131::kInte] == hy3131::kIntAd1);

	vin = 2.0f;                             // 80,000 counts on the 1 V range: up
	run(3000);
	m.latest(&r);
	CHECK(r.range == 1);
	CHECK((r.status & kValid) && near(r.value, 2.0f, 1e-3f));
	CHECK(dmm.faults() == 0);
	CHECK(chip.lostFlags == 0);
}

int main() {
	testAutoRange();
	testManualAndMailbox();
	testSettleStaleRefuse();
	testCalHoldRelMinMax();
	testFault();
	testAveraging();
	testEndToEnd();
	if (g_failures) { printf("%d FAILED\n", g_failures); return 1; }
	printf("all passed\n");
	return 0;
}
