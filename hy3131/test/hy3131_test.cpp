// Host test of hy3131_dmm against a simulated HY3131.
//
// The simulated chip works at the bit level, as the datasheet's section 7
// describes it: a command byte, then for a read one blank clock and the
// registers from the address on, each a bit late. A register counts as read
// as soon as its first bit goes out, so a read that ends seven clocks into
// INTF or CTSTA clears it: the test fails if the driver ever loses a flag or
// CTBOV that way.
//
//   make -C hy3131/test        (or see the Makefile for the include paths)

#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include "iBlockTransport.h"
#include "hy3131.h"

using namespace hy3131;

static int g_failures = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_failures++; } } while (0)

// ---- the simulated chip ----

struct SimChip {
	uint8_t reg[0x38];
	bool present = true;
	uint32_t resets = 0;
	uint32_t nowMs = 0;
	uint32_t lastAd1Ms = 0;
	int32_t ad1Value = -12345;
	int32_t ad2Value = 0x3FFFF;
	int64_t rmsValue = 123456789012LL;
	// counter
	bool counting = false;
	uint32_t countStartMs = 0;
	uint32_t ctbResult = 1000;
	bool ctbovSet = false;
	// what went wrong
	uint32_t lostFlags = 0;        // INTF flags cleared by a read the driver didn't mean
	uint32_t lostCtbov = 0;
	bool eventEveryRead = false;   // an AD1 result lands as each data read starts
	std::vector<std::pair<uint8_t, std::vector<uint8_t>>> writes;

	SimChip() { reset(); }
	void reset() { memset(reg, 0, sizeof reg); resets++; counting = false; }

	uint32_t sysclk() const { return kXtalHz; }
	bool oscOn() const { return reg[kR33] & kR33Enosc; }

	void writeReg(uint8_t a, uint8_t v) {
		if (a >= sizeof reg) return;
		if (a == kR37) { if (v == kResetValue) reset(); return; }
		if (a == kIntf) { reg[kIntf] &= v; return; }   // 0s only
		if (a == kR20) {
			const bool was = reg[kR20] & kR20Enctr, now = v & kR20Enctr;
			if (was && !now) { reg[kCta] = 0; memset(&reg[kCtb], 0, 3); memset(&reg[kCtc], 0, 3); reg[kCtsta] &= ~kCtstaCtbov; counting = false; }
			if (!was && now) { counting = true; countStartMs = nowMs; }
		}
		reg[a] = v;
	}

	// The first bit of register a is going out. partial: the read ends inside it.
	uint8_t readReg(uint8_t a, bool partial) {
		if (a >= sizeof reg) return 0;
		const uint8_t v = reg[a];
		if (a == kIntf) {
			if (partial && (v & (kIntAll | kIntBor))) lostFlags++;
			reg[kIntf] = 0;
		}
		if (a == kCtsta) {
			if (partial && (v & kCtstaCtbov)) lostCtbov++;
			reg[kCtsta] &= ~kCtstaCtbov;
		}
		return v;
	}

	void write(uint8_t cmd, const uint8_t* d, size_t n) {
		uint8_t a = cmd >> 1;
		writes.push_back({a, std::vector<uint8_t>(d, d + n)});
		for (size_t i = 0; i < n; i++) writeReg(a++, d[i]);
	}

	// n bytes clocked after the command: 1 blank bit, then 8n - 1 bits of registers.
	void read(uint8_t cmd, uint8_t* out, size_t n) {
		uint8_t a = cmd >> 1;
		if (eventEveryRead && a != kIntf && (reg[kR23] & kR23Enad1)) reg[kIntf] |= kIntAd1;
		const size_t bits = 8 * n;
		std::vector<uint8_t> stream(bits, 0);
		size_t pos = 1;                       // the blank bit
		while (pos < bits) {
			const bool partial = pos + 8 > bits;
			const uint8_t v = readReg(a++, partial);
			for (int b = 7; b >= 0 && pos < bits; b--) stream[pos++] = (v >> b) & 1;
		}
		for (size_t i = 0; i < n; i++) {
			uint8_t x = 0;
			for (int b = 0; b < 8; b++) x = (uint8_t)((x << 1) | stream[8 * i + b]);
			out[i] = x;
		}
	}

	void set24(uint8_t a, uint32_t v) { reg[a] = v; reg[a + 1] = v >> 8; reg[a + 2] = v >> 16; }

	void tick(uint32_t ms) {
		nowMs = ms;
		if (!oscOn()) return;
		const uint8_t osr = reg[kR22] & 7;
		const uint32_t periodMs = 1000000u / ad1RateMilliHz(sysclk(), osr);
		if ((reg[kR23] & kR23Enad1) && ms - lastAd1Ms >= periodMs) {
			lastAd1Ms = ms;
			set24(kAd1, (uint32_t)ad1Value & 0xFFFFFF);
			set24(kAd2, (uint32_t)ad2Value & 0xFFFFFF);
			for (int i = 0; i < 5; i++) reg[kRms + i] = (uint8_t)(rmsValue >> (8 * i));
			set24(kPkhMin, (uint32_t)-500 & 0xFFFFFF);
			set24(kPkhMax, 700);
			reg[kIntf] |= kIntAd1;
		}
		if (counting) {
			const uint32_t ctaInit = ((uint32_t)reg[kCta + 2] << 16) | ((uint32_t)reg[kCta + 1] << 8);
			const uint32_t gateMs = (uint32_t)((uint64_t)(0x1000000u - ctaInit) * 1000u / sysclk());
			if (ctbovSet && ms - countStartMs >= gateMs / 2) reg[kCtsta] |= kCtstaCtbov;
			if (ms - countStartMs >= gateMs + 1) {
				counting = false;
				set24(kCtb, ctbResult);
				set24(kCtc, ctbResult * 10);
				set24(kCta, 1234);
				reg[kIntf] |= kIntCt;
			}
		}
	}

	bool doHigh() const { return (reg[kIntf] & reg[kInte] & kIntAll) != 0; }
};

// ---- the transport ----

class SimTransport : public iBlockTransport {
public:
	SimChip* chip = nullptr;
	explicit SimTransport(SimChip* c) : chip(c) {}
	bool beginWrite(const uint8_t* h, uint8_t hl, const uint8_t* d, size_t n) override {
		assert(hl == 1 && !(h[0] & 1));
		if (chip->present) chip->write(h[0], d, n);
		return true;
	}
	bool beginRead(const uint8_t* h, uint8_t hl, uint8_t* d, size_t n) override {
		assert(hl == 1 && (h[0] & 1));
		if (chip->present) chip->read(h[0], d, n);
		else memset(d, 0xFF, n);
		return true;
	}
	bool isBusy() const override { return false; }
	bool lastOpFailed() const override { return false; }
};

struct Listener : iListener {
	int samples = 0, counts = 0, faults = 0, running = 0;
	Sample last{}; Count lastCount{};
	void onSample(const Sample& s) override { samples++; last = s; }
	void onCount(const Count& c) override { counts++; lastCount = c; }
	void onFault() override { faults++; }
	void onRunning() override { running++; }
};

static bool doLevel(void* ctx) { return static_cast<SimChip*>(ctx)->doHigh(); }

struct Rig {
	SimChip chip;
	hy3131_dmm<SimTransport> dmm;
	Listener l;
	uint32_t now = 0;
	static hy3131_param_t bind(hy3131_param_t p, SimChip* c) { if (p.doLevel) p.doCtx = c; return p; }
	explicit Rig(hy3131_param_t p) : dmm(bind(p, &chip), &chip) { dmm.setListener(&l); }
	void run(uint32_t ms) {
		for (uint32_t i = 0; i < ms; i++) {
			chip.tick(now);
			for (int k = 0; k < 8; k++) dmm.main(now);
			now++;
		}
	}
};

static hy3131_param_t param(bool level) {
	hy3131_param_t p = hy3131_default_param();
	if (level) { p.doLevel = doLevel; }
	return p;
}

// ---- tests ----

static void testUnshift() {
	printf("unshiftRead\n");
	// blank bit, then 0xA5, 0x3C
	const uint8_t rx[3] = { 0x52, 0x9E, 0x00 };   // 0 1010010 | 1 0011110 | 0...
	uint8_t out[2];
	unshiftRead(rx, out, 2);
	CHECK(out[0] == 0xA5);
	CHECK(out[1] == 0x3C);
	CHECK(le24((const uint8_t*)"\xFF\xFF\xFF") == -1);
	const uint8_t r40[5] = { 0, 0, 0, 0, 0xE0 };
	CHECK(le40(r40) == -(int64_t)0x2000000000LL);
	CHECK(ad1RateMilliHz(kXtalHz, 7) == 5000);
}

static void testStartupAndSample() {
	printf("startup, AD1 sample\n");
	Rig r(param(true));
	r.chip.ad1Value = -12345;
	Config cfg = defaultConfig();
	r.dmm.start(cfg);
	r.run(100);
	CHECK(r.dmm.running());
	CHECK(r.l.running == 1);
	CHECK(r.chip.resets >= 2);   // the constructor's, and R37 = 0x60
	for (uint8_t a = kR20; a <= kR36; a++) CHECK(r.chip.reg[a] == cfg.r(a));
	CHECK(r.chip.reg[kInte] == kIntAd1);
	// The first configuration write holds AD1 off.
	bool sawFirst = false;
	for (auto& w : r.chip.writes) {
		if (w.first == kR20 && w.second.size() == kCfgCount) {
			sawFirst = true;
			CHECK((w.second[cfgIndex(kR23)] & kR23Enad1) == 0);
			CHECK((w.second[cfgIndex(kR33)] & kR33Enosc) != 0);
			break;
		}
	}
	CHECK(sawFirst);
	r.run(1000);   // 5 Hz
	CHECK(r.l.samples >= 4);
	Sample s;
	CHECK(r.dmm.latest(&s));
	CHECK(s.ad1 == -12345);
	CHECK(s.ad2 == 0x3FFFF);
	CHECK(s.rms == 123456789012LL);
	CHECK(s.flags & kIntAd1);
	CHECK(!s.peakValid);
	CHECK(r.chip.lostFlags == 0);
	CHECK(r.dmm.faults() == 0);
}

static void testPollOnly() {
	printf("no IRQ, no DO level: the poll\n");
	Rig r(param(false));
	r.dmm.start(defaultConfig());
	r.run(1200);
	CHECK(r.l.samples >= 4);
	CHECK(r.dmm.faults() == 0);
}

static void testAbsent() {
	printf("no chip\n");
	Rig r(param(true));
	r.chip.present = false;
	r.dmm.start(defaultConfig());
	r.run(100);
	CHECK(!r.dmm.running());
	CHECK(r.dmm.faults() >= 1);
	CHECK(r.l.faults >= 1);
	r.chip.present = true;   // plugged in: it comes up after the back-off
	r.run(1200);
	CHECK(r.dmm.running());
}

static void testBrownOutAndHealth() {
	printf("brown-out, and a chip that resets by itself\n");
	Rig r(param(true));
	r.dmm.start(defaultConfig());
	r.run(100);
	const uint32_t resets = r.chip.resets;
	r.chip.reg[kIntf] |= kIntBor;
	r.run(60);   // the next poll sees it
	CHECK(r.dmm.faults() == 1);
	r.run(1200);
	CHECK(r.dmm.running());
	CHECK(r.chip.resets > resets);

	r.chip.reset();   // lost power: nothing configured, no IRQs
	r.run(1100);      // the health check
	CHECK(r.dmm.faults() == 2);
	r.run(1200);
	CHECK(r.dmm.running());
	CHECK(r.chip.reg[kR23] & kR23Enad1);
}

static void testCounterWithPeak() {
	printf("frequency counter, with peak hold\n");
	Rig r(param(true));
	Config cfg = defaultConfig();
	cfg.counter(true, true, kXtalHz / 10);       // 100 ms gate
	cfg.bit(kR29, kR29Enpkh, true);
	cfg.inte = kIntAd1 | kIntCt;
	r.chip.ctbResult = 4321;
	r.chip.eventEveryRead = true;
	r.dmm.start(cfg);
	r.run(1500);
	CHECK(r.dmm.running());
	CHECK(r.l.counts >= 5);
	Count c;
	CHECK(r.dmm.latestCount(&c));
	CHECK(c.ctb == 4321);
	CHECK(c.ctaInit == ((0x1000000u - kXtalHz / 10) & 0xFFFF00u));
	CHECK(c.ctaFinal == 1234);
	CHECK(!c.overflow);
	const double hz = c.frequencyHz(kXtalHz);
	CHECK(hz > 43000.0 && hz < 43300.0);   // 4321 cycles in ~0.1 s
	Sample s;
	CHECK(r.dmm.latest(&s));
	CHECK(s.peakValid);
	CHECK(s.peakMin == -500 && s.peakMax == 700);
	CHECK(r.chip.lostFlags == 0);
	CHECK(r.chip.lostCtbov == 0);

	printf("counter overflow is reported, not cleared on the way\n");
	r.chip.ctbovSet = true;
	r.run(500);
	CHECK(r.dmm.latestCount(&c));
	CHECK(c.overflow);
	CHECK(r.chip.lostCtbov == 0);
	CHECK(r.dmm.faults() == 0);
}

static void testStop() {
	printf("stop\n");
	Rig r(param(true));
	r.dmm.start(defaultConfig());
	r.run(100);
	r.dmm.stop();
	r.run(10);
	CHECK(r.dmm.driverState() == hy3131_stopped);
	CHECK((r.chip.reg[kR33] & kR33Enosc) == 0);   // reset
}

int main() {
	testUnshift();
	testStartupAndSample();
	testPollOnly();
	testAbsent();
	testBrownOutAndHealth();
	testCounterWithPeak();
	testStop();
	if (g_failures) { printf("%d FAILED\n", g_failures); return 1; }
	printf("all passed\n");
	return 0;
}
