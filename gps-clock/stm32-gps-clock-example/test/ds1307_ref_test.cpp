/*
 * ds1307_ref_test.cpp
 *
 * Host test for Core/Inc/ds1307_ref.h: labelling SQW edges with reads, the
 * falling-edge check, the RAM record, the offset and drift measured
 * against PPS, the prediction a cold start seeds from, and the holdover
 * agreement check. Not part of the firmware build.
 *
 *   g++ -std=c++14 -Wall -Wextra -I../Core/Inc ds1307_ref_test.cpp -o ds1307_ref_test
 */

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include "ds1307_ref.h"

using namespace dsref;

static int g_failures = 0;
static void check(bool ok, const char *what) {
	std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

// A DS1307 whose crystal runs ppm fast, set at UTC t0 + phase (its second
// starts phase seconds after UTC's). Its seconds tick at the falling edge
// of SQW, or (risingTick) half a second later.
struct SimDs {
	double ppm = 0, t0 = 0, phase = 0;
	uint32_t label0 = 0;
	bool risingTick = false;
	// UTC of the falling edge that starts DS1307 second label0 + k
	double edgeUtc(int64_t k) const { return t0 + phase + (double)k / (1 + ppm * 1e-6); }
	// What a read made at UTC u returns
	uint32_t readAt(double u) const {
		const double ds = (u - t0 - phase) * (1 + ppm * 1e-6) - (risingTick ? 0.5 : 0.0);
		return label0 + (uint32_t)(int64_t)std::floor(ds);
	}
};

int main() {
	std::printf("read phase\n");
	{
		const uint32_t e = 0xFFFFF000u;   // across the timer's wrap
		check(readPhase(e, e + 100000, e + 101000) == ReadPhase::Early, "START 100 ms after the edge: early");
		check(readPhase(e, e + 700000, e + 710000) == ReadPhase::Late, "700 ms: late");
		check(readPhase(e, e + 450000, e + 460000) == ReadPhase::Middle, "450 ms: middle");
		check(readPhase(e, e + 5000, e + 6000) == ReadPhase::None, "5 ms after an edge: not trusted");
		check(readPhase(e, e + 995000, e + 996000) == ReadPhase::None, "5 ms before the next: not trusted");
		check(readPhase(e, e + 100000, e + 150000) == ReadPhase::None, "START known only to 50 ms: not used");
		check(edgeIntervalOk(e, e + 1000050) && !edgeIntervalOk(e, e + 500000), "edges a second apart, not half");
	}

	std::printf("falling-edge check\n");
	for (int rising = 0; rising < 2; ++rising) {
		SimDs ds;
		ds.t0 = 1000; ds.phase = 0.3; ds.label0 = 1791763200u; ds.ppm = 30; ds.risingTick = rising != 0;
		LabelCheck lc;
		// alternate early and late reads as the firmware does; edge numbers count from 7
		for (int64_t k = 0; k < 8 && !lc.verified() && !lc.mismatch(); ++k) {
			const ReadPhase want = lc.wanted();
			const double u = ds.edgeUtc(k) + (want == ReadPhase::Early ? 0.15 : 0.75);
			lc.add(want, (int64_t)ds.readAt(u) - (7 + k));
		}
		if (!rising) check(lc.verified() && !lc.mismatch(), "seconds tick at the falling edge: verified in a few reads");
		else check(lc.mismatch() && !lc.verified(), "they tick half a second later: mismatch, SQW not used");
	}

	std::printf("record\n");
	{
		Record r;
		r.offsetUs = -123456789012LL; r.refUnix = 1791763200u; r.driftPpb = -17345; r.spanS = 86400;
		uint8_t b[kRecordLen];
		encode(r, b);
		Record q;
		check(decode(b, &q) && q.offsetUs == r.offsetUs && q.refUnix == r.refUnix && q.driftPpb == r.driftPpb
		      && q.spanS == r.spanS, "round trip, negative values too");
		check(b[0] == 'D' && b[4] == (uint8_t)(r.offsetUs & 0xFF) && b[12] == 0x00 && b[15] == 0x6A,
		      "little-endian: offset's low byte first, refUnix 0x6ACC9A00 as 00 9A CC 6A");
		const uint8_t c9[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
		check(crc8(c9, 9) == 0xF4, "CRC-8/SMBUS of \"123456789\" is 0xF4");
		bool allCaught = true;
		for (int i = 0; i < kRecordLen; ++i) {
			uint8_t x[kRecordLen];
			for (int j = 0; j < kRecordLen; ++j) x[j] = b[j];
			x[i] ^= 0x10;
			if (decode(x, &q)) allCaught = false;
		}
		check(allCaught, "any one changed byte refused");
		uint8_t junk[kRecordLen] = {0};
		check(!decode(junk, &q), "zeroed RAM is no record");
		r.offsetUs = 2000; r.refUnix = 1000; r.driftPpb = 25000;   // 25 ppm fast
		check(predictOffsetUs(r, 1000 + 3600) == 2000 + 90000 && predictOffsetUs(r, 1000 - 40) == 1000,
		      "prediction: 25 ppm is 90 ms an hour, backwards too");
		check(utcAtEdgeUs(r, 1000) == 1000LL * 1000000 - 2000, "UTC at an edge: label less the offset");
		// a year of a 100 ppm crystal: exact against the closed form
		r.offsetUs = 0; r.refUnix = 0; r.driftPpb = 100000;
		const int64_t L = 31536000;
		check(std::llabs(utcAtEdgeUs(r, (uint32_t)L) - (int64_t)std::llround(L * 1e6 / 1.0001)) <= 1,
		      "a year at 100 ppm: counts in the DS1307's seconds, to the microsecond");
	}

	std::printf("drift measured against PPS, then a cold start\n");
	const double ppms[] = {-63.0, -8.2, 0.0, 4.6, 87.5};
	for (double ppm : ppms) {
		SimDs ds;
		const uint32_t U0 = 1791763200u;
		ds.t0 = U0; ds.phase = 0.0473; ds.label0 = U0; ds.ppm = ppm;
		Meter m;
		Record kept;
		bool allFit = true;
		// four hours of edges, timed to the microsecond by TIM2 against the PPS
		for (int64_t k = 0; k < 4 * 3600; ++k) {
			const double u = ds.edgeUtc(k);
			const int64_t utcUs = (int64_t)std::llround(u * 1e6);
			const uint32_t label = ds.label0 + (uint32_t)k;
			if (!m.add(utcUs, (int64_t)label * 1000000 - utcUs)) allFit = false;
		}
		kept = m.record(kept);
		const double driftErr = kept.driftPpb / 1000.0 - ppm;
		// 30 days later the board is powered up without GPS: seed from an edge
		const int64_t kLater = 30LL * 86400;
		const double u = ds.edgeUtc(kLater);
		const uint32_t label = ds.label0 + (uint32_t)kLater;
		uint8_t b[kRecordLen];
		encode(kept, b);
		Record back;
		decode(b, &back);
		const double seedErrUs = (double)utcAtEdgeUs(back, label) - u * 1e6;
		std::printf("    DS1307 %+6.1f ppm: measured %+8.3f ppm over %u s, cold start 30 days on off by %+.0f us"
		            " (unseeded: %+.0f ms)\n", ppm, kept.driftPpb / 1000.0, kept.spanS, seedErrUs,
		            ((double)label - u) * 1e3);
		char what[96];
		std::snprintf(what, sizeof what, "%+.1f ppm: drift within 0.002 ppm, 30 day seed within 5 ms", ppm);
		check(allFit && m.haveDrift() && std::fabs(driftErr) < 0.002 && std::fabs(seedErrUs) < 5000, what);
	}

	std::printf("a write or a wrong label starts the measurement over\n");
	{
		Meter m;
		for (int64_t s = 0; s < 2000; ++s) m.add((1000 + s) * 1000000, 50000 + s * 10);   // 10 ppm
		check(m.haveDrift() && m.driftPpb() == 10000, "10 ppm over 1999 s");
		check(!m.add(3000LL * 1000000, 50000 + 2000 * 10 + 1000000) && m.jumps() == 1 && !m.haveDrift() && m.spanS() == 0,
		      "a second's jump (a label one off): starts over");
		m.add(3001LL * 1000000, 50000 + 2001 * 10 + 1000000);
		check(m.have() && !m.add(3002LL * 1000000, 80000 + 2002 * 10) && m.jumps() == 2, "the DS1307 written (30 ms jump): starts over");
		Record before; before.driftPpb = 7000; before.spanS = 5000;
		const Record r = m.record(before);
		check(r.driftPpb == 7000 && r.spanS == 5000 && r.refUnix == 3002 && r.offsetUs == 80000 + 2002 * 10,
		      "until measured again, the kept drift stays with the new offset");
		Meter u;
		bool fits = true;
		for (int64_t s = 1; s <= 1000; ++s) if (!u.add(s * 1000000, s * 150)) fits = false;   // 150 ppm, not yet measured
		check(fits, "an unmeasured 150 ppm crystal is not taken for jumps");
	}

	std::printf("holdover: do the oscillators agree?\n");
	{
		check(agrees(10000, 6000, 0) && !agrees(12000, 6000, 0), "4 ms apart at once: yes; 6 ms: no");
		check(agrees(150000, 0, 86400) && !agrees(190000, 0, 86400), "a day on: 2 ppm (173 ms) + 5 ms allowed");
	}

	std::printf("%s (%d failure%s)\n", g_failures ? "FAILED" : "PASSED", g_failures, g_failures == 1 ? "" : "s");
	return g_failures ? 1 : 0;
}
